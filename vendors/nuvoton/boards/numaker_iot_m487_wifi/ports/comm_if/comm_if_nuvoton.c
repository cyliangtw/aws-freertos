/*
 * Amazon FreeRTOS CELLULAR Preview Release
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* The config header is always included first. */
#include "iot_config.h"

/* Standard includes. */
#include <string.h>

#include "platform/iot_threads.h"
#include "event_groups.h"

/* Cellular includes. */
#include "aws_cellular_config.h"
#include "cellular_config_defaults.h"
#include "cellular_comm_interface.h"

/* Hardware and starter kit includes. */
#include "NuMicro.h"


/* Configure logs for the functions in this file. */
#ifdef IOT_LOG_LEVEL_GLOBAL
    #define LIBRARY_LOG_LEVEL    IOT_LOG_LEVEL_GLOBAL
#else
    #define LIBRARY_LOG_LEVEL    IOT_LOG_NONE
#endif

#define LIBRARY_LOG_NAME    ( "COMM_IF_NVT" )
#include "iot_logging_setup.h"

/*-----------------------------------------------------------*/

/* CELLULAR uses UART1 as main communication */
#define CELLULAR_UART               UART1
#define CELLULAR_UART_IRQ           UART1_IRQn
#define CELLULAR_UART_BAUDRATE      115200

/**
 * @brief Comm events.
 */
#define COMM_EVT_MASK_RX_DONE       ( 0x0001U )
#define COMM_EVT_MASK_RX_TIMEOUT    ( 0x0002U )

//#define DEFAULT_WAIT_INTERVAL       ( 20UL )
#define DEFAULT_RECV_WAIT_INTERVAL  ( 5UL )
//#define DEFAULT_RECV_TIMEOUT        ( 1000UL )

#define TICKS_TO_MS( xTicks )       ( ( ( xTicks ) * 1000U ) / ( ( uint32_t ) configTICK_RATE_HZ ) )

/*-----------------------------------------------------------*/

/**
 * @brief A context of the communication interface.
 */
typedef struct CellularCommInterfaceContextStruct
{
    UART_T * uart;                                  /**< Handle of physical interface. */
    EventGroupHandle_t pEventGroup;                 /**< EventGroup for processing tx/rx. */
    uint32_t lastErrorCode;                         /**< Last error codes (bit-wised) of physical interface. */
    uint8_t uartBusyFlag;                           /**< Flag for whether the physical interface is busy or not. */
    CellularCommInterfaceReceiveCallback_t pRecvCB; /**< Callback function of notify RX data. */
    void * pUserData;                               /**< Userdata to be provided in the callback. */
    uint8_t rxFifoReadingFlag;                      /**< Flag for whether the receiver is currently reading the buffer. */
    bool ifOpen;                                    /**< Communicate interface open status. */
} CellularCommInterfaceContext;

/*-----------------------------------------------------------*/

static CellularCommInterfaceError_t prvCellularOpen( CellularCommInterfaceReceiveCallback_t receiveCallback,
                                            void * pUserData,
                                            CellularCommInterfaceHandle_t * pCommInterfaceHandle );
static CellularCommInterfaceError_t prvCellularClose( CellularCommInterfaceHandle_t commInterfaceHandle );
static CellularCommInterfaceError_t prvCellularReceive( CellularCommInterfaceHandle_t commInterfaceHandle,
                                            uint8_t * pBuffer,
                                            uint32_t bufferLength,
                                            uint32_t timeoutMilliseconds,
                                            uint32_t * pDataReceivedLength );
static CellularCommInterfaceError_t prvCellularSend( CellularCommInterfaceHandle_t commInterfaceHandle,
                                            const uint8_t * pData,
                                            uint32_t dataLength,
                                            uint32_t timeoutMilliseconds,
                                            uint32_t * pDataSentLength );

/*-----------------------------------------------------------*/

/* Static Linked communication interface. */
/* This variable is defined as communication interface. */
/* coverity[misra_c_2012_rule_8_7_violation]. */
CellularCommInterface_t CellularCommInterface =
{
    .open = prvCellularOpen,
    .send = prvCellularSend,
    .recv = prvCellularReceive,
    .close = prvCellularClose
};

/*-----------------------------------------------------------*/

static CellularCommInterfaceContext _iotCommIntfCtx = { 0 };

/*-----------------------------------------------------------*/

/* Uart rx buffer control */
#define RX_BUF_SIZE         1600
#define RX_BUF_RESET()      (rx_buf_ridx = rx_buf_widx = 0)

static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_buf_ridx, rx_buf_widx;

void RX_BUF_PUSH( uint8_t d )
{
    rx_buf[rx_buf_widx++] = d;
    if( rx_buf_widx == RX_BUF_SIZE )
    {
        rx_buf_widx = 0;
    }
}

uint8_t RX_BUF_POP( void )
{
    uint8_t d = rx_buf[rx_buf_ridx++];

    if( rx_buf_ridx == RX_BUF_SIZE )
    {
        rx_buf_ridx = 0;
    }

    return d;
}

uint8_t RX_BUF_FULL( void )
{
    if( ( rx_buf_widx == ( RX_BUF_SIZE - 1 ) ) && ( rx_buf_ridx == 0 ) )
    {
        return 1;
    }

    if( rx_buf_ridx == ( rx_buf_widx + 1 ) )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint16_t RX_BUF_COUNT( void )
{
    if( rx_buf_widx >= rx_buf_ridx )
    {
        return ( rx_buf_widx - rx_buf_ridx );
    }
    else
    {
        return ( RX_BUF_SIZE + rx_buf_widx - rx_buf_ridx );
    }
}


/**
 * @brief  Initialize platform
 */
BaseType_t Cellular_Uart_Init( UART_T * uart )
{
    if( uart == ( UART_T * ) UART1 )
    {
        CLK_EnableModuleClock( UART1_MODULE );
        /* Select UART1 clock source is HXT */
        CLK_SetModuleClock( UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1( 1 ) );
        /* Set PB multi-function pins for UART1 RXD, TXD */
        SYS->GPB_MFPL &= ~ ( SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk );
        SYS->GPB_MFPL |= ( SYS_GPB_MFPL_PB3MFP_UART1_TXD | SYS_GPB_MFPL_PB2MFP_UART1_RXD );
    }
    else
    {
        IotLogError( "Do not support MFP setting of UART_BASE 0x%p !\n", uart );
        return pdFALSE;
    }

    /* Configure UART */
    UART_Open( uart, CELLULAR_UART_BAUDRATE );
    /* Set RX Trigger Level = 8 */
    uart->FIFO = ( uart->FIFO &~ UART_FIFO_RFITL_Msk ) | UART_FIFO_RFITL_8BYTES;
    /* Set Timeout time 0x3E bit-time */
    UART_SetTimeoutCnt( uart, 0x3E );
    /* enable uart */
    UART_EnableInt( uart, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk );
    /* UART interrupt init */
    NVIC_SetPriority( CELLULAR_UART_IRQ, 5 );
    NVIC_EnableIRQ( CELLULAR_UART_IRQ );

    return pdTRUE;
}

void Cellular_Uart_DeInit( UART_T * uart )
{
    UART_Close( uart );
}

/**
 * @brief  Write data to UART interface
 */
uint32_t Cellular_Uart_Send( UART_T * uart, uint8_t pucTxBuf[], uint16_t usWriteBytes )
{
    IotLogDebug( "[%s] %s", __func__, pucTxBuf );

    return UART_Write( uart, pucTxBuf, usWriteBytes );
}

/**
 * @brief  UART interrupt handler for RX
 */
void UART1_IRQHandler( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult = pdPASS;
    CellularCommInterfaceContext * pIotCommIntfCtx = & _iotCommIntfCtx;
    uint8_t ucDat;

    while( ! UART_GET_RX_EMPTY( pIotCommIntfCtx->uart ) )
    {
        if( RX_BUF_FULL() )
        {
            IotLogError( "[%s] RX buffer full !!\n", __func__ );
        }
        ucDat = UART_READ( pIotCommIntfCtx->uart );
        RX_BUF_PUSH( ucDat );
    }

    /* rxFifoReadingFlag indicate the reader is reading the FIFO in recv function.
     * Don't call the callback function until the reader finish read. */
    if( pIotCommIntfCtx->rxFifoReadingFlag == 0U )
    {
        if( pIotCommIntfCtx->pRecvCB != NULL )
        {
            pIotCommIntfCtx->pRecvCB( pIotCommIntfCtx->pUserData,
                                      ( CellularCommInterfaceHandle_t ) pIotCommIntfCtx );
        }
    }
    else
    {
        xResult = xEventGroupSetBitsFromISR( pIotCommIntfCtx->pEventGroup,
                                             COMM_EVT_MASK_RX_DONE,
                                             & xHigherPriorityTaskWoken );
        if( xResult == pdPASS )
        {
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }
    }
}

static CellularCommInterfaceError_t prvCellularOpen( CellularCommInterfaceReceiveCallback_t receiveCallback,
                                           void * pUserData,
                                           CellularCommInterfaceHandle_t * pCommInterfaceHandle )
{
    CellularCommInterfaceError_t ret = IOT_COMM_INTERFACE_SUCCESS;
    CellularCommInterfaceContext * pIotCommIntfCtx = & _iotCommIntfCtx;

    /* check input parameter. */
    if( ( pCommInterfaceHandle == NULL ) || ( receiveCallback == NULL ) )
    {
        ret = IOT_COMM_INTERFACE_BAD_PARAMETER;
    }
    else if( pIotCommIntfCtx->ifOpen == true )
    {
        ret = IOT_COMM_INTERFACE_FAILURE;
    }
    else
    {
        /* Initialize the context structure. */
        ( void ) memset( pIotCommIntfCtx, 0, sizeof( CellularCommInterfaceContext ) );
    }

    /* Setup the UART device. */
    if( ret == IOT_COMM_INTERFACE_SUCCESS )
    {
        pIotCommIntfCtx->uart = ( UART_T * ) CELLULAR_UART;
        if( Cellular_Uart_Init( pIotCommIntfCtx->uart ) != pdTRUE )
        {
            IotLogError( "UART init failed" );
            vEventGroupDelete( pIotCommIntfCtx->pEventGroup );
            ret = IOT_COMM_INTERFACE_DRIVER_ERROR;
        }
    }

    /* Setup the read FIFO. */
    if( ret == IOT_COMM_INTERFACE_SUCCESS )
    {
        RX_BUF_RESET();
        pIotCommIntfCtx->pEventGroup = xEventGroupCreate();
        if( pIotCommIntfCtx->pEventGroup == NULL )
        {
            IotLogError( "EventGroup create failed" );
            ret = IOT_COMM_INTERFACE_NO_MEMORY;
        }
    }

    /* setup callback function and userdata. */
    if( ret == IOT_COMM_INTERFACE_SUCCESS )
    {
        pIotCommIntfCtx->pRecvCB = receiveCallback;
        pIotCommIntfCtx->pUserData = pUserData;
        * pCommInterfaceHandle = ( CellularCommInterfaceHandle_t ) pIotCommIntfCtx;
        pIotCommIntfCtx->ifOpen = true;
    }

    return ret;
}

static CellularCommInterfaceError_t prvCellularClose( CellularCommInterfaceHandle_t commInterfaceHandle )
{
    CellularCommInterfaceError_t ret = IOT_COMM_INTERFACE_BAD_PARAMETER;
    CellularCommInterfaceContext * pIotCommIntfCtx = ( CellularCommInterfaceContext * ) commInterfaceHandle;

    if( pIotCommIntfCtx == NULL )
    {
        ret = IOT_COMM_INTERFACE_BAD_PARAMETER;
    }
    else if( pIotCommIntfCtx->ifOpen == false )
    {
        ret = IOT_COMM_INTERFACE_FAILURE;
    }
    else
    {
        /* Disable UART RX and TX interrupts */
        if( pIotCommIntfCtx->uart != NULL )
        {
            ( void ) Cellular_Uart_DeInit( pIotCommIntfCtx->uart );
            pIotCommIntfCtx->uart = NULL;
        }

        /* Clean event group. */
        if( pIotCommIntfCtx->pEventGroup != NULL )
        {
            vEventGroupDelete( pIotCommIntfCtx->pEventGroup );
            pIotCommIntfCtx->pEventGroup = NULL;
        }

        /* Set the device open status to false. */
        pIotCommIntfCtx->ifOpen = false;

        ret = IOT_COMM_INTERFACE_SUCCESS;
    }

    return ret;
}

static CellularCommInterfaceError_t prvCellularSend( CellularCommInterfaceHandle_t commInterfaceHandle,
                                           const uint8_t * pData,
                                           uint32_t dataLength,
                                           uint32_t timeoutMilliseconds,
                                           uint32_t * pDataSentLength )
{
    CellularCommInterfaceError_t ret = IOT_COMM_INTERFACE_BUSY;
    CellularCommInterfaceContext * pIotCommIntfCtx = ( CellularCommInterfaceContext * ) commInterfaceHandle;
    TickType_t xTickCurrent, xTickTimeout;
    uint32_t transferSize = 0;

    if( ( pIotCommIntfCtx == NULL ) || ( pData == NULL ) || ( dataLength == 0 ) )
    {
        ret = IOT_COMM_INTERFACE_BAD_PARAMETER;
    }
    else if( pIotCommIntfCtx->ifOpen == false )
    {
        ret = IOT_COMM_INTERFACE_FAILURE;
    }
    else
    {
        xTickTimeout = xTaskGetTickCount() + pdMS_TO_TICKS( timeoutMilliseconds );

        do {
            transferSize += Cellular_Uart_Send( pIotCommIntfCtx->uart, ( uint8_t * ) pData, ( uint16_t ) dataLength );
            if( transferSize >= dataLength )
            {
                break;
            }
            xTickCurrent = xTaskGetTickCount();
        } while ( xTickCurrent < xTickTimeout );

        if( transferSize >= dataLength )
        {
            ret = IOT_COMM_INTERFACE_SUCCESS;
        }
        else
        {
            ret = IOT_COMM_INTERFACE_TIMEOUT;
        }

        if( pDataSentLength != NULL )
        {
            * pDataSentLength = transferSize;
        }
    }

    return ret;
}

static CellularCommInterfaceError_t prvCellularReceive( CellularCommInterfaceHandle_t commInterfaceHandle,
                                              uint8_t * pBuffer,
                                              uint32_t bufferLength,
                                              uint32_t timeoutMilliseconds,
                                              uint32_t * pDataReceivedLength )
{
    CellularCommInterfaceError_t ret = IOT_COMM_INTERFACE_SUCCESS;
    CellularCommInterfaceContext * pIotCommIntfCtx = ( CellularCommInterfaceContext * ) commInterfaceHandle;
    const uint32_t waitInterval = DEFAULT_RECV_WAIT_INTERVAL;
    EventBits_t uxBits = 0;
    uint8_t rxChar = 0;
    uint32_t rxCount = 0;
    uint32_t waitTimeMs = 0, elapsedTimeMs = 0;
    uint32_t remainTimeMs = timeoutMilliseconds;
    uint32_t startTimeMs = TICKS_TO_MS( xTaskGetTickCount() );

    if( ( pIotCommIntfCtx == NULL ) || ( pBuffer == NULL ) || ( bufferLength == 0 ) )
    {
        ret = IOT_COMM_INTERFACE_BAD_PARAMETER;
    }
    else if( pIotCommIntfCtx->ifOpen == false )
    {
        ret = IOT_COMM_INTERFACE_FAILURE;
    }
    else
    {
        /* Set this flag to inform interrupt handler to stop callling callback function. */
        pIotCommIntfCtx->rxFifoReadingFlag = 1U;

        ( void ) xEventGroupClearBits( pIotCommIntfCtx->pEventGroup,
                                       COMM_EVT_MASK_RX_DONE |
                                       COMM_EVT_MASK_RX_TIMEOUT );

        while( rxCount < bufferLength )
        {
            /* If data received, reset timeout. */
            if( RX_BUF_COUNT() > 0 )
            {
                * pBuffer = RX_BUF_POP();
                pBuffer++;
                rxCount++;
            }
            else if( remainTimeMs > 0U )
            {
                if( rxCount > 0U )
                {
                    /* If bytes received, wait at most waitInterval. */
                    waitTimeMs = ( remainTimeMs > waitInterval ) ? waitInterval : remainTimeMs;
                }
                else
                {
                    waitTimeMs = remainTimeMs;
                }

                uxBits = xEventGroupWaitBits( pIotCommIntfCtx->pEventGroup,
                                              COMM_EVT_MASK_RX_DONE |
                                              COMM_EVT_MASK_RX_TIMEOUT,
                                              pdTRUE,
                                              pdFALSE,
                                              pdMS_TO_TICKS( waitTimeMs ) );
                if( ( uxBits & COMM_EVT_MASK_RX_DONE ) != 0U )
                {
                    elapsedTimeMs = TICKS_TO_MS( xTaskGetTickCount() ) - startTimeMs;
                    if( timeoutMilliseconds > elapsedTimeMs )
                    {
                        remainTimeMs = timeoutMilliseconds - elapsedTimeMs;
                    }
                    else
                    {
                        remainTimeMs = 0U;
                    }
                }
                else
                {
                    ret = IOT_COMM_INTERFACE_TIMEOUT;
                }
            }
            else
            {
                /* The timeout case. */
                ret = IOT_COMM_INTERFACE_TIMEOUT;
            }
            if( ret != IOT_COMM_INTERFACE_SUCCESS )
            {
                break;
            }
        }

        /* Clear this flag to inform interrupt handler to call callback function. */
        pIotCommIntfCtx->rxFifoReadingFlag = 0U;

        * pDataReceivedLength = rxCount;
        /* Return success if bytes received. Even if timeout or RX error. */
        if( rxCount > 0 )
        {
            ret = IOT_COMM_INTERFACE_SUCCESS;
        }
    }

    return ret;
}

/* missed function in Keil for file cellular_at_core.c */
size_t strnlen(const char* s, size_t maxlen)
{
    size_t len = 0;

    while( ( len <= maxlen ) && ( *s ) )
    {
        s++;
        len++;
    }

    return len;
}
