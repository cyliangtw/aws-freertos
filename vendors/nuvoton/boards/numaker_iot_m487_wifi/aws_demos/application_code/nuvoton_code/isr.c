/*************************************************************************//**
 * @file     isr.c
 * @version  V0.10
 * @brief    M480 ISR source file
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "ff.h"

#include "config.h"


extern uint32_t volatile u32AudioBuffPos;

static uint32_t PcmBuff[BUFF_LEN] = {0};

uint32_t audioRxWriterPos=0;
uint32_t audioRxReaderPos=0;
uint32_t audioRxCnt=0;
uint32_t audioTxCnt=0;
uint32_t rxFIFOLevel;
extern FIL* gAudiohFile;

void Audio_TX()
{
    uint32_t u32Reg;
    uint32_t u32Len, i;
    uint32_t *pBuffTx, *pBuffRx;
    uint32_t  sendBuf[128];
    UINT retBytes;
  
#if 1 // Send to I2S output  

    if ((audioRxCnt - audioTxCnt) >= 16)
    {
            // if overflow, reset TX pointer & counter 
            if( (audioRxCnt - audioTxCnt) >  BUFF_LEN )
            {
              printf("### In %s: Overflow and reset audioTxCnt from [%d] ", __FUNCTION__, audioTxCnt);
              audioRxReaderPos = 0;
              audioTxCnt = audioRxCnt - BUFF_LEN;
              printf(" to [%d]\n", audioTxCnt);
            }
            u32Len = 16 - I2S_GET_TX_FIFO_LEVEL(I2S0);      
            for (i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, PcmBuff[audioRxReaderPos++]);
                if( audioRxReaderPos >= BUFF_LEN) 
                {  
                  audioRxReaderPos = 0;
                }
            }
            audioTxCnt += u32Len;
     } else
     {
            u32Len = 16 - I2S_GET_TX_FIFO_LEVEL(I2S0);
            for (i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, 0x00);
            }
     }
     

#else // Send to Cloud
      // Set 128 bytes per packet to transfer
      while ((audioRxCnt - audioTxCnt) >= 128)
      {   
            // if overflow, reset TX pointer & counter 
            if( (audioRxCnt - audioTxCnt) >  BUFF_LEN )
            {
              printf("### In %s: Overflow and reset audioTxCnt from [%d] ", __FUNCTION__, audioTxCnt);
              audioRxReaderPos = 0;
              audioTxCnt = audioRxCnt - BUFF_LEN;
              printf(" to [%d]\n", audioTxCnt);
            }
            for (i = 0; i < 128; i++)
            {
                // copy to send_buf
                sendBuf[i] = PcmBuff[audioRxReaderPos++];
                if( audioRxReaderPos >= BUFF_LEN) 
                {  
                  audioRxReaderPos = 0;
                }
            }
            // Here, to deliver send buffer to Cloud by MQTT or HTTP. Or deliver by another task/thread
            if(gAudiohFile != NULL) f_write(gAudiohFile, sendBuf, sizeof(sendBuf), &retBytes);
            audioTxCnt += 128;            
      }
      printf("### In %s: gAudiohFile=0x%x, audioTxCnt=[%d]\n", __FUNCTION__, gAudiohFile, audioTxCnt);
#endif
}

void I2S0_IRQHandler(void)
{
    uint32_t u32Reg;
    uint32_t u32Len, i;
    uint32_t *pBuffTx, *pBuffRx;

    // enable sound output
    PA4 = 0;

    u32Reg = I2S_GET_INT_FLAG(I2S0, I2S_STATUS0_TXTHIF_Msk | I2S_STATUS0_RXTHIF_Msk);
#if 0
    if (u32Reg & I2S_STATUS0_TXTHIF_Msk)
    {
        pBuffTx = &PcmBuff[0];

        /* Read Tx FIFO free size */
        u32Len = 8 - I2S_GET_TX_FIFO_LEVEL(I2S0);

        if (u32AudioBuffPos >= 16) //8)
        {
            for (i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, pBuffTx[i]);
            }

            for (i = 0; i < BUFF_LEN - u32Len; i++)
            {
                pBuffTx[i] = pBuffTx[i + u32Len];
            }

            u32AudioBuffPos -= u32Len;
        }
        else
        {
            for (i = 0; i < u32Len; i++)
            {
                I2S_WRITE_TX_FIFO(I2S0, 0x00);
            }
        }
    }
#else
    Audio_TX();
#endif
    if (u32Reg & I2S_STATUS0_RXTHIF_Msk)
    {
#if 0
      if (u32AudioBuffPos < (BUFF_LEN -16)) //-8))
        {
            pBuffRx = &PcmBuff[u32AudioBuffPos];

            /* Read Rx FIFO Level */
            u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);
            rxFIFOLevel = u32Len;
            for ( i = 0; i < u32Len; i++ )
            {
                pBuffRx[i] = I2S_READ_RX_FIFO(I2S0);
            }
            u32AudioBuffPos += u32Len;

            if (u32AudioBuffPos >= BUFF_LEN)
            {
                u32AudioBuffPos =    0;
            }
        }
#else
        {
            static unsigned int isrtCnt=0;
            pBuffRx = &PcmBuff[u32AudioBuffPos];

            /* Read Rx FIFO Level */
            u32Len = I2S_GET_RX_FIFO_LEVEL(I2S0);
            rxFIFOLevel = u32Len;
            if(!((isrtCnt++)%0x100)) printf("RX_FIFO_LEVEL=%d, audioRxCnt=0x%x\n", u32Len, audioRxCnt);
            for ( i = 0; i < u32Len; i++ )
            {
                PcmBuff[u32AudioBuffPos++] = I2S_READ_RX_FIFO(I2S0);
                if( u32AudioBuffPos >= BUFF_LEN)
                {
                  u32AudioBuffPos = 0;
                }
            }
            audioRxWriterPos = u32AudioBuffPos;
            audioRxCnt += u32Len;   
        }
#endif
        
    }
}

void I2S0_ResetAudioBuffer(void)
{
    u32AudioBuffPos=0 ;
}

void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        g_u8SDDataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if (isr & SDH_INTSTS_CDIF_Msk)   // port 0 card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for (i=0; i<0x500; i++);  // delay to make sure got updated value from REG_SDISR.
            isr = SDH0->INTSTS;
        }

        if (isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!g_u8R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
