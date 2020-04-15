/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * @brief    This is an I2S demo using NAU8822/88L25 audio codec, and used to play
 *           back the input from line-in or MIC interface..
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "FreeRTOS.h"
#include "diskio.h"
#include "ff.h"

#define NAU8822     0

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[16] ;                   /* Working buffer */
#else
BYTE Buff[16] __attribute__((aligned(32)));       /* Working buffer */
#endif

uint32_t volatile u32AudioBuffPos = 0;

#ifdef NVT_AUDIO

#if NAU8822

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C2                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{

    I2C_START(I2C2);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, 0x1A<<1);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_STOP(I2C2);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup()
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    vTaskDelay(10000/1000); 

    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
}

#else   // NAU88L25

uint8_t I2cWrite_MultiByteforNAU88L25(uint8_t chipadd,uint16_t subaddr, const uint8_t *p,uint32_t len)
{
    /* Send START */
    I2C_START(I2C2);
    I2C_WAIT_READY(I2C2);

    /* Send device address */
    I2C_SET_DATA(I2C2, chipadd);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C2, (uint8_t)(subaddr>>8));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C2, (uint8_t)(subaddr));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send data */
    I2C_SET_DATA(I2C2, p[0]);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send data */
    I2C_SET_DATA(I2C2, p[1]);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send STOP */
    I2C_STOP(I2C2);

    return  0;
}

uint8_t I2C_WriteNAU88L25(uint16_t addr,uint16_t dat)
{
    uint8_t Tx_Data0[2];

    Tx_Data0[0] = (uint8_t)(dat >> 8);
    Tx_Data0[1] = (uint8_t)(dat & 0x00FF);

    return ( I2cWrite_MultiByteforNAU88L25(0x1A << 1,addr,&Tx_Data0[0],2) );
}

void NAU88L25_Reset(void)
{
    I2C_WriteNAU88L25(0,  0x1);
    I2C_WriteNAU88L25(0,  0);   // Reset all registers
    vTaskDelay(10000/1000); 

    printf("NAU88L25 Software Reset.\n");
}

void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0x0003,  0x8053);
    I2C_WriteNAU88L25(0x0004,  0x0001);
    I2C_WriteNAU88L25(0x0005,  0x3126);
    I2C_WriteNAU88L25(0x0006,  0x0008);
    I2C_WriteNAU88L25(0x0007,  0x0010);
    I2C_WriteNAU88L25(0x0008,  0xC000);
    I2C_WriteNAU88L25(0x0009,  0x6000);
    I2C_WriteNAU88L25(0x000A,  0xF13C);
    I2C_WriteNAU88L25(0x000C,  0x0048);
    I2C_WriteNAU88L25(0x000D,  0x0000);
    I2C_WriteNAU88L25(0x000F,  0x0000);
    I2C_WriteNAU88L25(0x0010,  0x0000);
    I2C_WriteNAU88L25(0x0011,  0x0000);
    I2C_WriteNAU88L25(0x0012,  0xFFFF);
    I2C_WriteNAU88L25(0x0013,  0x0015);
    I2C_WriteNAU88L25(0x0014,  0x0110);
    I2C_WriteNAU88L25(0x0015,  0x0000);
    I2C_WriteNAU88L25(0x0016,  0x0000);
    I2C_WriteNAU88L25(0x0017,  0x0000);
    I2C_WriteNAU88L25(0x0018,  0x0000);
    I2C_WriteNAU88L25(0x0019,  0x0000);
    I2C_WriteNAU88L25(0x001A,  0x0000);
    I2C_WriteNAU88L25(0x001B,  0x0000);
    I2C_WriteNAU88L25(0x001C,  0x0002);
    I2C_WriteNAU88L25(0x001D,  0x301a);   //301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K
    I2C_WriteNAU88L25(0x001E,  0x0000);
    I2C_WriteNAU88L25(0x001F,  0x0000);
    I2C_WriteNAU88L25(0x0020,  0x0000);
    I2C_WriteNAU88L25(0x0021,  0x0000);
    I2C_WriteNAU88L25(0x0022,  0x0000);
    I2C_WriteNAU88L25(0x0023,  0x0000);
    I2C_WriteNAU88L25(0x0024,  0x0000);
    I2C_WriteNAU88L25(0x0025,  0x0000);
    I2C_WriteNAU88L25(0x0026,  0x0000);
    I2C_WriteNAU88L25(0x0027,  0x0000);
    I2C_WriteNAU88L25(0x0028,  0x0000);
    I2C_WriteNAU88L25(0x0029,  0x0000);
    I2C_WriteNAU88L25(0x002A,  0x0000);
    I2C_WriteNAU88L25(0x002B,  0x0012);
    I2C_WriteNAU88L25(0x002C,  0x0082);
    I2C_WriteNAU88L25(0x002D,  0x0000);
    I2C_WriteNAU88L25(0x0030,  0x00CF);
    I2C_WriteNAU88L25(0x0031,  0x0000);
    I2C_WriteNAU88L25(0x0032,  0x0000);
#if 0
    I2C_WriteNAU88L25(0x0033,  0x009E);
    I2C_WriteNAU88L25(0x0034,  0x029E);
#else // For bigger volume
    I2C_WriteNAU88L25(0x0033,  0x00CF);
    I2C_WriteNAU88L25(0x0034,  0x02CF);
#endif
    I2C_WriteNAU88L25(0x0038,  0x1486);
    I2C_WriteNAU88L25(0x0039,  0x0F12);
    I2C_WriteNAU88L25(0x003A,  0x25FF);
    I2C_WriteNAU88L25(0x003B,  0x3457);
    I2C_WriteNAU88L25(0x0045,  0x1486);
    I2C_WriteNAU88L25(0x0046,  0x0F12);
    I2C_WriteNAU88L25(0x0047,  0x25F9);
    I2C_WriteNAU88L25(0x0048,  0x3457);
    I2C_WriteNAU88L25(0x004C,  0x0000);
    I2C_WriteNAU88L25(0x004D,  0x0000);
    I2C_WriteNAU88L25(0x004E,  0x0000);
    I2C_WriteNAU88L25(0x0050,  0x2007);
    I2C_WriteNAU88L25(0x0051,  0x0000);
    I2C_WriteNAU88L25(0x0053,  0xC201);
    I2C_WriteNAU88L25(0x0054,  0x0C95);
    I2C_WriteNAU88L25(0x0055,  0x0000);
    I2C_WriteNAU88L25(0x0058,  0x1A14);
    I2C_WriteNAU88L25(0x0059,  0x00FF);
    I2C_WriteNAU88L25(0x0066,  0x0060);
    I2C_WriteNAU88L25(0x0068,  0xC300);
    I2C_WriteNAU88L25(0x0069,  0x0000);
    I2C_WriteNAU88L25(0x006A,  0x0083);
    I2C_WriteNAU88L25(0x0071,  0x0011);
    I2C_WriteNAU88L25(0x0072,  0x0260);
    I2C_WriteNAU88L25(0x0073,  0x332C);
    I2C_WriteNAU88L25(0x0074,  0x4502);
    I2C_WriteNAU88L25(0x0076,  0x3140);
    I2C_WriteNAU88L25(0x0077,  0x0000);
    I2C_WriteNAU88L25(0x007F,  0x553F);
    I2C_WriteNAU88L25(0x0080,  0x0420);
    I2C_WriteNAU88L25(0x0001,  0x07D4);

    printf("NAU88L25 Configured done.\n");
}

/* config play sampling rate */
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU88L25] Configure Sampling Rate to %d\n", u32SampleRate);

    if((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU88L25(0x0005, 0x3126); //12.288Mhz
        I2C_WriteNAU88L25(0x0006, 0x0008);
    }
    else
    {
        I2C_WriteNAU88L25(0x0005, 0x86C2); //11.2896Mhz
        I2C_WriteNAU88L25(0x0006, 0x0007);
    }

    switch (u32SampleRate)
    {
    case 16000:
        I2C_WriteNAU88L25(0x0003,  0x801B); //MCLK = SYSCLK_SRC/12
        I2C_WriteNAU88L25(0x0004,  0x0001);
        I2C_WriteNAU88L25(0x0005,  0x3126); //MCLK = 4.096MHz
        I2C_WriteNAU88L25(0x0006,  0x0008);
        I2C_WriteNAU88L25(0x001D,  0x301A); //301A:Master, BCLK_DIV=MCLK/8=512K, LRC_DIV=512K/32=16K
        I2C_WriteNAU88L25(0x002B,  0x0002);
        I2C_WriteNAU88L25(0x002C,  0x0082);
        break;

    case 44100:
        I2C_WriteNAU88L25(0x001D,  0x301A); //301A:Master, BCLK_DIV=11.2896M/8=1.4112M, LRC_DIV=1.4112M/32=44.1K
        I2C_WriteNAU88L25(0x002B,  0x0012);
        I2C_WriteNAU88L25(0x002C,  0x0082);
        break;

    case 48000:
        I2C_WriteNAU88L25(0x001D,  0x301A); //301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K
        I2C_WriteNAU88L25(0x002B,  0x0012);
        I2C_WriteNAU88L25(0x002C,  0x0082);
        break;

    case 96000:
        I2C_WriteNAU88L25(0x0003,  0x80A2); //MCLK = SYSCLK_SRC/2
        I2C_WriteNAU88L25(0x0004,  0x1801);
        I2C_WriteNAU88L25(0x0005,  0x3126); //MCLK = 24.576MHz
        I2C_WriteNAU88L25(0x0006,  0xF008);
        I2C_WriteNAU88L25(0x001D,  0x301A); //3019:Master, BCLK_DIV=MCLK/8=3.072M, LRC_DIV=3.072M/32=96K
        I2C_WriteNAU88L25(0x002B,  0x0001);
        I2C_WriteNAU88L25(0x002C,  0x0080);
        break;
    }
}


#endif

void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C2));

}

void SD_Inits(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* select multi-function pin */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE6MFP_Msk|SYS_GPE_MFPL_PE3MFP_Msk|SYS_GPE_MFPL_PE2MFP_Msk|SYS_GPE_MFPL_PE7MFP_Msk);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk|SYS_GPB_MFPL_PB4MFP_Msk);

    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD13MFP_Msk;

    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SD0_DAT0|SYS_GPE_MFPL_PE3MFP_SD0_DAT1|
                      SYS_GPE_MFPL_PE6MFP_SD0_CLK|SYS_GPE_MFPL_PE7MFP_SD0_CMD);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_SD0_DAT2|SYS_GPB_MFPL_PB5MFP_SD0_DAT3);

    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD13MFP_SD0_nCD;
  
    /* Select IP clock source */
    //CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL, CLK_CLKDIV0_SDH0(10));
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_HXT, CLK_CLKDIV0_SDH0(1));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Lock protected registers */
    SYS_LockReg();
    
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}


extern uint32_t audioRxWriterPos;
extern uint32_t audioRxReaderPos;
extern uint32_t audioRxCnt;
extern uint32_t audioTxCnt;
extern uint32_t rxFIFOLevel;
FIL* gAudiohFile=NULL;
void Audio_TX();

//int32_t main (void)
void vAudioTask( void *pvParameters )
{
    uint32_t u32startFlag = 1;

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   I2S Driver Sample Code with WAU88L25                 |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with WAU88L25.\n");

    printf("  Please press keypad sw3 to start audio record ...\n");
    /* SW3 button */
    SYS->GPG_MFPL = (SYS->GPG_MFPL & ~(SYS_GPG_MFPL_PG5MFP_Msk));
    GPIO_SetMode(PG, BIT5, GPIO_MODE_INPUT);    
    /* SW2 button */
    SYS->GPG_MFPL = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF11MFP_Msk));
    GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT);
#if 0  
    while( 1 )
    {
      printf(" PF11=[%d]\n", PF11);
      vTaskDelay(500);
    }
#endif  
    while ( PG5 != 0)
    {
      vTaskDelay(1);
    }
    printf(" Now, start audio record ...[%d]\n", PG5);
    
    I2S0_ResetAudioBuffer();
  
    /* Init I2C2 to access Codec */
    I2C2_Init();

    /* Init SD */
    SD_Inits();

    FIL hFile;
    UINT retBytes;
    TCHAR       sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* set default path */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    gAudiohFile = NULL;
#if 0
//    printf(" File Ret= %d\n", f_open(&hFile, "0:\\test.mp3", FA_OPEN_EXISTING | FA_READ));
    printf(" File Ret= %d\n", f_open(&hFile, "0:\\test.txt", FA_OPEN_APPEND | FA_WRITE));
    f_write(&hFile, "12345678", 8, &retBytes);
    f_close(&hFile);
#endif        
    
    // Plug-In DET
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA4MFP_Msk));
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OUTPUT);
    PA4 = 1;

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

#if 0
    /* Open I2S0 interface and set to slave mode, 16K  sample rate & stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 16000, I2S_DATABIT_16, I2S_DISABLE_MONO, I2S_FORMAT_I2S);
#else
    /* Open I2S0 interface and set to slave mode, 16K sample rate & mono channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 48000, I2S_DATABIT_16, I2S_ENABLE_MONO, I2S_FORMAT_I2S);
#endif

    NVIC_EnableIRQ(I2S0_IRQn);

    /* Set PE.13 low to enable phone jack on NuMaker board. */
    SYS->GPE_MFPH &= ~(SYS_GPE_MFPH_PE13MFP_Msk);
    GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);
    PE13 = 0;

    // select source from HXT(12MHz)
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();
#else
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    vTaskDelay(20000/1000); 
    NAU88L25_Setup();
    NAU88L25_ConfigSampleRate(48000);    
#endif
    /* FIFO threshold could be 0~15 */
    I2S_SetFIFO(I2S0, 7, 7);

    /* Enable Rx threshold level interrupt */
    I2S_EnableInt(I2S0, I2S_IEN_RXTHIEN_Msk);

    f_open(&hFile, "0:\\test.pcm", FA_CREATE_ALWAYS | FA_WRITE);
    gAudiohFile = &hFile;
    printf("### In %s: gAudiohFile=0x%x\n", __FUNCTION__, gAudiohFile);

    /* Enable I2S Rx function to receive data */
    I2S_ENABLE_RX(I2S0);

    while(1)
    {
#if 0      
        if (u32startFlag)
        {
            /* Enable I2S Tx function to send data when data in the buffer is more than half of buffer size */
            if (u32AudioBuffPos >= BUFF_LEN/2)
            {
                I2S_EnableInt(I2S0, I2S_IEN_TXTHIEN_Msk);
                I2S_ENABLE_TX(I2S0);
                u32startFlag = 0;
            }
        } else {
            printf("Audio delay ...%d, rxFIFOLevel[%d]\n",xTaskGetTickCount(), rxFIFOLevel);
            vTaskDelay(1);
        }
#else
        if (u32startFlag)
        {
            /* Enable I2S Tx function to send data when data in the buffer is more than half of buffer size */
            if (audioRxCnt >= BUFF_LEN/2)
            {
//                I2S_EnableInt(I2S0, I2S_IEN_TXTHIEN_Msk);
                I2S_ENABLE_TX(I2S0);
                u32startFlag = 0;
                printf("Audio start TX ...\n");
            }
        } else {
            printf("Audio delay ...%d, [%d] [%d]\n",xTaskGetTickCount(),audioRxCnt, audioTxCnt);
            Audio_TX();
            //vTaskDelay(1);
            if ( PF11 == 0)
            {
              printf("Finish Audio record [%d] bytes\n",audioTxCnt);
              break;
            }
        }
#endif
    }
    gAudiohFile = NULL;
    f_close(&hFile);
}
#endif // end of NVT_AUDIO
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
