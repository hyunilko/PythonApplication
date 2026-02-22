/*
 *   @file  main_full_mss.c
 *
 *   @brief
 *      Unit Test code for the mmWave 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020-2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#define DebugP_LOG_ENABLED 1

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* mmWave PDK Include Files: */
#include <kernel/dpl/ClockP.h>
#include <ti_drivers_config.h>
#include <ti_board_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <FreeRTOS.h>
#include <task.h>

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <utils/testlogger/logger.h>

#include "ti_control_example_config.h"

/* FreeRTOS Task declarations. */
#define APP_TASK_PRI                        (5U)
#define APP_CTRL_TASK_PRI                   (7U)
#define APP_TASK_STACK_SIZE                 (8*1024U)
#define APP_CTRL_TASK_STACK_SIZE            (6*1024U)

#if (SPI_ENABLE == 1U)
#define ADC_TASK_PRI                        (3U)
#define ADC_TASK_SIZE                       (32*1024U)
StaticTask_t gAdcTaskObj;
TaskHandle_t gAdcTask;
StackType_t gAdcTaskStack[ADC_TASK_SIZE] __attribute__((aligned(32)));
#endif

#define MMWAVE_TASK_PRI                        (2U)
#define MMWAVE_TASK_SIZE                       (32*1024U)

StaticTask_t gMmWaveTaskObj;
TaskHandle_t gMmWaveTask;
StackType_t gMmWaveTaskStack[MMWAVE_TASK_SIZE] __attribute__((aligned(32)));

/* UART Buffers*/
#define APP_UART_BUFSIZE                    (200U)
#define APP_UART_RECEIVE_BUFSIZE            (8U)

#define INFINITE_FRAMES                     (0)

#define EDMA_TEST_EVT_QUEUE_NO             (0U)
#define EDMA_CHIRP_AVAIL_EVENT             (37)

 /* Buffer to store Raw ADC Data per frame 
 * Important Note: Size of this buffer has to be updated based on chirp configuration
 * This should be minimum of (Num of Chirps in a frame) * (Number of ADC Samples per chirp) * (2 Bytes) * (Num of Rx Antennas)
 */
#define MAX_ADC_BUF_SIZE                   (300*1024U)


/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/* This is the handle to the mmWave module */
MMWave_Handle   gMMWaveHandle;
/* This is the structure which holds all mmwave configurations */
MMWave_Cfg      mmwcfg;
#if (ENABLE_DIGLB == 1U)
/* This is the structure which holds the Digital LoopBack configuration*/
MMWave_DigLbCfg lbcfg;
#endif
/* Counters for Frames */
int frmEndCntr = 0;
int frmcnt = 0;

/* Hwi Objects */
HwiP_Object gHwiFrameStartHwiObject;
HwiP_Object gHwiFrameEndHwiObject;

/* Semaphore object for synchronizing frames*/
SemaphoreP_Object mmwaveFrameSemHandle;
#if (SPI_ENABLE == 1U)
/* Semaphore object for ADC Data Logging*/
SemaphoreP_Object mmwaveADCDataLoggingSemHandle;
#endif

/* Semaphore object for Main task*/
SemaphoreP_Object mmwaveControlHandle;

/* UART Buffers */
uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];

uint32_t adcDataPerFrame;
#if (SPI_ENABLE == 1U)
/*! SPI Host Intr configurations */
uint32_t gSPIHostIntrBaseAddrLed,gSPIHostIntrPinNumLed;

/* Max Frame Size for FTDI chip is 64KB */
#define MAXSPISIZEFTDI               (65536U)
__attribute__((section(".bss.l3"))) uint8_t adcbuffer[MAX_ADC_BUF_SIZE];
__attribute__((section(".bss.l3"))) uint8_t adcbuffer2[MAX_ADC_BUF_SIZE];
#endif
/*! LED configurations */
uint32_t gFrameBaseAddrLed,gFramePinNumLed;



/**************************************************************************
 *********************** mmWave Unit Test Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to write to the serial interface.
 *
 *  @param[in]  format
 *      Character pointer for taking string input
 *
 *  @retval
 *      Not applicable
 */
void MmWave_serialWrite(const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;
    UART_Transaction trans;
    UART_Handle uartWriteHandle;

    uartWriteHandle = gUartHandle[CONFIG_UART_CONSOLE];

    UART_Transaction_init(&trans);

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* If CLI_write is called before CLI init has happened, return */
    if (uartWriteHandle == NULL)
    {
        return;
    }

    trans.buf   = &logMessage[0U];
    trans.count = sizeMessage;

    /* Log the message on the UART CLI console: */
    /* Blocking Mode: */
    UART_write (uartWriteHandle, &trans);
}

static inline void u32_to_be(uint32_t value, uint8_t *buffer) {
    buffer[0] = (uint8_t)((value >> 24) & 0xFFu);
    buffer[1] = (uint8_t)((value >> 16) & 0xFFu);
    buffer[2] = (uint8_t)((value >> 8)  & 0xFFu);
    buffer[3] = (uint8_t)(value & 0xFFu);
}

#if (SPI_ENABLE == 1U)
/**
 *  @b Description
 *  @n
 *      The function is used stream the adc data to host via SPI interface 
 *
 *  @param[in]  format
 *      Character pointer for taking string input
 *
 *  @retval
 *      Not applicable
 */
void adcDataLoggingViaSPI()
{
    /* Variables require for ADC Data Logging via SPI */
    MCSPI_Transaction   spiTransaction;
    int32_t             transferOK;
    uint32_t totalSizeToTfr, tempSize;
    uint8_t count;
    uint32_t* adc_data = (uint32_t*)adcbuffer2;
    uint32_t cnt = 0;
    uint32_t i;
    uint8_t* byte_ptr;

    /***************************ADC Streaming Via SPI***********************************************************/
    /* This code Section allows the streaming of raw ADC data over SPI interface every frame during the frame idle time. */

    while(true)
    {
        SemaphoreP_pend(&mmwaveADCDataLoggingSemHandle, SystemP_WAIT_FOREVER);

        byte_ptr = (uint8_t*)adcbuffer2;
        /* 0 ~ 3 byte = frame number  */
        u32_to_be(cnt, byte_ptr);

        /* Fill rest with incrementing sequence */
        for(i = 4; i < adcDataPerFrame; i++)
        {
            byte_ptr[i] = (uint8_t)((i - 4) & 0xFF);
        }

        /* Print first 16 bytes for comparison */
        // MmWave_serialWrite("Frame %u, Size %u bytes\r\n", cnt, adcDataPerFrame);
        // MmWave_serialWrite("Frame %u : %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", cnt,
        //     byte_ptr[0], byte_ptr[1], byte_ptr[2], byte_ptr[3],
        //     byte_ptr[4], byte_ptr[5], byte_ptr[6], byte_ptr[7],
        //     byte_ptr[8], byte_ptr[9], byte_ptr[10], byte_ptr[11],
        //     byte_ptr[12], byte_ptr[13], byte_ptr[14], byte_ptr[15]);

        totalSizeToTfr = adcDataPerFrame;
        tempSize = adcDataPerFrame;
        count = 0;

        while(totalSizeToTfr > 0)
        {
            if(totalSizeToTfr > MAXSPISIZEFTDI)
            {
                tempSize = MAXSPISIZEFTDI;
            }
            else
            {
                tempSize = totalSizeToTfr;
            }

            MCSPI_Transaction_init(&spiTransaction);
            spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
            spiTransaction.dataSize = 32;
            spiTransaction.csDisable = TRUE;
            spiTransaction.count    = tempSize / 4;
            spiTransaction.txBuf    = (void *)(&adc_data[(MAXSPISIZEFTDI/4) * count]);
            spiTransaction.rxBuf    = NULL;
            spiTransaction.args     = NULL;

            GPIO_pinWriteLow(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);
            transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
            if(transferOK != 0)
            {
                MmWave_serialWrite("SPI Transfer Failed at chunk %u\r\n", count);
            }
            GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

            totalSizeToTfr = totalSizeToTfr - tempSize;
            count++;
        }

        MmWave_serialWrite("Frame %u complete, %u chunks\r\n", cnt, count);
        cnt++;
    }
}
#endif


/**
 *  @b Description
 *  @n
 *      The function is used to populate the default configurations.
 *
 *  @param[out]  ptrMmwaveCfg
 *      Pointer to the Mmwave configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultCfg ()
{

    /* Populate Calib Cfg*/
    mmwcfg.calibCfg.rxGain = 44;
    mmwcfg.calibCfg.txBackoffSel = 1;
    mmwcfg.calibCfg.ptrFactoryCalibData = NULL;

    /* Populate Open Configuration*/
    mmwcfg.openCfg.runTxCLPCCalib = false;

    /* Populate APLL frequency */
    mmwcfg.apllFreqMHz = APLL_FREQ_400MHZ;

    /* Important Note: Size of Software buffer used to hold ADC Data
    * has to be updated based on chirp configuration (MAX_ADC_BUF_SIZE)
    */

    /* Populate the profile configuration: */
    mmwcfg.profileComCfg.chirpRxHpfSel = M_RL_SENS_RX_HPF_SEL_1400KHZ; 
    mmwcfg.profileComCfg.dfeFirSel = M_RL_SENS_DFE_FIR_LONG_FILT;
    mmwcfg.profileComCfg.digOutputBitsSel = M_RL_SENS_DIG_OUT_12BITS_4LSB_ROUND;
    mmwcfg.profileComCfg.digOutputSampRate = M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_25M;
    mmwcfg.profileComCfg.numOfAdcSamples = 256U; 
    mmwcfg.profileComCfg.chirpTxMimoPatSel = 0U;
    mmwcfg.profileComCfg.chirpRampEndTimeus = 24.3; /* 24.3us low res */

    mmwcfg.profileTimeCfg.chirpIdleTimeus = 28; /* 28us low res */
    mmwcfg.profileTimeCfg.chirpAdcStartTime = 63;
    mmwcfg.profileTimeCfg.chirpTxStartTimeus = 0;
    mmwcfg.profileTimeCfg.chirpSlope = 160; /* Although we give non-zero value for chirp slope, in CW mode it is forced to 0 by firmware. */
    mmwcfg.profileTimeCfg.startFreqGHz = 59; 

    #if (ENABLE_DIGLB == 1U)
    /* Populate the Digital LoopBack configuration */
    lbcfg.lbFreqSel = 10U;
    lbcfg.ifaLbGainIndex = 153U;
    lbcfg.loPaLbCmnGainIndex = 48U;
    lbcfg.loLbGainIndex = 96U;
    lbcfg.paLbGainIndex = 48U;
    lbcfg.extLbTxBpmEnSel = 0U;
    lbcfg.lbEnable = 0x1U;
    #endif

    /* Populating Channel Cfg */
    mmwcfg.txEnbl = (TX0A_EN_SEL|TX0B_EN_SEL|TX1A_EN_SEL|TX1B_EN_SEL| \
                     TX2A_EN_SEL|TX2B_EN_SEL|TX3A_EN_SEL|TX3B_EN_SEL); /* Enable all 4 Tx */
    mmwcfg.rxEnbl = (RX0A_EN_SEL | RX1A_EN_SEL | RX2A_EN_SEL | RX3A_EN_SEL); /* Enable all 4 Rx */

    /* Populate the frame configuration: */
    mmwcfg.frameCfg.numOfChirpsInBurst = 64;
    mmwcfg.frameCfg.numOfChirpsAccum = 0;
    mmwcfg.frameCfg.burstPeriodus = 4000;
    mmwcfg.frameCfg.numOfBurstsInFrame = 1;
    mmwcfg.frameCfg.framePeriodicityus = (64 * 1000);
    mmwcfg.frameCfg.numOfFrames = 0;

    /*Populate the Start configuration: */
    #if (CW_MODE_ENABLE == 1U)
    mmwcfg.strtCfg.frameTrigMode = M_RL_SENS_FRAME_CW_CZ_TRIG;
    #else
    mmwcfg.strtCfg.frameTrigMode = M_RL_SENS_FRAME_SW_TRIG; // SW trigger
    #endif
    mmwcfg.strtCfg.chirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS;
    mmwcfg.strtCfg.frameLivMonEn = 0; //Disable all Live Monitors
    mmwcfg.strtCfg.frameTrigTimerVal = 0;
    
}

/**
 *  @b Description
 *  @n
 *      ISR Handler for Frame Start Interrupt
 *
 *  @retval
 *      Not Applicable.
 */
void mmwDemoFrameStartISR(void *arg)
{
    frmcnt++;
    HwiP_clearInt(CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);
}

/**
 *  @b Description
 *  @n
 *      ISR Handler for Frame End Interrupt
 *
 *  @retval
 *      Not Applicable.
 */
void mmwDemoFrameEndISR(void *arg)
{
    frmEndCntr++;
    HwiP_clearInt(CSL_APPSS_INTR_FECSS_CHIRPTIMER_FRAME_END);
    #if (SPI_ENABLE == 1U)
    SemaphoreP_post(&mmwaveADCDataLoggingSemHandle);
    #endif
    if((mmwcfg.frameCfg.numOfFrames != INFINITE_FRAMES) && (frmEndCntr == mmwcfg.frameCfg.numOfFrames))
    {
        SemaphoreP_post(&mmwaveFrameSemHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      Registers the Frame Start Interrupt
 *
 *  @retval
 *      Success -   Interrupt is registered
 *  @retval
 *      Error   -   NULL
 */
int32_t mmwDemo_registerFrameStartInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START;
    hwiPrms.callback    = mmwDemoFrameStartISR;
    /* Use this to change the priority */
    // hwiPrms.priority    = 0;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiFrameStartHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Registers the Frame End Interrupt
 *
 *  @retval
 *      Success -   Interrupt is registered
 *  @retval
 *      Error   -   NULL
 */
int32_t mmwDemo_registerFrameEndInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = CSL_APPSS_INTR_FECSS_CHIRPTIMER_FRAME_END;
    hwiPrms.callback    = mmwDemoFrameEndISR;
    /* Use this to change the priority */
    hwiPrms.priority    = 2;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwiFrameEndHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}


#if (SPI_ENABLE == 1U)
/**
 *  @b Description
 *  @n
 *      Will configure EDMA to transfer ADC Data every chirp for a frame
 *
 */
void Enable_Streaming ()
{
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    uint8_t             *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry    edmaParam;
    uint32_t            dmaCh, tcc, param, parambackup;

    /* Allocate memory for buffer */
    adcDataPerFrame = mmwcfg.frameCfg.numOfChirpsInBurst *  mmwcfg.frameCfg.numOfBurstsInFrame * mmwcfg.profileComCfg.numOfAdcSamples * 4 * 2;
    /* Configure EDMA channel for reading the Raw ADC data */
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[CONFIG_EDMA1]);
    regionId = EDMA_getRegionId(gEdmaHandle[CONFIG_EDMA1]);
    dmaCh = EDMA_CHIRP_AVAIL_EVENT;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[CONFIG_EDMA1], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    tcc = EDMA_CHIRP_AVAIL_EVENT;
    testStatus = EDMA_allocTcc(gEdmaHandle[CONFIG_EDMA1], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    param = EDMA_CHIRP_AVAIL_EVENT;
    testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    parambackup = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[CONFIG_EDMA1], &parambackup);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    srcBuffPtr = (uint8_t *) CSL_DSS_ADCBUF_READ_U_BASE;
    dstBuffPtr = (uint8_t *) (adcbuffer);
    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
    dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);
    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) mmwcfg.profileComCfg.numOfAdcSamples * 4 * 2;
    edmaParam.bCnt          = (uint16_t) mmwcfg.frameCfg.numOfChirpsInBurst * mmwcfg.frameCfg.numOfBurstsInFrame;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) mmwcfg.frameCfg.numOfChirpsInBurst * mmwcfg.frameCfg.numOfBurstsInFrame;
    edmaParam.srcBIdx       = (int16_t) 0;
    edmaParam.destBIdx      = (int16_t) (mmwcfg.profileComCfg.numOfAdcSamples * 4 * 2);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
    edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
    edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
    EDMASetPaRAM(baseAddr, param, &edmaParam);
    EDMAEnableTransferRegion(baseAddr, regionId, dmaCh,
        EDMA_TRIG_MODE_EVENT);

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) mmwcfg.profileComCfg.numOfAdcSamples * 4 * 2;
    edmaParam.bCnt          = (uint16_t) mmwcfg.frameCfg.numOfChirpsInBurst * mmwcfg.frameCfg.numOfBurstsInFrame;
    edmaParam.cCnt          = (uint16_t) 1;
    edmaParam.bCntReload    = (uint16_t) mmwcfg.frameCfg.numOfChirpsInBurst * mmwcfg.frameCfg.numOfBurstsInFrame;
    edmaParam.srcBIdx       = (int16_t) 0;
    edmaParam.destBIdx      = (int16_t) (mmwcfg.profileComCfg.numOfAdcSamples * 4 * 2);
    edmaParam.srcCIdx       = (int16_t) 0;
    edmaParam.destCIdx      = (int16_t) (-1 * adcDataPerFrame);
    edmaParam.linkAddr      = (0x4000 + (32 * parambackup));
    edmaParam.opt          |= ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK);
    EDMASetPaRAM(baseAddr, parambackup, &edmaParam);
}
#endif

/**
 *  @b Description
 *  @n
 *      Test implementation
 *
 *  @retval
 *      Not Applicable.
 */
void Mmwave_Test ()
{
    int32_t                 errCode;
    int32_t                 retVal;
    MMWave_ErrorLevel       errorLevel;
    int16_t                 mmWaveErrorCode;
    int16_t                 subsysErrorCode;
    ADCBuf_Params           params;
    ADCBuf_Handle           handle;
    ADCBuf_RxChanConf       rxChanConf;

    /************************************************************************
     * Intializing the mmWave:
     ************************************************************************/
    /* Initialize the configuration: */
    memset ((void *)&mmwcfg, 0, sizeof(MMWave_Cfg));
    memset((void *)&rxChanConf, 0, sizeof(rxChanConf));

    #if (CW_MODE_ENABLE == 0U)
    mmwDemo_registerFrameStartInterrupt();
    mmwDemo_registerFrameEndInterrupt();
    #endif

    /* Open the first ADCBUF Instance */
    ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
    
    /* Initialize and setup the mmWave Control module */
    gMMWaveHandle = MMWave_init (&mmwcfg.initCfg, &errCode);
    if (gMMWaveHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        MmWave_serialWrite ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        MmWave_serialWrite("MMWave MSS Initialization Failed\r\n");
        return;
    }
    
    MmWave_serialWrite("MMWave MSS Initialization Done\r\n");

    /************************************************************************
     * Populating the Cfg
     ************************************************************************/
    Mmwave_populateDefaultCfg ();
    #if (SPI_ENABLE == 1U)
    Enable_Streaming();
    #endif

    /************************************************************************
     * Enable ADCBUF Channel
     ************************************************************************/
    rxChanConf.channel = 0;
    rxChanConf.offset = 0;
    ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
    rxChanConf.channel = 1;
    rxChanConf.offset = mmwcfg.profileComCfg.numOfAdcSamples*2;
    ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
    rxChanConf.channel = 2;
    rxChanConf.offset =mmwcfg.profileComCfg.numOfAdcSamples*2*2;
    ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);
    rxChanConf.channel = 3;
    rxChanConf.offset = mmwcfg.profileComCfg.numOfAdcSamples*3*2;
    ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);

    /************************************************************************
     * Turn on FECSS/APLL Clock
     ************************************************************************/
    /* FECSS/APLL Clock Turn OFF */
    retVal = MMWave_FecssDevClockCtrl(&mmwcfg.initCfg, MMWAVE_APLL_CLOCK_DISABLE, &errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        MmWave_serialWrite ("Error: FECSS/APLL Clock Turn OFF failed\r\n");
        return;
    }

    /* Configure APLL clock registers for 400MHz APLL frequency */
    retVal = MMWave_ConfigApllReg(APLL_FREQ_400MHZ);
    if(retVal != SystemP_SUCCESS)
    {
        MmWave_serialWrite ("Error: APLL clock register configuration failed\r\n");
        return;
    }

    retVal = MMWave_FecssDevClockCtrl(&mmwcfg.initCfg, MMWAVE_APLL_CLOCK_ENABLE, &errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        MmWave_serialWrite("MMWave Clock POWER ON Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave Clock POWER ON Done\r\n");

    /************************************************************************
     * RF Power On/Off
     ************************************************************************/
    retVal = MMWave_FecssRfPwrOnOff(mmwcfg.txEnbl,mmwcfg.rxEnbl,&errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        MmWave_serialWrite("MMWave RF POWER ON/OFF Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave RF POWER ON/OFF Done\r\n");

    /************************************************************************
     * Run Factory Calibrations.
     ************************************************************************/
    retVal = MMWave_factoryCalib(gMMWaveHandle, &mmwcfg, &errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        MmWave_serialWrite("MMWave Factory Calibration Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave Factory Calibration Done\r\n");

    /************************************************************************
     * Open the mmWave:
     ************************************************************************/
    if (MMWave_open (gMMWaveHandle, &mmwcfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        MmWave_serialWrite ("Error: mmWave open failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave MSS Open Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave MSS Open done.\r\n");

    /************************************************************************
     * Configure the mmWave:
     ************************************************************************/
    if (MMWave_config (gMMWaveHandle, &mmwcfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        MmWave_serialWrite ("Error: mmWave configuration failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave MSS Configuration Failed\r\n");
        return;
    }
    MmWave_serialWrite ("MMWave MSS Configuration done\r\n");

#if (ENABLE_DIGLB == 1U)
    /************************************************************************
     * Enable Digital Loopback:
     ************************************************************************/
    if (MMWave_enblDigLb (&lbcfg,&errCode) < 0)
    {
        /* Error: Unable to enable Digital Loopback */
        MmWave_serialWrite ("Error: mmWave digital loopback failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave Digital Loopback Failed\r\n");
        return;
    }
    MmWave_serialWrite ("MMWave Digital Loopback done\r\n");
#endif

    /************************************************************************
     * Start the mmWave:
     ************************************************************************/
    if (MMWave_start (gMMWaveHandle, &mmwcfg.strtCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        MmWave_serialWrite ("Error: mmWave start failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave MSS Start Failed\r\n");
        return;
    }
    GPIO_pinWriteHigh(gFrameBaseAddrLed, gFramePinNumLed);
    #if (CW_MODE_ENABLE == 0U)
    MmWave_serialWrite("MMWave MSS Start done\r\n");
    SemaphoreP_pend(&mmwaveFrameSemHandle, SystemP_WAIT_FOREVER);
    #else
    MmWave_serialWrite("MMWave CW Mode Start done\r\n");
    ClockP_usleep(CW_MODE_TIME_US);
    #endif
    GPIO_pinWriteLow(gFrameBaseAddrLed, gFramePinNumLed);

    /************************************************************************
     * Stop the mmWave:
     ************************************************************************/
    retVal = MMWave_stop (gMMWaveHandle, &mmwcfg.strtCfg, &errCode);
    if (retVal < 0)
    {
        /* Error: Stopping the sensor failed. Decode the error code. */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        MmWave_serialWrite ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Did we fail because of an error? */
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error Level: The test has failed. */
            MmWave_serialWrite("MMWave MSS Stop Failed\r\n");
            return;
        }
        else
        {
            /* Informational Level: The test has passed. Fall through...*/
        }
    }
    #if (CW_MODE_ENABLE == 0U)
    MmWave_serialWrite ("MMWave MSS Stop done.\r\n");
    #else
    MmWave_serialWrite ("MMWave CW Mode Stop done.\r\n");
    #endif
    
    /************************************************************************
     * Close the mmWave:
     ************************************************************************/
    if (MMWave_close (gMMWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        MmWave_serialWrite ("Error: mmWave close failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave MSS Close Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave MSS close done.\r\n");
    
    /************************************************************************
     * Deinitialize the mmWave module:
     ************************************************************************/
    if (MMWave_deinit(gMMWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to deinitialize the mmWave control module */
        MmWave_serialWrite ("Error: mmWave Deinitialization failed [Error code %d]\n", errCode);
        MmWave_serialWrite("MMWave MSS Deinitialization Failed\r\n");
        return;
    }
    MmWave_serialWrite("MMWave MSS Deinitializion done\r\n");
    MmWave_serialWrite ("--- Test Completed ---\r\n");
    return;
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void mmwave_example_main(void* args)
{
    int32_t errorCode = SystemP_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    /* Configure the LED */
    gFrameBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_FRAME_LED_BASE_ADDR);
    gFramePinNumLed = CONFIG_FRAME_LED_PIN;
    GPIO_setDirMode(gFrameBaseAddrLed, gFramePinNumLed, CONFIG_FRAME_LED_DIR);
    GPIO_pinWriteLow(gFrameBaseAddrLed, gFramePinNumLed);

    #if (SPI_ENABLE == 1U)
    /* Configure the FTDI HOST INTR PIN */
    gSPIHostIntrBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(SPI_HOST_INTR_BASE_ADDR);
    gSPIHostIntrPinNumLed = SPI_HOST_INTR_PIN;
    GPIO_setDirMode(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed, SPI_HOST_INTR_DIR);
    GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

    /* then create the tasks, order of task creation does not matter for this example */
    gAdcTask = xTaskCreateStatic(adcDataLoggingViaSPI,      /* Pointer to the function that implements the task. */
                                   "adcLogging",          /* Text name for the task.  This is to facilitate debugging only. */
                                   ADC_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                   NULL,            /* We are not using the task parameter. */
                                   ADC_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                   gAdcTaskStack,  /* pointer to stack base */
                                   &gAdcTaskObj ); /* pointer to statically allocated task object memory */
    #endif

    /* then create the tasks, order of task creation does not matter for this example */
    gMmWaveTask = xTaskCreateStatic(Mmwave_Test,      /* Pointer to the function that implements the task. */
                                  "mmwaveTask",          /* Text name for the task.  This is to facilitate debugging only. */
                                  MMWAVE_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  MMWAVE_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gMmWaveTaskStack,  /* pointer to stack base */
                                  &gMmWaveTaskObj ); /* pointer to statically allocated task object memory */


    /* Debug Message: */
    MmWave_serialWrite ("***********************\r\n");
    MmWave_serialWrite ("Debug: Launching mmwave\r\n");
    MmWave_serialWrite ("***********************\r\n");

    errorCode = SemaphoreP_constructBinary(&mmwaveFrameSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);
    #if (SPI_ENABLE == 1U)
    errorCode = SemaphoreP_constructBinary(&mmwaveADCDataLoggingSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);
    #endif
    errorCode = SemaphoreP_constructBinary(&mmwaveControlHandle, 0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    SemaphoreP_pend(&mmwaveControlHandle, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();

}
