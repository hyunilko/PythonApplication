/*
 * spi_mw.c
 *
 * SPI Middleware for Multi Controller Mode
 * - Supports TCAN1576 (CAN Transceiver) on Channel 0
 * - Supports FT4222H (PC Tool Communication) on Channel 1
 * - Provides Mutex-based bus synchronization for safe multi-device access
 */

/* Standard Include Files */
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <drivers/mcspi.h>

#include <ti_drivers_config.h>
#include <ti_drivers_open_close.h>

#include <mw/datapath/dpc_mss.h>
#include <mw/mmw/mmwave_mss.h>
#include "mw/spi/spi_mw.h"
#include <conversion.h>
#include <hexdump.h>

/* ===================================================================================== */
/*                                  Private Definitions                                  */
/* ===================================================================================== */
#define MAXSPISIZEFTDI              (65536U)
#define SPI_BUS_LOCK_TIMEOUT_MS     (100U)

/* CHCONF Register bits (from cslr_app_spi.h) */
#define MCSPI_CHCONF_FORCE_MASK     (0x00100000U)  /* Bit 20: FORCE */

/* MODULCTRL Register (offset 0x128 from base) */
#define MCSPI_MODULCTRL_OFFSET      (0x00000128U)
#define MCSPI_MODULCTRL_SINGLE_MASK (0x00000001U)  /* Bit 0: SINGLE */

/* ===================================================================================== */
/*          CS Force Control (Multi Controller Mode Workaround)                          */
/* ===================================================================================== */
/*
 * TI MCSPI EPOL 비트 정의:
 *   EPOL = 0 → CS Active HIGH (CS HIGH = 선택 상태)
 *   EPOL = 1 → CS Active LOW  (CS LOW  = 선택 상태) ← TCAN1576 요구사항
 *
 * MCSPI_chConfig()가 csPolarity = MCSPI_CS_POL_LOW 설정 시 EPOL=1 로 올바르게 설정함.
 * ForceCSAssert/Deassert는 FORCE 비트만 조작하고, EPOL은 절대 건드리지 않는다.
 */
static inline void MW_SPI_ForceCSAssert(uint32_t baseAddr, uint32_t chNum)
{
    uint32_t regVal = MCSPI_readChConf(baseAddr, chNum);
    /* FORCE 비트만 Set */
    regVal |= MCSPI_CHCONF_FORCE_MASK;
    MCSPI_writeChConfReg(baseAddr, chNum, regVal);
}

static inline void MW_SPI_ForceCSDeassert(uint32_t baseAddr, uint32_t chNum)
{
    uint32_t regVal = MCSPI_readChConf(baseAddr, chNum);
    /* FORCE 비트만 Clear */
    regVal &= ~MCSPI_CHCONF_FORCE_MASK;
    MCSPI_writeChConfReg(baseAddr, chNum, regVal);
}

/* ===================================================================================== */
/*                                  Private Variables                                    */
/* ===================================================================================== */
/* SPI Host Interrupt configurations for FT4222H */
static uint32_t gSPIHostIntrBaseAddrLed;
static uint32_t gSPIHostIntrPinNumLed;

/* SPI Bus Mutex for Multi Controller synchronization */
static SemaphoreP_Object gSpiMutex;
static bool gSpiMutexInitialized = false;
static bool gSpiChannelsConfigured = false;

/* ===================================================================================== */
/*                              Mutex / Bus Control Implementation                       */
/* ===================================================================================== */

/* Configure MCSPI channels for Multi Controller mode */
static int32_t MW_SPI_ConfigChannels(void)
{
    int32_t status = SystemP_SUCCESS;

    if (gSpiChannelsConfigured)
    {
        return SystemP_SUCCESS;
    }

    if (gMcspiHandle[CONFIG_MCSPI0] == NULL)
    {
        return SystemP_FAILURE;
    }

    /* Configure Channel 0 (TCAN1576) */
    status = MCSPI_chConfig(gMcspiHandle[CONFIG_MCSPI0], &gConfigMcspi0ChCfg[SPI_CH_TCAN1576]);
    if (status != SystemP_SUCCESS)
    {
        printf("[SPI_MW] CH0 config failed: %d\r\n", status);
        return status;
    }

    /* Configure Channel 1 (FT4222H) */
    status = MCSPI_chConfig(gMcspiHandle[CONFIG_MCSPI0], &gConfigMcspi0ChCfg[SPI_CH_FT4222H]);
    if (status != SystemP_SUCCESS)
    {
        printf("[SPI_MW] CH1 config failed: %d\r\n", status);
        return status;
    }

    //printf("[SPI_MW] Multi Controller channels configured\r\n");
    gSpiChannelsConfigured = true;

    return SystemP_SUCCESS;
}

static inline void MW_SPI_SetSingleMode(uint32_t baseAddr, uint32_t chNum)
{
    uint32_t regVal;

    /* MODULCTRL.SINGLE = 1 (Single Channel mode) */
    regVal = CSL_REG32_RD(baseAddr + MCSPI_MODULCTRL_OFFSET);
    regVal |= MCSPI_MODULCTRL_SINGLE_MASK;
    CSL_REG32_WR(baseAddr + MCSPI_MODULCTRL_OFFSET, regVal);

    regVal = MCSPI_readChConf(baseAddr, chNum);
    regVal |= MCSPI_CHCONF_FORCE_MASK;   /* FORCE만 Set */
    MCSPI_writeChConfReg(baseAddr, chNum, regVal);
}

int32_t MW_SPI_MutexInit(void)
{
    int32_t status;

    if (gSpiMutexInitialized)
    {
        return SystemP_SUCCESS;
    }

    status = SemaphoreP_constructMutex(&gSpiMutex);
    if (status == SystemP_SUCCESS)
    {
        gSpiMutexInitialized = true;
    }

    /* Configure MCSPI channels for Multi Controller mode */
    status = MW_SPI_ConfigChannels();

    return status;
}

int32_t MW_SPI_BusLock(uint32_t timeoutMs)
{
    if (!gSpiMutexInitialized)
    {
        return SystemP_FAILURE;
    }

    return SemaphoreP_pend(&gSpiMutex, timeoutMs);
}

void MW_SPI_BusUnlock(void)
{
    if (gSpiMutexInitialized)
    {
        SemaphoreP_post(&gSpiMutex);
    }
}

/* ===================================================================================== */
/*                              Generic SPI Transfer (Multi Controller Safe)             */
/* ===================================================================================== */
spi_status_t MW_SPI_Transfer(uint32_t channel, uint8_t *txBuf, uint8_t *rxBuf, uint32_t countBytes)
{
    int32_t           transferOK;
    MCSPI_Transaction trans;
    uint32_t          chNum;
    uint32_t          baseAddr;
    static bool firstCall = true;

    if ((gMcspiHandle[CONFIG_MCSPI0] == NULL) || (countBytes == 0u))
    {
        return SPI_STATUS_PARAM;
    }

    /* Acquire SPI bus lock */
    if (MW_SPI_BusLock(SPI_BUS_LOCK_TIMEOUT_MS) != SystemP_SUCCESS)
    {
        return SPI_STATUS_BUSY;
    }

    chNum = gConfigMcspi0ChCfg[channel].chNum;
    baseAddr = MCSPI_getBaseAddr(gMcspiHandle[CONFIG_MCSPI0]);

    MCSPI_Transaction_init(&trans);

    trans.channel    = chNum;
    trans.dataSize   = 8U;
    trans.csDisable  = TRUE;
    trans.count      = countBytes;
    trans.txBuf      = (void *)txBuf;
    trans.rxBuf      = (void *)rxBuf;
    trans.args       = NULL;

    MW_SPI_SetSingleMode(baseAddr, chNum);

    if (firstCall)
    {
        // uint32_t chCtrl = MCSPI_readChCtrlReg(baseAddr, chNum);
        // uint32_t chConf = MCSPI_readChConf(baseAddr, chNum);
        // printf("[SPI_MW] channel idx=%u, chNum=%u, csDisable=%d\r\n", channel, trans.channel, trans.csDisable);
        // printf("[SPI_MW] Before: CHCTRL=0x%08X, CHCONF=0x%08X\r\n", chCtrl, chConf);

        /* Multi Controller Mode: Force CS assert before transfer */
        MW_SPI_ForceCSAssert(baseAddr, chNum);

        // chConf = MCSPI_readChConf(baseAddr, chNum);
        // printf("[SPI_MW] After FORCE: CHCONF=0x%08X\r\n", chConf);

        firstCall = false;
    }
    else
    {
        /* Multi Controller Mode: Force CS assert before transfer */
        MW_SPI_ForceCSAssert(baseAddr, chNum);
    }

    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &trans);

    /* Multi Controller Mode: Force CS deassert after transfer */
    MW_SPI_ForceCSDeassert(baseAddr, chNum);

    /* Release SPI bus lock */
    MW_SPI_BusUnlock();

    if ((SystemP_SUCCESS != transferOK) || (MCSPI_TRANSFER_COMPLETED != trans.status))
    {
        uint32_t chCtrl = MCSPI_readChCtrlReg(baseAddr, chNum);
        uint32_t chStat = MCSPI_readChStatusReg(baseAddr, chNum);
        printf("[SPI_MW] Transfer FAILED: ret=%d, status=%d\r\n", transferOK, trans.status);
        printf("[SPI_MW] After: CHCTRL=0x%08X, CHSTAT=0x%08X\r\n", chCtrl, chStat);
        return SPI_STATUS_FAIL;
    }

    return SPI_STATUS_OK;
}

/* ===================================================================================== */
/*                              TCAN1576 SPI Transfer Wrapper                            */
/* ===================================================================================== */
spi_status_t MW_SPI_TCAN_Transfer(uint8_t *txBuf, uint8_t *rxBuf, uint32_t countBytes)
{
    return MW_SPI_Transfer(SPI_CH_TCAN1576, txBuf, rxBuf, countBytes);
}

/* ===================================================================================== */
/*                              FT4222H (PC Tool) Implementation                         */
/* ===================================================================================== */
void MW_SPI_Init(void)
{
    /* Initialize SPI Mutex first */
    MW_SPI_MutexInit();

    /* Configure the FTDI HOST INTR PIN */
    gSPIHostIntrBaseAddrLed = (uint32_t) AddrTranslateP_getLocalAddr(SPI_HOST_INTR_BASE_ADDR);
    gSPIHostIntrPinNumLed = SPI_HOST_INTR_PIN;
    GPIO_setDirMode(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed, SPI_HOST_INTR_DIR);
    GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

    /* Initial sync pulse to ensure FT4222H slave is ready */
    GPIO_pinWriteLow(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);
    ClockP_usleep(5000); /* 5ms initial sync delay */
    GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);
    ClockP_usleep(5000); /* 5ms wait for slave init */
}

/* 65536 - 2 header bytes */
#define CHUNK_LENGTH_SPI          65534u

static int32_t send_data_via_spi(uint8_t* data, int32_t size)
{
    MCSPI_Transaction   spiTransaction;
    int32_t             transferOK;
    uint32_t totalSizeToTfr, tempSize;
    uint8_t chunkIdx;
    static uint32_t cnt = 0;
    uint32_t i;
    uint8_t* txBufPtr; //= (uint8_t*)mmwDemo_getMCB()->radarCube.data;

    printf("Frame size: %u bytes, Chunk size: %u bytes\n", size, MAXSPISIZEFTDI);

    totalSizeToTfr = size;
    tempSize = size;
    chunkIdx = 0;
    txBufPtr = (uint8_t*)data;

    /* Transfer data in chunks (max 64KB per chunk for FT4222H compatibility) */
    while(totalSizeToTfr > 0)
    {
        uint32_t chNum = gConfigMcspi0ChCfg[SPI_CH_FT4222H].chNum;
        uint32_t baseAddr = MCSPI_getBaseAddr(gMcspiHandle[CONFIG_MCSPI0]);

        if(totalSizeToTfr > MAXSPISIZEFTDI)
        {
            tempSize = MAXSPISIZEFTDI;
        }
        else
        {
            tempSize = totalSizeToTfr;
        }

        /* Acquire SPI bus lock */
        if (MW_SPI_BusLock(SPI_BUS_LOCK_TIMEOUT_MS) != SystemP_SUCCESS)
        {
            printf("SPI Bus Lock Failed!\n");
            return SystemP_FAILURE;
        }

        /* Configure SPI transaction - 8-bit mode for correct byte order */
        MCSPI_Transaction_init(&spiTransaction);
        spiTransaction.channel   = chNum;
        spiTransaction.dataSize  = 8;            /* 8-bit data width (no endian swap) */
        spiTransaction.csDisable = TRUE;         /* Multi Controller: explicit CS deassert */
        spiTransaction.count     = tempSize;     /* Number of bytes */
        spiTransaction.txBuf     = (void *)txBufPtr;
        spiTransaction.rxBuf     = NULL;         /* TX only, no RX */
        spiTransaction.args      = NULL;

        /* Signal HOST_INTR LOW first (notify slave) */
        GPIO_pinWriteLow(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

        /* Delay for slave to detect LOW via USB polling and prepare */
        ClockP_usleep(500);

        /* Multi Controller Mode: Force CS assert before transfer */
        MW_SPI_ForceCSAssert(baseAddr, chNum);

        transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);

        /* Multi Controller Mode: Force CS deassert after transfer */
        MW_SPI_ForceCSDeassert(baseAddr, chNum);

        GPIO_pinWriteHigh(gSPIHostIntrBaseAddrLed, gSPIHostIntrPinNumLed);

        /* Release SPI bus lock */
        MW_SPI_BusUnlock();

        if(transferOK != 0)
        {
            printf("SPI Raw Data Transfer Failed !!!\n");
            return SystemP_FAILURE;
        }

        ClockP_usleep(1000); /* 1ms delay for slave to process */
        // hexdump("SPI_SEND", txBufPtr, 64);
        totalSizeToTfr -= tempSize;
        txBufPtr += tempSize;
        chunkIdx++;
    }

    printf("Frame %u complete, %u chunks\n", cnt, chunkIdx);
    cnt++;

    return SystemP_SUCCESS;
}

/**
 * 🧿 SPI 통신 APP_PDU Data 구조 정의 (Header[16B] + Payload)
 * - Application Layer: Application PDU = frameID(4B, BE) + frameCnt(4B, BE) + chunkLen(2B, BE) + payloadCRC(2B, BE) payloadTotalLen(4B, BE) + Payload
 * - 00 ~ 03 byte     : frame indentification(4 bytes, big-endian, 12 34 56 78), 12 34 56 78은 0x12 0x34 0x56 0x78와 같이 Hex 값임
 * - 04 ~ 07 byte     : frame count(4 bytes, big-endian, XX XX XX XX)
 * - 08 ~ 09 byte     : max chunk length(2 bytes, big-endian, PP PP)
 * - 10 ~ 11 byte     : Payload CRC(2byte, big-endian, ZZ ZZ)
 * - 12 ~ 15 byte     : Payload total length(4 bytes, big-endian, YY YY YY YY)
 * - 16 ~             : Payload
 *
 * - Patten           : ``12 34 56 78 XX XX XX XX PP PP ZZ ZZ YY YY YY YY ...``
 */
int32_t MW_SPI_SendData(uint8_t* data, int32_t size)
{
    if (!data || size <= 0){
        printf("MW_SPI_SendData() size: %u/%u error!!!\n", mmwDemo_getMCB()->radarCube.dataSize, size);
        return -1;
    }
    if ((size + 1) > (int32_t)mmwDemo_getMCB()->radarCube.dataSize) {
        printf("MW_SPI_SendData() size: %u/%u error!!!\n", mmwDemo_getMCB()->radarCube.dataSize, size);
      return -1;
    }

    uint8_t* tx_buf = (uint8_t*)mmwDemo_getMCB()->radarCube.data;
    static uint32_t count = 0;

    printf("MW_SPI_SendData() Frame %u size: %u/%u\n", count, mmwDemo_getMCB()->radarCube.dataSize, size);

    /* 00 ~ 03 byte = frame indentification (Big Endian) */
    u32_to_be(0x12345678, &tx_buf[0]);
    /* 04 ~ 07 byte = frame count (Big Endian) */
    u32_to_be(count, &tx_buf[4]);
    /* 08 ~ 09 byte = max chunk length (Big Endian) */
    u16_to_be(CHUNK_LENGTH_SPI, &tx_buf[8]);
    /* 10 ~ 11 byte = payload CRC (Big Endian) */
    u16_to_be(0xabcd, &tx_buf[10]);
    /* 12 ~ 15 byte = payload total length (Big Endian) */
    u32_to_be(size, &tx_buf[12]);

    memcpy(&tx_buf[16], data, (size_t)size);

    int rc = send_data_via_spi(tx_buf, size + 16u);
    count++;
    return rc;
}
