#define TRUE  1
#define FALSE 0
#define bool  BYTE

#include "stm32g4xx_hal.h"
#include "diskio.h"
#include "fatfs_sd.h"

/* SPI chip-select pin */
#define SD_CS_GPIO_Port GPIOC
#define SD_CS_Pin       GPIO_PIN_4

extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t Timer1, Timer2;  /* 10 ms decremented by SDTimer_Handler */

/* Iteration guard for low-level SPI busy-waits: a marginal SD card must not
   be able to freeze TaskSDCard indefinitely. */
#define SPI_BUSY_RETRY 100000UL

static volatile DSTATUS Stat    = STA_NOINIT;
static uint8_t          CardType;   /* 0: MMC, 1: SDCv1, 2: SDCv1 block, 6: SDCv2 block */
static uint8_t          PowerFlag = 0;

/* ===========================================================================
 * LOW-LEVEL SPI HELPERS
 * =========================================================================*/

static void SELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void DESELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static void SPI_TxByte(BYTE data)
{
    /* Bounded: a stuck SPI peripheral must not hang TaskSDCard forever. */
    uint32_t guard = SPI_BUSY_RETRY;
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY && --guard);
    HAL_SPI_Transmit(&hspi1, &data, 1, SPI_TIMEOUT);
}

static uint8_t SPI_RxByte(void)
{
    uint8_t dummy = 0xFF;
    uint8_t data  = 0;

    /* Bounded: a stuck SPI peripheral must not hang TaskSDCard forever. */
    uint32_t guard = SPI_BUSY_RETRY;
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY && --guard);
    HAL_SPI_TransmitReceive(&hspi1, &dummy, &data, 1, SPI_TIMEOUT);

    return data;
}

static void SPI_RxBytePtr(uint8_t *buff)
{
    *buff = SPI_RxByte();
}

/* ===========================================================================
 * SD CARD PROTOCOL HELPERS
 * =========================================================================*/

/* Poll the bus until the card releases it (returns 0xFF) or the 500 ms
   Timer2 expires. */
static uint8_t SD_ReadyWait(void)
{
    uint8_t res;
    Timer2 = 50;  /* 50 × 10 ms = 500 ms */
    SPI_RxByte();
    do {
        res = SPI_RxByte();
    } while ((res != 0xFF) && Timer2);
    return res;
}

static void SD_PowerOn(void)
{
    uint8_t cmd_arg[6];
    uint32_t Count = 0x1FFF;

    DESELECT();

    /* Send ≥74 dummy clocks to initialize the card's internal state */
    for (int i = 0; i < 10; i++) {
        SPI_TxByte(0xFF);
    }

    SELECT();

    /* CMD0: GO_IDLE_STATE */
    cmd_arg[0] = CMD0 | 0x40;
    cmd_arg[1] = 0;
    cmd_arg[2] = 0;
    cmd_arg[3] = 0;
    cmd_arg[4] = 0;
    cmd_arg[5] = 0x95;  /* Fixed CRC for CMD0(0) */

    for (int i = 0; i < 6; i++) {
        SPI_TxByte(cmd_arg[i]);
    }

    while ((SPI_RxByte() != 0x01) && Count) {
        Count--;
    }

    DESELECT();
    SPI_TxByte(0xFF);

    PowerFlag = 1;
}

static void SD_PowerOff(void)
{
    PowerFlag = 0;
}

static uint8_t SD_CheckPower(void)
{
    return PowerFlag;  /* 0 = off, 1 = on */
}

/* Receive a 512-byte data block preceded by a start token (0xFE). */
static bool SD_RxDataBlock(BYTE *buff, UINT btr)
{
    uint8_t token;

    Timer1 = 10;  /* 10 × 10 ms = 100 ms timeout */
    do {
        token = SPI_RxByte();
    } while ((token == 0xFF) && Timer1);

    if (token != 0xFE)
        return FALSE;  /* Unexpected token */

    /* Read data two bytes at a time */
    do {
        SPI_RxBytePtr(buff++);
        SPI_RxBytePtr(buff++);
    } while (btr -= 2);

    SPI_RxByte();  /* Discard CRC (2 bytes) */
    SPI_RxByte();

    return TRUE;
}

#if _READONLY == 0
/* Transmit a data block (512 bytes) or a stop token (0xFD). */
static bool SD_TxDataBlock(const BYTE *buff, BYTE token)
{
    if (SD_ReadyWait() != 0xFF)
        return FALSE;

    SPI_TxByte(token);

    if (token != 0xFD) {
        /* Data token: send 512 bytes (wc wraps 0→255, loop runs 256×2 = 512 bytes) */
        uint8_t wc = 0;
        do {
            SPI_TxByte(*buff++);
            SPI_TxByte(*buff++);
        } while (--wc);

        SPI_RxByte();  /* Discard CRC (2 bytes) */
        SPI_RxByte();

        /* Poll for the data response token (bits[3:1] = 010 → accepted) */
        uint8_t resp;
        uint8_t i = 0;
        while (i <= 64) {
            resp = SPI_RxByte();
            if ((resp & 0x1F) == 0x05)
                break;
            i++;
        }

        /* Wait for the card to finish its internal write (bounded to ~1 s).
           An unbounded wait here was what froze LIDAR logging in flight. */
        Timer1 = 100;
        while ((SPI_RxByte() == 0) && Timer1);

        return ((resp & 0x1F) == 0x05) ? TRUE : FALSE;
    }

    /* Stop token (0xFD): no data block or response expected */
    return TRUE;
}
#endif /* _READONLY */

/* Send a command packet and return the R1 response byte. */
static BYTE SD_SendCmd(BYTE cmd, DWORD arg)
{
    uint8_t crc, res;

    if (SD_ReadyWait() != 0xFF)
        return 0xFF;

    /* Command packet */
    SPI_TxByte(cmd);
    SPI_TxByte((BYTE)(arg >> 24));
    SPI_TxByte((BYTE)(arg >> 16));
    SPI_TxByte((BYTE)(arg >> 8));
    SPI_TxByte((BYTE)arg);

    /* CRC is only checked for CMD0 and CMD8 in SPI mode */
    crc = 0;
    if (cmd == CMD0) crc = 0x95;
    if (cmd == CMD8) crc = 0x87;
    SPI_TxByte(crc);

    /* CMD12 (Stop Transmission): discard one padding byte before the response */
    if (cmd == CMD12)
        SPI_RxByte();

    /* Wait up to 10 attempts for a valid response (MSB clear = valid) */
    uint8_t n = 10;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

/* ===========================================================================
 * FATFS DISK I/O INTERFACE  (called by user_diskio.c)
 * =========================================================================*/

DSTATUS SD_disk_initialize(BYTE drv)
{
    uint8_t n, type, ocr[4];

    if (drv)
        return STA_NOINIT;  /* Only drive 0 supported */

    if (Stat & STA_NODISK)
        return Stat;

    SD_PowerOn();
    SELECT();
    type = 0;

    if (SD_SendCmd(CMD0, 0) == 1) {  /* Enter idle state */
        Timer1 = 100;                 /* 1 s initialization timeout */

        if (SD_SendCmd(CMD8, 0x1AA) == 1) {
            /* SDCv2: read OCR to verify voltage range */
            for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();

            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {  /* 2.7–3.6 V */
                do {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0)
                        break;  /* ACMD41 with HCS bit */
                } while (Timer1);

                if (Timer1 && SD_SendCmd(CMD58, 0) == 0) {  /* Read CCS bit */
                    for (n = 0; n < 4; n++) ocr[n] = SPI_RxByte();
                    type = (ocr[0] & 0x40) ? 6 : 2;
                }
            }
        } else {
            /* SDCv1 or MMC */
            type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? 2 : 1;
            do {
                if (type == 2) {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0)
                        break;  /* ACMD41 */
                } else {
                    if (SD_SendCmd(CMD1, 0) == 0)
                        break;  /* CMD1 */
                }
            } while (Timer1);

            if (!Timer1 || SD_SendCmd(CMD16, 512) != 0)
                type = 0;  /* Set block length failed */
        }
    }

    CardType = type;
    DESELECT();
    SPI_RxByte();  /* Release DO line */

    if (type) {
        Stat &= ~STA_NOINIT;
    } else {
        SD_PowerOff();
    }

    return Stat;
}

DSTATUS SD_disk_status(BYTE drv)
{
    if (drv)
        return STA_NOINIT;
    return Stat;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count)
        return RES_PARERR;
    if (Stat & STA_NOINIT)
        return RES_NOTRDY;

    if (!(CardType & 4))
        sector *= 512;  /* Convert to byte address for non-block-addressed cards */

    SELECT();

    if (count == 1) {
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512))
            count = 0;
    } else {
        if (SD_SendCmd(CMD18, sector) == 0) {
            do {
                if (!SD_RxDataBlock(buff, 512))
                    break;
                buff += 512;
            } while (--count);
            SD_SendCmd(CMD12, 0);  /* STOP_TRANSMISSION */
        }
    }

    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}

#if _READONLY == 0
DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv || !count)
        return RES_PARERR;
    if (Stat & STA_NOINIT)
        return RES_NOTRDY;
    if (Stat & STA_PROTECT)
        return RES_WRPRT;

    if (!(CardType & 4))
        sector *= 512;

    SELECT();

    if (count == 1) {
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
            count = 0;
    } else {
        if (CardType & 2) {
            SD_SendCmd(CMD55, 0);
            SD_SendCmd(CMD23, count);  /* ACMD23: pre-erase block count */
        }
        if (SD_SendCmd(CMD25, sector) == 0) {
            do {
                if (!SD_TxDataBlock(buff, 0xFC))
                    break;
                buff += 512;
            } while (--count);
            if (!SD_TxDataBlock(0, 0xFD))
                count = 1;
        }
    }

    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */

DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
    DRESULT res;
    BYTE n, csd[16], *ptr = buff;
    WORD csize;

    if (drv)
        return RES_PARERR;

    res = RES_ERROR;

    if (ctrl == CTRL_POWER) {
        switch (*ptr) {
        case 0:
            if (SD_CheckPower()) SD_PowerOff();
            res = RES_OK;
            break;
        case 1:
            SD_PowerOn();
            res = RES_OK;
            break;
        case 2:
            *(ptr + 1) = (BYTE)SD_CheckPower();
            res = RES_OK;
            break;
        default:
            res = RES_PARERR;
        }
    } else {
        if (Stat & STA_NOINIT)
            return RES_NOTRDY;

        SELECT();

        switch (ctrl) {
        case GET_SECTOR_COUNT:
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16)) {
                if ((csd[0] >> 6) == 1) {
                    /* SDCv2 */
                    csize = csd[9] + ((WORD)csd[8] << 8) + 1;
                    *(DWORD *)buff = (DWORD)csize << 10;
                } else {
                    /* MMC or SDCv1 */
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                    *(DWORD *)buff = (DWORD)csize << (n - 9);
                }
                res = RES_OK;
            }
            break;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            res = RES_OK;
            break;

        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF)
                res = RES_OK;
            break;

        case MMC_GET_CSD:
            if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16))
                res = RES_OK;
            break;

        case MMC_GET_CID:
            if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16))
                res = RES_OK;
            break;

        case MMC_GET_OCR:
            if (SD_SendCmd(CMD58, 0) == 0) {
                for (n = 0; n < 4; n++) *ptr++ = SPI_RxByte();
                res = RES_OK;
            }
            break;

        default:
            res = RES_PARERR;
        }

        DESELECT();
        SPI_RxByte();
    }

    return res;
}
