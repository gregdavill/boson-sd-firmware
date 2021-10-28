#include <liblitesdcard/sdcard.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <system.h>

#include "fatfs/source/ff.h"
#include "fatfs/source/diskio.h" /* Common include file for FatFs and disk I/O layer */
#include <stdint.h>


//#define SDCARD_CMD23_SUPPORT /* SET_BLOCK_COUNT */
#define SDCARD_CMD18_SUPPORT /* READ_MULTIPLE_BLOCK */
#define SDCARD_CMD25_SUPPORT /* WRITE_MULTIPLE_BLOCK */

#ifndef SDCARD_CLK_FREQ_INIT
#define SDCARD_CLK_FREQ_INIT 400000
#endif

#ifndef SDCARD_CLK_FREQ
#define SDCARD_CLK_FREQ 25000000
#endif

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count){
    sdcard_write(sector, count, buff);
    return RES_OK;
}
 

 static inline int sdcard_send_command(uint32_t arg, uint8_t cmd, uint8_t rsp) {
	sdcore_cmd_argument_write(arg);
	sdcore_cmd_command_write((cmd << 8) | rsp);
	sdcore_cmd_send_write(1);
	return sdcard_wait_cmd_done();
}


#define NULL (0)

extern void print(const char *);
extern void dump(const BYTE *buff, WORD cnt);

static void memcpy(uint8_t *dst, uint8_t *src, uint32_t count)
{
	if (count == 0)
		return;

	while (count--)
		*dst++ = *src++;
}

/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/

#define dly_us(n) busy_wait_us(n)

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* ----- MMC/SDC command ----- */
#define CMD0 (0)		   /* GO_IDLE_STATE */
#define CMD1 (1)		   /* SEND_OP_COND (MMC) */
#define CMD2 (2)		   /* ALL_SEND_CID */
#define CMD3 (3)		   /* SEND_RELATIVE_ADDR */
#define ACMD6 (6 | 0x80)   /* SET_BUS_WIDTH (SDC) */
#define CMD7 (7)		   /* SELECT_CARD */
#define CMD8 (8)		   /* SEND_IF_COND */
#define CMD9 (9)		   /* SEND_CSD */
#define CMD10 (10)		   /* SEND_CID */
#define CMD12 (12)		   /* STOP_TRANSMISSION */
#define CMD13 (13)		   /* SEND_STATUS */
#define ACMD13 (13 | 0x80) /* SD_STATUS (SDC) */
#define CMD16 (16)		   /* SET_BLOCKLEN */
#define CMD17 (17)		   /* READ_SINGLE_BLOCK */
#define CMD18 (18)		   /* READ_MULTIPLE_BLOCK */
#define CMD23 (23)		   /* SET_BLK_COUNT (MMC) */
#define ACMD23 (23 | 0x80) /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24 (24)		   /* WRITE_BLOCK */
#define CMD25 (25)		   /* WRITE_MULTIPLE_BLOCK */
#define CMD32 (32)		   /* ERASE_ER_BLK_START */
#define CMD33 (33)		   /* ERASE_ER_BLK_END */
#define CMD38 (38)		   /* ERASE */
#define ACMD41 (41 | 0x80) /* SEND_OP_COND (SDC) */
#define CMD55 (55)		   /* APP_CMD */

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */

static volatile WORD Timer[2]; /* 1000Hz decrement timer for Transaction and Command */

static WORD CardRCA;	   /* Assigned RCA */
static BYTE CardType,	  /* Card type flag */
	CardInfo[16 + 16 + 4]; /* CSD(16), CID(16), OCR(4) */

/* Block transfer buffer (located in USB RAM) */
//static DWORD blockBuff[128] __attribute__((section(".scratchpadRam0")));

/*-----------------------------------------------------------------------*/
/* Send a command token to the card and receive a response               */
/*-----------------------------------------------------------------------*/

static int send_cmd(			/* Returns 1 when function succeeded otherwise returns 0 */
					UINT idx,   /* Command index (bit[5..0]), ACMD flag (bit7) */
					DWORD arg,  /* Command argument */
					UINT rt,	/* Expected response type. None(0), Short(1) or Long(2), 4 Read Data, 8 Write Data */
					DWORD *buff /* Response return buffer */
)
{
	int ret = sdcard_send_command(arg, idx, rt);
    uint32_t r[SD_CMD_RESPONSE_SIZE/4];
    csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);

    buff[0] = r[0];
    buff[1] = r[1];
    buff[2] = r[2];
    buff[3] = r[3];

	return ret; /* Return */
}

/*-----------------------------------------------------------------------*/
/* Wait card ready                                                       */
/*-----------------------------------------------------------------------*/

static int wait_ready(		   /* Returns 1 when card is tran state, otherwise returns 0 */
					  WORD tmr /* Timeout in unit of 1ms */
)
{
	DWORD rc;

	Timer[0] = tmr;
	while (Timer[0]--)
	{
		if (send_cmd(CMD13, (DWORD)CardRCA << 16, 1, &rc) && ((rc & 0x01E00) == 0x00800))
			break;

		dly_us(1000);

		/* This loop takes a time. Insert rot_rdq() here for multitask envilonment. */
	}
	return Timer[0] ? 1 : 0;
}

/*-----------------------------------------------------------------------*/
/* Swap byte order                                                       */
/*-----------------------------------------------------------------------*/

static void bswap_cp(BYTE *dst, const DWORD *src)
{
	DWORD d;

	d = *src;
	*dst++ = (BYTE)(d >> 24);
	*dst++ = (BYTE)(d >> 16);
	*dst++ = (BYTE)(d >> 8);
	*dst++ = (BYTE)(d >> 0);
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize0(BYTE pdrv)
{
    uint16_t rca, timeout;
	UINT cmd, n;
	DWORD resp[4];
	BYTE ty;

	/* Set SD clk freq to Initialization frequency */
	sdcard_set_clk_freq(SDCARD_CLK_FREQ_INIT, 0);
	busy_wait(1);

	for (timeout=1000; timeout>0; timeout--) {
		/* Set SDCard in SPI Mode (generate 80 dummy clocks) */
		sdphy_init_initialize_write(1);
		busy_wait(1);

		/* Set SDCard in Idle state */
		if (sdcard_go_idle() == SD_OK)
			break;
		busy_wait(1);
	}
	if (timeout == 0)
		return 0;

	CardRCA = 0;

	/*---- Card is 'idle' state ----*/

	int Timer = 1000;				   /* Initialization timeout of 1000 msec */
	if (send_cmd(CMD8, 0x1AA, 1, resp) /* Is the card SDv2? */
		&& (resp[0] & 0xFFF) == 0x1AA)
	{ /* The card can work at vdd range of 2.7-3.6V */
		do
		{ /* Wait while card is busy state (use ACMD41 with HCS bit) */

			dly_us(1000); /* 1ms */

			/* This loop takes a time. Insert task rotation here for multitask envilonment. */

			if (!Timer--)
				goto di_fail;
		} while (!send_cmd(ACMD41, 0x40FF8000, 1, resp) || !(resp[0] & 0x80000000));
		ty = (resp[0] & 0x40000000) ? CT_SD2 | CT_BLOCK : CT_SD2; /* Check CCS bit in the OCR */
	}
	else
	{ /* SDv1 or MMCv3 */
		if (send_cmd(ACMD41, 0x00FF8000, 1, resp))
		{
			ty = CT_SD1;
			cmd = ACMD41; /* ACMD41 is accepted -> SDC Ver1 */
		}
		else
		{
			ty = CT_MMC;
			cmd = CMD1; /* ACMD41 is rejected -> MMC */
		}
		do
		{ /* Wait while card is busy state (use ACMD41 or CMD1) */

			/* This loop will take a time. Insert task rotation here for multitask envilonment. */

			if (!Timer--)
				goto di_fail;
		} while (!send_cmd(cmd, 0x00FF8000, 1, resp) || !(resp[0] & 0x80000000));
	}

	CardType = ty;				   /* Save card type */
	bswap_cp(&CardInfo[32], resp); /* Save OCR */

	/*---- Card is 'ready' state ----*/

	if (!send_cmd(CMD2, 0, 2, resp))
		goto di_fail; /* Enter ident state */
	for (n = 0; n < 4; n++)
		bswap_cp(&CardInfo[n * 4 + 16], &resp[n]); /* Save CID */

	/*---- Card is 'ident' state ----*/

	if (ty & CT_SDC)
	{ /* SDC: Get generated RCA and save it */
		if (!send_cmd(CMD3, 0, 1, resp))
			goto di_fail;
		CardRCA = (WORD)(resp[0] >> 16);
	}
	else
	{ /* MMC: Assign RCA to the card */
		if (!send_cmd(CMD3, 1 << 16, 1, resp))
			goto di_fail;
		CardRCA = 1;
	}

	/*---- Card is 'stby' state ----*/

	if (!send_cmd(CMD9, (DWORD)CardRCA << 16, 2, resp))
		goto di_fail; /* Get CSD and save it */
	for (n = 0; n < 4; n++)
		bswap_cp(&CardInfo[n * 4], &resp[n]);
	if (!send_cmd(CMD7, (DWORD)CardRCA << 16, 1, resp))
		goto di_fail; /* Select card */

	/*---- Card is 'tran' state ----*/

	if (!(ty & CT_BLOCK))
	{ /* Set data block length to 512 (for byte addressing cards) */
		if (!send_cmd(CMD16, 512, 1, resp) || (resp[0] & 0xFDF90000))
			goto di_fail;
	}

	if (ty & CT_SDC)
	{ /* Set wide bus mode (for SDCs) */
		if (!send_cmd(ACMD6, 2, 1, resp) || (resp[0] & 0xFDF90000))
		{ /* Set wide bus mode of SDC */
			goto di_fail;
		}

		//SDC_CONTROL = 1; /* Enable Wide bus */
	}

	/* Select card */
	if (sdcard_select_card(CardRCA) != SD_OK)
		return 0;

	/* Switch speed */
	if (sdcard_switch(SD_SWITCH_SWITCH, SD_GROUP_ACCESSMODE, SD_SPEED_SDR25) != SD_OK)
		return 0;

	/* Send SCR */
	/* FIXME: add scr decoding (optional) */
	if (sdcard_app_cmd(CardRCA) != SD_OK)
		return 0;
	if (sdcard_app_send_scr() != SD_OK)
		return 0;

	/* Set block length */
	if (sdcard_app_set_blocklen(512) != SD_OK)
		return 0;


	Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
	return Stat;

	/* Allocate 1Mb to the cache! */
	//sd_cache_init(0x04000000, 2048);


di_fail:
	print("Init Fail\r\n");
	
	Stat |= STA_NOINIT; /* Set STA_NOINIT */
	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status0(BYTE pdrv)
{
	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
	BYTE pdrv,
	BYTE cmd,  /* Control code */
	void *buff /* Buffer to send/receive data block */
)
{
	DRESULT res;
	BYTE b, *ptr = buff, sdstat[64];
	DWORD resp[4], d, *dp, st, ed;
	static const DWORD au_size[] = {1, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 24576, 32768, 49152, 65536, 131072};

	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	res = RES_ERROR;

	switch (cmd)
	{
	case CTRL_SYNC: /* Make sure that all data has been written on the media */
		if (wait_ready(500))
			res = RES_OK; /* Wait for card enters tarn state */
		break;

	case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
		if ((CardInfo[0] >> 6) == 1)
		{ /* SDC CSD v2.0 */
			d = CardInfo[9] + ((WORD)CardInfo[8] << 8) + ((DWORD)(CardInfo[7] & 63) << 16) + 1;
			*(DWORD *)buff = d << 10;
		}
		else
		{ /* MMC or SDC CSD v1.0 */
			b = (CardInfo[5] & 15) + ((CardInfo[10] & 128) >> 7) + ((CardInfo[9] & 3) << 1) + 2;
			d = (CardInfo[8] >> 6) + ((WORD)CardInfo[7] << 2) + ((WORD)(CardInfo[6] & 3) << 10) + 1;
			*(DWORD *)buff = d << (b - 9);
		}
		res = RES_OK;
		break;

	case GET_BLOCK_SIZE: /* Get erase block size in unit of sectors (DWORD) */
		if (CardType & CT_SD2)
		{ /* SDC ver 2.00 */
			if (disk_ioctl(pdrv, MMC_GET_SDSTAT, sdstat))
				break;
			*(DWORD *)buff = au_size[sdstat[10] >> 4];
		}
		else
		{ /* SDC ver 1.XX or MMC */
			if (CardType & CT_SD1)
			{ /* SDC v1 */
				*(DWORD *)buff = (((CardInfo[10] & 63) << 1) + ((WORD)(CardInfo[11] & 128) >> 7) + 1) << ((CardInfo[13] >> 6) - 1);
			}
			else
			{ /* MMC */
				*(DWORD *)buff = ((WORD)((CardInfo[10] & 124) >> 2) + 1) * (((CardInfo[11] & 3) << 3) + ((CardInfo[11] & 224) >> 5) + 1);
			}
		}
		res = RES_OK;
		break;

	case CTRL_TRIM: /* Erase a block of sectors */
		if (!(CardType & CT_SDC) || (!(CardInfo[0] >> 6) && !(CardInfo[10] & 0x40)))
			break; /* Check if sector erase can be applied to the card */
		dp = buff;
		st = dp[0];
		ed = dp[1];
		if (!(CardType & CT_BLOCK))
		{
			st *= 512;
			ed *= 512;
		}
		if (send_cmd(CMD32, st, 1, resp) && send_cmd(CMD33, ed, 1, resp) && send_cmd(CMD38, 0, 1, resp) && wait_ready(30000))
		{
			res = RES_OK;
		}
		break;

	// case CTRL_POWER_OFF:
	// 	//power_off(); /* Power off */
	// 	res = RES_OK;
	// 	break;

	case MMC_GET_TYPE: /* Get card type flags (1 byte) */
		*ptr = CardType;
		res = RES_OK;
		break;

	case MMC_GET_CSD: /* Get CSD (16 bytes) */
		memcpy(buff, &CardInfo[0], 16);
		res = RES_OK;
		break;

	case MMC_GET_CID: /* Get CID (16 bytes) */
		memcpy(buff, &CardInfo[16], 16);
		res = RES_OK;
		break;

	case MMC_GET_OCR: /* Get OCR (4 bytes) */
		memcpy(buff, &CardInfo[32], 4);
		res = RES_OK;
		break;

	case MMC_GET_SDSTAT: /* Receive SD status as a data block (64 bytes) */
		if (CardType & CT_SDC)
		{ /* SDC */
			print("Get Data Block ");
			if (wait_ready(500))
			{
				print("Ready?");
				//SDC_BLOCKSIZE = 63;
				//SDC_BLOCKCOUNT = 0x1;
				//ready_reception(1, 64);			 /* Ready to receive data blocks */
				if (send_cmd(ACMD13, 0, 1 | 0x4, resp) /* Start to read */
					&& !(resp[0] & 0xC0580000))
				{
					res = RES_OK;
				}
			}
			print("\r\n");
			//stop_transfer(); /* Close data path */
		}
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}
