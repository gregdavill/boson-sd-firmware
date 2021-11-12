// This file is Copyright (c) 2017-2020 Florent Kermarrec <florent@enjoy-digital.fr>
// This file is Copyright (c) 2019-2020 Gabriel L. Somlo <gsomlo@gmail.com>
// This file is Copyright (c) 2019 Kees Jongenburger <kees.jongenburger@gmail.com>
// This file is Copyright (c) 2018 bunnie <bunnie@kosagi.com>
// This file is Copyright (c) 2020 Antmicro <www.antmicro.com>
// This file is Copyright (c) 2021 Greg Davill <greg.davill@gmail.com>
// License: BSD


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>
#include <system.h>

#include "ff.h"
#include "diskio.h" /* Common include file for FatFs and disk I/O layer */
#include "sdcard.h"

//#define USE_CACHE

//#define SDCARD_CMD23_SUPPORT /* SET_BLOCK_COUNT */
#define SDCARD_CMD18_SUPPORT /* READ_MULTIPLE_BLOCK */
#define SDCARD_CMD25_SUPPORT /* WRITE_MULTIPLE_BLOCK */

#ifndef SDCARD_CLK_FREQ_INIT
#define SDCARD_CLK_FREQ_INIT 400000
#endif

#ifndef SDCARD_CLK_FREQ
#define SDCARD_CLK_FREQ 25000000UL
#endif

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */


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

/*-----------------------------------------------------------------------*/
/* Helpers                                                               */
/*-----------------------------------------------------------------------*/

#define dly_us(n) busy_wait_us(n)

#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))

/*-----------------------------------------------------------------------*/
/* SDCard command helpers                                                */
/*-----------------------------------------------------------------------*/

int sdcard_wait_cmd_done(void) {
	unsigned int event;
#ifdef SDCARD_DEBUG
	uint32_t r[SD_CMD_RESPONSE_SIZE/4];
#endif
	for (;;) {
		event = sdcore_cmd_event_read();
#ifdef SDCARD_DEBUG
		//printf("cmdevt: %08x\n", event);
#endif
		busy_wait_us(20);
		if (event & 0x1)
			break;
	}
	
	/* Load bearing delay. 
	 * When SDCARD_DEBUG is undefined 
	 * this delay is required for correct operation */
	busy_wait_us(50);

#ifdef SDCARD_DEBUG
	csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);
	printf("%08x %08x %08x %08x\n", r[0], r[1], r[2], r[3]);
#endif
	if (event & 0x4)
		return SD_TIMEOUT;
	if (event & 0x8)
		return SD_CRCERROR;
	return SD_OK;
}

int sdcard_wait_data_done(void) {
	unsigned int event;
	for (;;) {
		event = sdcore_data_event_read();
#ifdef SDCARD_DEBUG
		//printf("dataevt: %08x\n", event);
#endif
		if (event & 0x1)
			break;
		busy_wait_us(20);
	}

	busy_wait_us(50);

	if (event & 0x4)
		return SD_TIMEOUT;
	else if (event & 0x8)
		return SD_CRCERROR;
	return SD_OK;
}

/*-----------------------------------------------------------------------*/
/* SDCard clocker functions                                              */
/*-----------------------------------------------------------------------*/

/* round up to closest power-of-two */
static inline uint32_t pow2_round_up(uint32_t r) {
	r--;
	r |= r >>  1;
	r |= r >>  2;
	r |= r >>  4;
	r |= r >>  8;
	r |= r >> 16;
	r++;
	return r;
}

void sdcard_set_clk_freq(unsigned long clk_freq, int show) {
	uint32_t divider;
	divider = clk_freq ? CONFIG_CLOCK_FREQUENCY/clk_freq : 256;
	divider = pow2_round_up(divider);
	divider = min(max(divider, 2), 256);
#ifdef SDCARD_DEBUG
	show = 1;
#endif
	if (show) {
		/* this is the *effective* new clk_freq */
		clk_freq = CONFIG_CLOCK_FREQUENCY/divider;
		printf("Setting SDCard clk freq to ");
		if (clk_freq > 1000000)
			printf("%ld MHz\n", clk_freq/1000000);
		else
			printf("%ld KHz\n", clk_freq/1000);
	}
	sdphy_clocker_divider_write(divider);
}

/*-----------------------------------------------------------------------*/
/* SDCard commands functions                                             */
/*-----------------------------------------------------------------------*/

static inline int sdcard_send_command(uint32_t arg, uint8_t cmd, uint8_t rsp) {
	sdcore_cmd_argument_write(arg);
	sdcore_cmd_command_write((cmd << 8) | rsp);
	sdcore_cmd_send_write(1);
	return sdcard_wait_cmd_done();
}

int sdcard_go_idle(void) {
#ifdef SDCARD_DEBUG
	printf("CMD0: GO_IDLE\n");
#endif
	return sdcard_send_command(0, 0, SDCARD_CTRL_RESPONSE_NONE);
}

int sdcard_send_ext_csd(void) {
	uint32_t arg = 0x000001aa;
#ifdef SDCARD_DEBUG
	printf("CMD8: SEND_EXT_CSD, arg: 0x%08x\n", arg);
#endif
	return sdcard_send_command(arg, 8, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_app_cmd(uint16_t rca) {
#ifdef SDCARD_DEBUG
	printf("CMD55: APP_CMD\n");
#endif
	return sdcard_send_command(rca << 16, 55, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_app_send_op_cond(int hcs) {
	uint32_t arg = 0x10ff8000;
	if (hcs)
		arg |= 0x60000000;
#ifdef SDCARD_DEBUG
	printf("ACMD41: APP_SEND_OP_COND, arg: %08x\n", arg);
#endif
	return sdcard_send_command(arg, 41, SDCARD_CTRL_RESPONSE_SHORT_BUSY);
}

int sdcard_all_send_cid(void) {
#ifdef SDCARD_DEBUG
	printf("CMD2: ALL_SEND_CID\n");
#endif
	return sdcard_send_command(0, 2, SDCARD_CTRL_RESPONSE_LONG);
}

int sdcard_set_relative_address(void) {
#ifdef SDCARD_DEBUG
	printf("CMD3: SET_RELATIVE_ADDRESS\n");
#endif
	return sdcard_send_command(0, 3, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_send_cid(uint16_t rca) {
#ifdef SDCARD_DEBUG
	printf("CMD10: SEND_CID\n");
#endif
	return sdcard_send_command(rca << 16, 10, SDCARD_CTRL_RESPONSE_LONG);
}

int sdcard_send_csd(uint16_t rca) {
#ifdef SDCARD_DEBUG
	printf("CMD9: SEND_CSD\n");
#endif
	return sdcard_send_command(rca << 16, 9, SDCARD_CTRL_RESPONSE_LONG);
}

int sdcard_select_card(uint16_t rca) {
#ifdef SDCARD_DEBUG
	printf("CMD7: SELECT_CARD\n");
#endif
	return sdcard_send_command(rca << 16, 7, SDCARD_CTRL_RESPONSE_SHORT_BUSY);
}

int sdcard_go_inactive_state() {
	return sdcard_send_command(CardRCA << 16, 7, SDCARD_CTRL_RESPONSE_SHORT_BUSY);
}

int sdcard_app_set_bus_width(void) {
#ifdef SDCARD_DEBUG
	printf("ACMD6: SET_BUS_WIDTH\n");
#endif
	return sdcard_send_command(2, 6, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_switch(unsigned int mode, unsigned int group, unsigned int value) {
	unsigned int arg;
	arg = (mode << 31) | 0xffffff;
	arg &= ~(0xf << (group * 4));
	arg |= value << (group * 4);
#ifdef SDCARD_DEBUG
	printf("CMD6: SWITCH_FUNC\n");
#endif
	sdcore_block_length_write(64);
	sdcore_block_count_write(1);
	while (sdcard_send_command(arg, 6,
		(SDCARD_CTRL_DATA_TRANSFER_READ << 5) |
		SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return sdcard_wait_data_done();
}

int sdcard_app_send_scr(void) {
#ifdef SDCARD_DEBUG
	printf("CMD51: APP_SEND_SCR\n");
#endif
	sdcore_block_length_write(8);
	sdcore_block_count_write(1);
	while (sdcard_send_command(0, 51,
		(SDCARD_CTRL_DATA_TRANSFER_READ << 5) |
		SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return sdcard_wait_data_done();
}

int sdcard_app_set_blocklen(unsigned int blocklen) {
#ifdef SDCARD_DEBUG
	printf("CMD16: SET_BLOCKLEN\n");
#endif
	return sdcard_send_command(blocklen, 16, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_app_set_wr_block_erase_count(unsigned int blocks) {
#ifdef SDCARD_DEBUG
	printf("ACMD23: SET_WR_BLK_ERASE_COUNT\n");
#endif
	sdcard_app_cmd(CardRCA);
	return sdcard_send_command(blocks, 23, SDCARD_CTRL_RESPONSE_SHORT);
}


int sdcard_write_single_block(unsigned int blockaddr) {
#ifdef SDCARD_DEBUG
	printf("CMD24: WRITE_SINGLE_BLOCK\n");
#endif
	sdcore_block_length_write(512);
	sdcore_block_count_write(1);
	while (sdcard_send_command(blockaddr, 24,
	    (SDCARD_CTRL_DATA_TRANSFER_WRITE << 5) |
	    SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return SD_OK;
}

int sdcard_write_multiple_block(unsigned int blockaddr, unsigned int blockcnt) {
#ifdef SDCARD_DEBUG
	printf("CMD25: WRITE_MULTIPLE_BLOCK\n");
#endif

	//sdcard_app_set_wr_block_erase_count(blockcnt);

	sdcore_block_length_write(512);
	sdcore_block_count_write(blockcnt);
	while (sdcard_send_command(blockaddr, 25,
	    (SDCARD_CTRL_DATA_TRANSFER_WRITE << 5) |
	    SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return SD_OK;
}

int sdcard_read_single_block(unsigned int blockaddr) {
#ifdef SDCARD_DEBUG
	printf("CMD17: READ_SINGLE_BLOCK\n");
#endif
	sdcore_block_length_write(512);
	sdcore_block_count_write(1);
	while (sdcard_send_command(blockaddr, 17,
	    (SDCARD_CTRL_DATA_TRANSFER_READ << 5) |
	    SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return sdcard_wait_data_done();
}

int sdcard_read_multiple_block(unsigned int blockaddr, unsigned int blockcnt) {
#ifdef SDCARD_DEBUG
	printf("CMD18: READ_MULTIPLE_BLOCK\n");
#endif
	sdcore_block_length_write(512);
	sdcore_block_count_write(blockcnt);
	while (sdcard_send_command(blockaddr, 18,
	    (SDCARD_CTRL_DATA_TRANSFER_READ << 5) |
	    SDCARD_CTRL_RESPONSE_SHORT) != SD_OK);
	return sdcard_wait_data_done();
}

int sdcard_stop_transmission(void) {
#ifdef SDCARD_DEBUG
	printf("CMD12: STOP_TRANSMISSION\n");
#endif
	dly_us(100);
	return sdcard_send_command(0, 12, SDCARD_CTRL_RESPONSE_SHORT_BUSY);
}

int sdcard_send_status(uint16_t rca) {
#ifdef SDCARD_DEBUG
	printf("CMD13: SEND_STATUS\n");
#endif
	return sdcard_send_command(rca << 16, 13, SDCARD_CTRL_RESPONSE_SHORT);
}

int sdcard_set_block_count(unsigned int blockcnt) {
#ifdef SDCARD_DEBUG
	printf("CMD23: SET_BLOCK_COUNT\n");
#endif
	return sdcard_send_command(blockcnt, 23, SDCARD_CTRL_RESPONSE_SHORT);
}

uint16_t sdcard_decode_rca(void) {
	uint32_t r[SD_CMD_RESPONSE_SIZE/4];
	csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);
	return (r[3] >> 16) & 0xffff;
}

#ifdef SDCARD_DEBUG
void sdcard_decode_cid(void) {
	uint32_t r[SD_CMD_RESPONSE_SIZE/4];
	csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);
	printf(
		"CID Register: 0x%08x%08x%08x%08x\n"
		"Manufacturer ID: 0x%x\n"
		"Application ID 0x%x\n"
		"Product name: %c%c%c%c%c\n"
		"CRC: %02x\n"
		"Production date(m/yy): %d/%d\n"
		"PSN: %08x\n"
		"OID: %c%c\n",

		r[0], r[1], r[2], r[3],

		(r[0] >> 16) & 0xffff,

		r[0] & 0xffff,

		(r[1] >> 24) & 0xff, (r[1] >> 16) & 0xff,
		(r[1] >>  8) & 0xff, (r[1] >>  0) & 0xff, (r[2] >> 24) & 0xff,

		r[3] & 0xff,

		(r[3] >>  8) & 0x0f, (r[3] >> 12) & 0xff,

		(r[3] >> 24) | (r[2] <<  8),

		(r[0] >> 16) & 0xff, (r[0] >>  8) & 0xff
	);
}

void sdcard_decode_csd(void) {
	uint32_t r[SD_CMD_RESPONSE_SIZE/4];
	csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);
	/* FIXME: only support CSR structure version 2.0 */
	printf(
		"CSD Register: 0x%08x%08x%08x%08x\n"
		"Max data transfer rate: %d MB/s\n"
		"Max read block length: %d bytes\n"
		"Device size: %d GB\n",

		r[0], r[1], r[2], r[3],

		(r[0] >> 24) & 0xff,

		(1 << ((r[1] >> 16) & 0xf)),

		((r[2] >> 16) + ((r[1] & 0xff) << 16) + 1) * 512 / (1024 * 1024)
	);
}
#endif

/*-----------------------------------------------------------------------*/
/* SDCard user functions                                                 */
/*-----------------------------------------------------------------------*/

#ifdef CSR_SDBLOCK2MEM_BASE

void sdcard_read(uint32_t block, uint32_t count, uint8_t* buf)
{
#ifdef SDCARD_DEBUG
	printf("sdcard_read(), block=%08x, count=%08x, buff=%08x\n", block, count, buf);
#endif
	while (count) {
		uint32_t nblocks;
#ifdef SDCARD_CMD18_SUPPORT
		nblocks = count;
#else
		nblocks = 1;
#endif
		/* Initialize DMA Writer */
		sdblock2mem_dma_enable_write(0);
		sdblock2mem_dma_base_write((uint64_t)(uintptr_t) buf);
		sdblock2mem_dma_length_write(512*nblocks);
		sdblock2mem_dma_enable_write(1);

		/* Read Block(s) from SDCard */
#ifdef SDCARD_CMD23_SUPPORT
		sdcard_set_block_count(nblocks);
#endif
		if (nblocks > 1)
			sdcard_read_multiple_block(block, nblocks);
		else
			sdcard_read_single_block(block);

		/* Wait for DMA Writer to complete */
		while ((sdblock2mem_dma_done_read() & 0x1) == 0);

		/* Stop transmission (Only for multiple block reads) */
		if (nblocks > 1)
			sdcard_stop_transmission();

	
#ifdef SDCARD_DEBUG

	flush_cpu_dcache();
	flush_l2_cache();
	dump_bytes(buf, 512, buf);
#endif
		/* Update Block/Buffer/Count */
		block += nblocks;
		buf   += 512*nblocks;
		count -= nblocks;
	}

	sdblock2mem_dma_enable_write(0);
	
	/* Flush caches */
	flush_cpu_dcache();
	flush_l2_cache();

}

#endif


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
	if (idx & 0x80)
	{														/* Send a CMD55 prior to the specified command if it is ACMD class */
		if (!send_cmd(CMD55, (DWORD)CardRCA << 16, 1, buff) /* When CMD55 is failed, */
			|| !(buff[0] & 0x00000020))
			return 0; /* exit with error */
	}
	idx &= 0x3F; /* Mask out ACMD flag */
	
	int ret = sdcard_send_command(arg, idx, rt);
    uint32_t r[SD_CMD_RESPONSE_SIZE/4];
    csr_rd_buf_uint32(CSR_SDCORE_CMD_RESPONSE_ADDR,
			  r, SD_CMD_RESPONSE_SIZE/4);

    buff[0] = r[3];
    buff[1] = r[2];
    buff[2] = r[1];
    buff[3] = r[0];

	return 1; /* Return */
}

/*-----------------------------------------------------------------------*/
/* Wait card ready                                                       */
/*-----------------------------------------------------------------------*/

static int wait_ready(		   /* Returns 1 when card is tran state, otherwise returns 0 */
					  WORD tmr /* Timeout in unit of 1ms */
)
{
	DWORD rc[4];

	Timer[0] = tmr;
	while (Timer[0]--)
	{
		if (send_cmd(CMD13, (DWORD)CardRCA << 16, 1, rc) && ((rc[0] & 0x01E00) == 0x00800))
			break;

		dly_us(10);

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

DSTATUS disk_initialize(BYTE pdrv)
{
    uint16_t timeout;
	UINT cmd, n;
	DWORD resp[4];
	BYTE ty;

	printf("disk_initialize()\n");

	/* Set SD clk freq to Initialization frequency */
	sdcard_set_clk_freq(SDCARD_CLK_FREQ_INIT, 0);
	dly_us(1000);

	for (timeout=20; timeout>0; timeout--) {
		/* Set SDCard in SPI Mode (generate 80 dummy clocks) */
		sdphy_init_initialize_write(1);
		dly_us(1000);

		/* Set SDCard in Idle state */
		if (sdcard_go_idle() == SD_OK)
			break;
	}
	if (timeout == 0)
		return 0;

	CardRCA = 0;

	/*---- Card is 'idle' state ----*/

	int Timer = 50;				   /* Initialization timeout of 500 msec */
	if (send_cmd(CMD8, 0x1AA, 1, resp) /* Is the card SDv2? */
		&& (resp[0] & 0xFFF) == 0x1AA)
	{ /* The card can work at vdd range of 2.7-3.6V */
		do
		{ /* Wait while card is busy state (use ACMD41 with HCS bit) */

			dly_us(10); /* 1ms */

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

	sdcard_set_clk_freq((unsigned long)20e6, 1);

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

	/* Set bus width */
	if (sdcard_app_cmd(CardRCA) != SD_OK)
		return 0;

	if(sdcard_app_set_bus_width() != SD_OK)
		return 0;


	/* Switch speed */
	if (sdcard_switch(SD_SWITCH_SWITCH, SD_GROUP_ACCESSMODE, SD_SPEED_SDR25) != SD_OK)
		return 0;


	sdcard_set_clk_freq((unsigned long)50e6, 1);

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

di_fail:
	printf("Init Fail :(\n");
	
	Stat |= STA_NOINIT; /* Set STA_NOINIT */
	return Stat;
}


/*-----------------------------------------------------------------------*/
/* SDCard FatFs disk functions                                           */
/*-----------------------------------------------------------------------*/


DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t sector, UINT count) {

#ifdef USE_CACHE
	if(count == 1){
		if(sd_cache_read(buff, sector, 1))
		{
			/* Found a block in our cache */
			/* The call already performs a copy into our buffer */
			return RES_OK;
		}
	}
#endif
	sdcard_read(sector, count, buff);

#ifdef USE_CACHE
	/* We have read a block, add it to the cache */
	if(count == 1){
		if(sd_cache_create(buff, sector, 1))
		{
			/* Return true either way */
		}
	}
#endif

	return RES_OK;
}


DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count){

	//printf("disk_write() sector=%08x,buff=%08x,cnt=%u\n", sector, buff, count);

    sdcard_write(sector, count, buff);
	
#ifdef USE_CACHE
	/* We have written a block, if this block exists in cache then update */
	if(count == 1){
		if(sd_cache_update(buff, sector, 1))
		{
			/* Return true either way */
		}
	}
#endif

    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE pdrv)
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
		if (wait_ready(50))
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
		if (send_cmd(CMD32, st, 1, resp) && send_cmd(CMD33, ed, 1, resp) && send_cmd(CMD38, 0, 1, resp) && wait_ready(3000))
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
			printf("Get Data Block ");
			if (wait_ready(50))
			{
				printf("Ready?");
				//SDC_BLOCKSIZE = 63;
				//SDC_BLOCKCOUNT = 0x1;
				//ready_reception(1, 64);			 /* Ready to receive data blocks */
				if (send_cmd(ACMD13, 0, 1 | 0x4, resp) /* Start to read */
					&& !(resp[0] & 0xC0580000))
				{
					res = RES_OK;
				}
			}
			printf("\r\n");
			//stop_transfer(); /* Close data path */
		}
		break;

	default:
		res = RES_PARERR;
	}

	return res;
}
