/*

	Frontend/Card driver for TwinHan DST Frontend
	Copyright (C) 2003 Jamie Honan
	Copyright (C) 2004, 2005 Manu Abraham (manu@kromtek.com)

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <asm/div64.h>

#include "dvb_frontend.h"
#include "dst_priv.h"
#include "dst_common.h"


static unsigned int verbose = 1;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "verbose startup messages, default is 1 (yes)");

static unsigned int debug = 1;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug messages, default is 0 (yes)");

static unsigned int dst_addons;
module_param(dst_addons, int, 0644);
MODULE_PARM_DESC(dst_addons, "CA daughterboard, default is 0 (No addons)");

#define dprintk	if (debug) printk

#define HAS_LOCK	1
#define ATTEMPT_TUNE	2
#define HAS_POWER	4

static void dst_packsize(struct dst_state* state, int psize)
{
	union dst_gpio_packet bits;

	bits.psize = psize;
	bt878_device_control(state->bt, DST_IG_TS, &bits);
}

int dst_gpio_outb(struct dst_state* state, u32 mask, u32 enbb, u32 outhigh, int delay)
{
	union dst_gpio_packet enb;
	union dst_gpio_packet bits;
	int err;

	enb.enb.mask = mask;
	enb.enb.enable = enbb;
	if (verbose > 4)
		dprintk("%s: mask=[%04x], enbb=[%04x], outhigh=[%04x]\n", __FUNCTION__, mask, enbb, outhigh);

	if ((err = bt878_device_control(state->bt, DST_IG_ENABLE, &enb)) < 0) {
		dprintk("%s: dst_gpio_enb error (err == %i, mask == %02x, enb == %02x)\n", __FUNCTION__, err, mask, enbb);
		return -EREMOTEIO;
	}
	udelay(1000);
	/* because complete disabling means no output, no need to do output packet */
	if (enbb == 0)
		return 0;

	if (delay)
		msleep(10);

	bits.outp.mask = enbb;
	bits.outp.highvals = outhigh;

	if ((err = bt878_device_control(state->bt, DST_IG_WRITE, &bits)) < 0) {
		dprintk("%s: dst_gpio_outb error (err == %i, enbb == %02x, outhigh == %02x)\n", __FUNCTION__, err, enbb, outhigh);
		return -EREMOTEIO;
	}
	return 0;
}
EXPORT_SYMBOL(dst_gpio_outb);

int dst_gpio_inb(struct dst_state *state, u8 * result)
{
	union dst_gpio_packet rd_packet;
	int err;

	*result = 0;

	if ((err = bt878_device_control(state->bt, DST_IG_READ, &rd_packet)) < 0) {
		dprintk("%s: dst_gpio_inb error (err == %i)\n", __FUNCTION__, err);
		return -EREMOTEIO;
	}

	*result = (u8) rd_packet.rd.value;
	return 0;
}
EXPORT_SYMBOL(dst_gpio_inb);

int rdc_reset_state(struct dst_state *state)
{
	if (verbose > 1)
		dprintk("%s: Resetting state machine\n", __FUNCTION__);

	if (dst_gpio_outb(state, RDC_8820_INT, RDC_8820_INT, 0, NO_DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		return -1;
	}

	msleep(10);

	if (dst_gpio_outb(state, RDC_8820_INT, RDC_8820_INT, RDC_8820_INT, NO_DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		msleep(10);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(rdc_reset_state);

int rdc_8820_reset(struct dst_state *state)
{
	if (verbose > 1)
		dprintk("%s: Resetting DST\n", __FUNCTION__);

	if (dst_gpio_outb(state, RDC_8820_RESET, RDC_8820_RESET, 0, NO_DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		return -1;
	}
	udelay(1000);
	if (dst_gpio_outb(state, RDC_8820_RESET, RDC_8820_RESET, RDC_8820_RESET, DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(rdc_8820_reset);

int dst_pio_enable(struct dst_state *state)
{
	if (dst_gpio_outb(state, ~0, RDC_8820_PIO_0_ENABLE, 0, NO_DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		return -1;
	}
	udelay(1000);
	return 0;
}
EXPORT_SYMBOL(dst_pio_enable);

int dst_pio_disable(struct dst_state *state)
{
	if (dst_gpio_outb(state, ~0, RDC_8820_PIO_0_DISABLE, RDC_8820_PIO_0_DISABLE, NO_DELAY) < 0) {
		dprintk("%s: dst_gpio_outb ERROR !\n", __FUNCTION__);
		return -1;
	}
	if (state->type_flags & DST_TYPE_HAS_FW_1)
		udelay(1000);

	return 0;
}
EXPORT_SYMBOL(dst_pio_disable);

int dst_wait_dst_ready(struct dst_state *state, u8 delay_mode)
{
	u8 reply;
	int i;

	for (i = 0; i < 200; i++) {
		if (dst_gpio_inb(state, &reply) < 0) {
			dprintk("%s: dst_gpio_inb ERROR !\n", __FUNCTION__);
			return -1;
		}

		if ((reply & RDC_8820_PIO_0_ENABLE) == 0) {
			if (verbose > 4)
				dprintk("%s: dst wait ready after %d\n", __FUNCTION__, i);
			return 1;
		}
		msleep(10);
	}
	if (verbose > 1)
		dprintk("%s: dst wait NOT ready after %d\n", __FUNCTION__, i);

	return 0;
}
EXPORT_SYMBOL(dst_wait_dst_ready);

int dst_error_recovery(struct dst_state *state)
{
	dprintk("%s: Trying to return from previous errors...\n", __FUNCTION__);
	dst_pio_disable(state);
	msleep(10);
	dst_pio_enable(state);
	msleep(10);

	return 0;
}
EXPORT_SYMBOL(dst_error_recovery);

int dst_error_bailout(struct dst_state *state)
{
	dprintk("%s: Trying to bailout from previous error...\n", __FUNCTION__);
	rdc_8820_reset(state);
	dst_pio_disable(state);
	msleep(10);

	return 0;
}
EXPORT_SYMBOL(dst_error_bailout);


int dst_comm_init(struct dst_state* state)
{
	if (verbose > 1)
		dprintk ("%s: Initializing DST..\n", __FUNCTION__);
	if ((dst_pio_enable(state)) < 0) {
		dprintk("%s: PIO Enable Failed.\n", __FUNCTION__);
		return -1;
	}
	if ((rdc_reset_state(state)) < 0) {
		dprintk("%s: RDC 8820 State RESET Failed.\n", __FUNCTION__);
		return -1;
	}
	if (state->type_flags & DST_TYPE_HAS_FW_1)
		msleep(100);
	else
		msleep(5);

	return 0;
}
EXPORT_SYMBOL(dst_comm_init);


int write_dst(struct dst_state *state, u8 *data, u8 len)
{
	struct i2c_msg msg = {
		.addr = state->config->demod_address,.flags = 0,.buf = data,.len = len
	};

	int err;
	int cnt;
	if (debug && (verbose > 4)) {
		u8 i;
		if (verbose > 4) {
			dprintk("%s writing", __FUNCTION__);
			for (i = 0; i < len; i++)
				dprintk(" %02x", data[i]);
			dprintk("\n");
		}
	}
	for (cnt = 0; cnt < 2; cnt++) {
		if ((err = i2c_transfer(state->i2c, &msg, 1)) < 0) {
			dprintk("%s: _write_dst error (err == %i, len == 0x%02x, b0 == 0x%02x)\n", __FUNCTION__, err, len, data[0]);
			dst_error_recovery(state);
			continue;
		} else
			break;
	}

	if (cnt >= 2) {
		if (verbose > 1)
			printk("%s: RDC 8820 RESET...\n", __FUNCTION__);
		dst_error_bailout(state);

		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(write_dst);

int read_dst(struct dst_state *state, u8 * ret, u8 len)
{
	struct i2c_msg msg = {.addr = state->config->demod_address,.flags = I2C_M_RD,.buf = ret,.len = len };
	int err;
	int cnt;

	for (cnt = 0; cnt < 2; cnt++) {
		if ((err = i2c_transfer(state->i2c, &msg, 1)) < 0) {

			dprintk("%s: read_dst error (err == %i, len == 0x%02x, b0 == 0x%02x)\n", __FUNCTION__, err, len, ret[0]);
			dst_error_recovery(state);

			continue;
		} else
			break;
	}
	if (cnt >= 2) {
		if (verbose > 1)
			printk("%s: RDC 8820 RESET...\n", __FUNCTION__);
		dst_error_bailout(state);

		return -1;
	}
	if (debug && (verbose > 4)) {
		dprintk("%s reply is 0x%x\n", __FUNCTION__, ret[0]);
		for (err = 1; err < len; err++)
			dprintk(" 0x%x", ret[err]);
		if (err > 1)
			dprintk("\n");
	}

	return 0;
}
EXPORT_SYMBOL(read_dst);

static int dst_set_freq(struct dst_state *state, u32 freq)
{
	u8 *val;

	state->frequency = freq;
	if (debug > 4)
		dprintk("%s: set Frequency %u\n", __FUNCTION__, freq);

	if (state->dst_type == DST_TYPE_IS_SAT) {
		freq = freq / 1000;
		if (freq < 950 || freq > 2150)
			return -EINVAL;
		val = &state->tx_tuna[0];
		val[2] = (freq >> 8) & 0x7f;
		val[3] = (u8) freq;
		val[4] = 1;
		val[8] &= ~4;
		if (freq < 1531)
			val[8] |= 4;
	} else if (state->dst_type == DST_TYPE_IS_TERR) {
		freq = freq / 1000;
		if (freq < 137000 || freq > 858000)
			return -EINVAL;
		val = &state->tx_tuna[0];
		val[2] = (freq >> 16) & 0xff;
		val[3] = (freq >> 8) & 0xff;
		val[4] = (u8) freq;
		val[5] = 0;
		switch (state->bandwidth) {
		case BANDWIDTH_6_MHZ:
			val[6] = 6;
			break;

		case BANDWIDTH_7_MHZ:
		case BANDWIDTH_AUTO:
			val[6] = 7;
			break;

		case BANDWIDTH_8_MHZ:
			val[6] = 8;
			break;
		}

		val[7] = 0;
		val[8] = 0;
	} else if (state->dst_type == DST_TYPE_IS_CABLE) {
		/* guess till will get one */
		freq = freq / 1000;
		val = &state->tx_tuna[0];
		val[2] = (freq >> 16) & 0xff;
		val[3] = (freq >> 8) & 0xff;
		val[4] = (u8) freq;
	} else
		return -EINVAL;
	return 0;
}

static int dst_set_bandwidth(struct dst_state* state, fe_bandwidth_t bandwidth)
{
	u8 *val;

	state->bandwidth = bandwidth;

	if (state->dst_type != DST_TYPE_IS_TERR)
		return 0;

	val = &state->tx_tuna[0];
	switch (bandwidth) {
	case BANDWIDTH_6_MHZ:
		val[6] = 6;
		break;

	case BANDWIDTH_7_MHZ:
		val[6] = 7;
		break;

	case BANDWIDTH_8_MHZ:
		val[6] = 8;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int dst_set_inversion(struct dst_state* state, fe_spectral_inversion_t inversion)
{
	u8 *val;

	state->inversion = inversion;

	val = &state->tx_tuna[0];

	val[8] &= ~0x80;

	switch (inversion) {
	case INVERSION_OFF:
		break;
	case INVERSION_ON:
		val[8] |= 0x80;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int dst_set_fec(struct dst_state* state, fe_code_rate_t fec)
{
	state->fec = fec;
	return 0;
}

static fe_code_rate_t dst_get_fec(struct dst_state* state)
{
	return state->fec;
}

static int dst_set_symbolrate(struct dst_state* state, u32 srate)
{
	u8 *val;
	u32 symcalc;
	u64 sval;

	state->symbol_rate = srate;

	if (state->dst_type == DST_TYPE_IS_TERR) {
		return 0;
	}
	if (debug > 4)
		dprintk("%s: set symrate %u\n", __FUNCTION__, srate);
	srate /= 1000;
	val = &state->tx_tuna[0];

	if (state->type_flags & DST_TYPE_HAS_SYMDIV) {
		sval = srate;
		sval <<= 20;
		do_div(sval, 88000);
		symcalc = (u32) sval;

		if (debug > 4)
			dprintk("%s: set symcalc %u\n", __FUNCTION__, symcalc);

		val[5] = (u8) (symcalc >> 12);
		val[6] = (u8) (symcalc >> 4);
		val[7] = (u8) (symcalc << 4);
	} else {
		val[5] = (u8) (srate >> 16) & 0x7f;
		val[6] = (u8) (srate >> 8);
		val[7] = (u8) srate;
	}
	val[8] &= ~0x20;
	if (srate > 8000)
		val[8] |= 0x20;
	return 0;
}

u8 dst_check_sum(u8 * buf, u32 len)
{
	u32 i;
	u8 val = 0;
	if (!len)
		return 0;
	for (i = 0; i < len; i++) {
		val += buf[i];
	}
	return ((~val) + 1);
}
EXPORT_SYMBOL(dst_check_sum);

static void dst_type_flags_print(u32 type_flags)
{
	printk("DST type flags :");
	if (type_flags & DST_TYPE_HAS_NEWTUNE)
		printk(" 0x%x newtuner", DST_TYPE_HAS_NEWTUNE);
	if (type_flags & DST_TYPE_HAS_TS204)
		printk(" 0x%x ts204", DST_TYPE_HAS_TS204);
	if (type_flags & DST_TYPE_HAS_SYMDIV)
		printk(" 0x%x symdiv", DST_TYPE_HAS_SYMDIV);
	if (type_flags & DST_TYPE_HAS_FW_1)
		printk(" 0x%x firmware version = 1", DST_TYPE_HAS_FW_1);
	if (type_flags & DST_TYPE_HAS_FW_2)
		printk(" 0x%x firmware version = 2", DST_TYPE_HAS_FW_2);
	if (type_flags & DST_TYPE_HAS_FW_3)
		printk(" 0x%x firmware version = 3", DST_TYPE_HAS_FW_3);
//	if ((type_flags & DST_TYPE_HAS_FW_BUILD) && new_fw)

	printk("\n");
}


static int dst_type_print (u8 type)
{
	char *otype;
	switch (type) {
	case DST_TYPE_IS_SAT:
		otype = "satellite";
		break;

	case DST_TYPE_IS_TERR:
		otype = "terrestrial";
		break;

	case DST_TYPE_IS_CABLE:
		otype = "cable";
		break;

	default:
		printk("%s: invalid dst type %d\n", __FUNCTION__, type);
		return -EINVAL;
	}
	printk("DST type : %s\n", otype);

	return 0;
}

/*
	Known cards list
	Satellite
	-------------------
		  200103A
	VP-1020   DST-MOT	LG(old), TS=188

	VP-1020   DST-03T	LG(new), TS=204
	VP-1022   DST-03T	LG(new), TS=204
	VP-1025   DST-03T	LG(new), TS=204

	VP-1030   DSTMCI,	LG(new), TS=188
	VP-1032   DSTMCI,	LG(new), TS=188

	Cable
	-------------------
	VP-2030   DCT-CI,	Samsung, TS=204
	VP-2021   DCT-CI,	Unknown, TS=204
	VP-2031   DCT-CI,	Philips, TS=188
	VP-2040   DCT-CI,	Philips, TS=188, with CA daughter board
	VP-2040   DCT-CI,	Philips, TS=204, without CA daughter board

	Terrestrial
	-------------------
	VP-3050  DTTNXT			 TS=188
	VP-3040  DTT-CI,	Philips, TS=188
	VP-3040  DTT-CI,	Philips, TS=204

	ATSC
	-------------------
	VP-3220  ATSCDI,		 TS=188
	VP-3250  ATSCAD,		 TS=188

*/

struct dst_types dst_tlist[] = {
	{
		.device_id = "200103A",
		.offset = 0,
		.dst_type =  DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_SYMDIV | DST_TYPE_HAS_FW_1,
		.dst_feature = 0
	},	/*	obsolete	*/

	{
		.device_id = "DST-020",
		.offset = 0,
		.dst_type =  DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_SYMDIV | DST_TYPE_HAS_FW_1,
		.dst_feature = 0
	},	/*	obsolete	*/

	{
		.device_id = "DST-030",
		.offset =  0,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_TS204 | DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_1,
		.dst_feature = 0
	},	/*	obsolete	*/

	{
		.device_id = "DST-03T",
		.offset = 0,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_SYMDIV | DST_TYPE_HAS_TS204 | DST_TYPE_HAS_FW_2,
		.dst_feature = DST_TYPE_HAS_DISEQC3 | DST_TYPE_HAS_DISEQC4 | DST_TYPE_HAS_DISEQC5
							 | DST_TYPE_HAS_MAC | DST_TYPE_HAS_MOTO
	 },

	{
		.device_id = "DST-MOT",
		.offset =  0,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_SYMDIV | DST_TYPE_HAS_FW_1,
		.dst_feature = 0
	},	/*	obsolete	*/

	{
		.device_id = "DST-CI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_TS204 | DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_1,
		.dst_feature = DST_TYPE_HAS_CA
	},	/*	An OEM board	*/

	{
		.device_id = "DSTMCI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_2 | DST_TYPE_HAS_FW_BUILD,
		.dst_feature = DST_TYPE_HAS_CA | DST_TYPE_HAS_DISEQC3 | DST_TYPE_HAS_DISEQC4
							| DST_TYPE_HAS_MOTO | DST_TYPE_HAS_MAC
	},

	{
		.device_id = "DSTFCI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_SAT,
		.type_flags = DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_1,
		.dst_feature = 0
	},	/* unknown to vendor	*/

	{
		.device_id = "DCT-CI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_CABLE,
		.type_flags = DST_TYPE_HAS_TS204 | DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_1
							| DST_TYPE_HAS_FW_2 | DST_TYPE_HAS_FW_BUILD,
		.dst_feature = DST_TYPE_HAS_CA
	},

	{
		.device_id = "DCTNEW",
		.offset = 1,
		.dst_type = DST_TYPE_IS_CABLE,
		.type_flags = DST_TYPE_HAS_NEWTUNE | DST_TYPE_HAS_FW_3,
		.dst_feature = 0
	},

	{
		.device_id = "DTT-CI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_TERR,
		.type_flags = DST_TYPE_HAS_TS204 | DST_TYPE_HAS_FW_2 | DST_TYPE_HAS_FW_BUILD,
		.dst_feature = 0
	},

	{
		.device_id = "DTTDIG",
		.offset = 1,
		.dst_type = DST_TYPE_IS_TERR,
		.type_flags = DST_TYPE_HAS_FW_2,
		.dst_feature = 0
	},

	{
		.device_id = "DTTNXT",
		.offset = 1,
		.dst_type = DST_TYPE_IS_TERR,
		.type_flags = DST_TYPE_HAS_FW_2,
		.dst_feature = DST_TYPE_HAS_ANALOG
	},

	{
		.device_id = "ATSCDI",
		.offset = 1,
		.dst_type = DST_TYPE_IS_ATSC,
		.type_flags = DST_TYPE_HAS_FW_2,
		.dst_feature = 0
	},

	{
		.device_id = "ATSCAD",
		.offset = 1,
		.dst_type = DST_TYPE_IS_ATSC,
		.type_flags = DST_TYPE_HAS_FW_2,
		.dst_feature = 0
	},

	{ }

};


static int dst_get_device_id(struct dst_state *state)
{
	u8 reply;

	int i;
	struct dst_types *p_dst_type;
	u8 use_dst_type = 0;
	u32 use_type_flags = 0;

	static u8 device_type[8] = {0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};

	device_type[7] = dst_check_sum(device_type, 7);

	if (write_dst(state, device_type, FIXED_COMM))
		return -1;		/*	Write failed		*/

	if ((dst_pio_disable(state)) < 0)
		return -1;

	if (read_dst(state, &reply, GET_ACK))
		return -1;		/*	Read failure		*/

	if (reply != ACK) {
		dprintk("%s: Write not Acknowledged! [Reply=0x%02x]\n", __FUNCTION__, reply);
		return -1;		/*	Unack'd write		*/
	}

	if (!dst_wait_dst_ready(state, DEVICE_INIT))
		return -1;		/*	DST not ready yet	*/

	if (read_dst(state, state->rxbuffer, FIXED_COMM))
		return -1;

	dst_pio_disable(state);

	if (state->rxbuffer[7] != dst_check_sum(state->rxbuffer, 7)) {
		dprintk("%s: Checksum failure! \n", __FUNCTION__);
		return -1;		/*	Checksum failure	*/
	}

	state->rxbuffer[7] = '\0';

	for (i = 0, p_dst_type = dst_tlist; i < ARRAY_SIZE (dst_tlist); i++, p_dst_type++) {
		if (!strncmp (&state->rxbuffer[p_dst_type->offset], p_dst_type->device_id, strlen (p_dst_type->device_id))) {
			use_type_flags = p_dst_type->type_flags;
			use_dst_type = p_dst_type->dst_type;

			/*	Card capabilities	*/
			state->dst_hw_cap = p_dst_type->dst_feature;
			printk ("%s: Recognise [%s]\n", __FUNCTION__, p_dst_type->device_id);

			break;
		}
	}

	if (i >= sizeof (dst_tlist) / sizeof (dst_tlist [0])) {
		printk("%s: Unable to recognize %s or %s\n", __FUNCTION__, &state->rxbuffer[0], &state->rxbuffer[1]);
		printk("%s: please email linux-dvb@linuxtv.org with this type in\n", __FUNCTION__);
		use_dst_type = DST_TYPE_IS_SAT;
		use_type_flags = DST_TYPE_HAS_SYMDIV;
	}

	dst_type_print(use_dst_type);
	state->type_flags = use_type_flags;
	state->dst_type = use_dst_type;
	dst_type_flags_print(state->type_flags);

	if (state->type_flags & DST_TYPE_HAS_TS204) {
		dst_packsize(state, 204);
	}

	return 0;
}

static int dst_probe(struct dst_state *state)
{
	if ((rdc_8820_reset(state)) < 0) {
		dprintk("%s: RDC 8820 RESET Failed.\n", __FUNCTION__);
		return -1;
	}
	if (dst_addons & DST_TYPE_HAS_CA)
		msleep(4000);
	else
		msleep(100);

	if ((dst_comm_init(state)) < 0) {
		dprintk("%s: DST Initialization Failed.\n", __FUNCTION__);
		return -1;
	}
	msleep(100);
	if (dst_get_device_id(state) < 0) {
		dprintk("%s: unknown device.\n", __FUNCTION__);
		return -1;
	}

	return 0;
}

int dst_command(struct dst_state* state, u8 * data, u8 len)
{
	u8 reply;
	if ((dst_comm_init(state)) < 0) {
		dprintk("%s: DST Communication Initialization Failed.\n", __FUNCTION__);
		return -1;
	}

	if (write_dst(state, data, len)) {
		if (verbose > 1)
			dprintk("%s: Tring to recover.. \n", __FUNCTION__);
		if ((dst_error_recovery(state)) < 0) {
			dprintk("%s: Recovery Failed.\n", __FUNCTION__);
			return -1;
		}
		return -1;
	}
	if ((dst_pio_disable(state)) < 0) {
		dprintk("%s: PIO Disable Failed.\n", __FUNCTION__);
		return -1;
	}
	if (state->type_flags & DST_TYPE_HAS_FW_1)
		udelay(3000);

	if (read_dst(state, &reply, GET_ACK)) {
		if (verbose > 1)
			dprintk("%s: Trying to recover.. \n", __FUNCTION__);
		if ((dst_error_recovery(state)) < 0) {
			dprintk("%s: Recovery Failed.\n", __FUNCTION__);
			return -1;
		}
		return -1;
	}

	if (reply != ACK) {
		dprintk("%s: write not acknowledged 0x%02x \n", __FUNCTION__, reply);
		return -1;
	}
	if (len >= 2 && data[0] == 0 && (data[1] == 1 || data[1] == 3))
		return 0;

//	udelay(3000);
	if (state->type_flags & DST_TYPE_HAS_FW_1)
		udelay(3000);
	else
		udelay(2000);

	if (!dst_wait_dst_ready(state, NO_DELAY))
		return -1;

	if (read_dst(state, state->rxbuffer, FIXED_COMM)) {
		if (verbose > 1)
			dprintk("%s: Trying to recover.. \n", __FUNCTION__);
		if ((dst_error_recovery(state)) < 0) {
			dprintk("%s: Recovery failed.\n", __FUNCTION__);
			return -1;
		}
		return -1;
	}

	if (state->rxbuffer[7] != dst_check_sum(state->rxbuffer, 7)) {
		dprintk("%s: checksum failure\n", __FUNCTION__);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(dst_command);

static int dst_get_signal(struct dst_state* state)
{
	int retval;
	u8 get_signal[] = { 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb };

	if ((state->diseq_flags & ATTEMPT_TUNE) == 0) {
		state->decode_lock = state->decode_strength = state->decode_snr = 0;
		return 0;
	}
	if (0 == (state->diseq_flags & HAS_LOCK)) {
		state->decode_lock = state->decode_strength = state->decode_snr = 0;
		return 0;
	}
	if (time_after_eq(jiffies, state->cur_jiff + (HZ / 5))) {
		retval = dst_command(state, get_signal, 8);
		if (retval < 0)
			return retval;
		if (state->dst_type == DST_TYPE_IS_SAT) {
			state->decode_lock = ((state->rxbuffer[6] & 0x10) == 0) ? 1 : 0;
			state->decode_strength = state->rxbuffer[5] << 8;
			state->decode_snr = state->rxbuffer[2] << 8 | state->rxbuffer[3];
		} else if ((state->dst_type == DST_TYPE_IS_TERR) || (state->dst_type == DST_TYPE_IS_CABLE)) {
			state->decode_lock = (state->rxbuffer[1]) ? 1 : 0;
			state->decode_strength = state->rxbuffer[4] << 8;
			state->decode_snr = state->rxbuffer[3] << 8;
		}
		state->cur_jiff = jiffies;
	}
	return 0;
}

static int dst_tone_power_cmd(struct dst_state* state)
{
	u8 paket[8] = { 0x00, 0x09, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00 };

	if (state->dst_type == DST_TYPE_IS_TERR)
		return 0;

	paket[4] = state->tx_tuna[4];
	paket[2] = state->tx_tuna[2];
	paket[3] = state->tx_tuna[3];
	paket[7] = dst_check_sum (paket, 7);
	dst_command(state, paket, 8);

	return 0;
}

static int dst_get_tuna(struct dst_state* state)
{
	int retval;

	if ((state->diseq_flags & ATTEMPT_TUNE) == 0)
		return 0;

	state->diseq_flags &= ~(HAS_LOCK);
	if (!dst_wait_dst_ready(state, NO_DELAY))
		return 0;

	if (state->type_flags & DST_TYPE_HAS_NEWTUNE) {
		/* how to get variable length reply ???? */
		retval = read_dst(state, state->rx_tuna, 10);
	} else {
		retval = read_dst(state, &state->rx_tuna[2], FIXED_COMM);
	}

	if (retval < 0) {
		dprintk("%s: read not successful\n", __FUNCTION__);
		return 0;
	}

	if (state->type_flags & DST_TYPE_HAS_NEWTUNE) {
		if (state->rx_tuna[9] != dst_check_sum(&state->rx_tuna[0], 9)) {
			dprintk("%s: checksum failure?\n", __FUNCTION__);
			return 0;
		}
	} else {
		if (state->rx_tuna[9] != dst_check_sum(&state->rx_tuna[2], 7)) {
			dprintk("%s: checksum failure?\n", __FUNCTION__);
			return 0;
		}
	}
	if (state->rx_tuna[2] == 0 && state->rx_tuna[3] == 0)
		return 0;
	state->decode_freq = ((state->rx_tuna[2] & 0x7f) << 8) + state->rx_tuna[3];

	state->decode_lock = 1;
	/*
	   dst->decode_n1 = (dst->rx_tuna[4] << 8) +
	   (dst->rx_tuna[5]);

	   dst->decode_n2 = (dst->rx_tuna[8] << 8) +
	   (dst->rx_tuna[7]);
	 */
	state->diseq_flags |= HAS_LOCK;
	/* dst->cur_jiff = jiffies; */
	return 1;
}

static int dst_set_voltage(struct dvb_frontend* fe, fe_sec_voltage_t voltage);

static int dst_write_tuna(struct dvb_frontend* fe)
{
	struct dst_state* state = fe->demodulator_priv;
	int retval;
	u8 reply;

	if (debug > 4)
		dprintk("%s: type_flags 0x%x \n", __FUNCTION__, state->type_flags);

	state->decode_freq = 0;
	state->decode_lock = state->decode_strength = state->decode_snr = 0;
	if (state->dst_type == DST_TYPE_IS_SAT) {
		if (!(state->diseq_flags & HAS_POWER))
			dst_set_voltage(fe, SEC_VOLTAGE_13);
	}
	state->diseq_flags &= ~(HAS_LOCK | ATTEMPT_TUNE);

	if ((dst_comm_init(state)) < 0) {
		dprintk("%s: DST Communication initialization failed.\n", __FUNCTION__);
		return -1;
	}

	if (state->type_flags & DST_TYPE_HAS_NEWTUNE) {
		state->tx_tuna[9] = dst_check_sum(&state->tx_tuna[0], 9);
		retval = write_dst(state, &state->tx_tuna[0], 10);

	} else {
		state->tx_tuna[9] = dst_check_sum(&state->tx_tuna[2], 7);
		retval = write_dst(state, &state->tx_tuna[2], FIXED_COMM);
	}
	if (retval < 0) {
		dst_pio_disable(state);
		dprintk("%s: write not successful\n", __FUNCTION__);
		return retval;
	}

	if ((dst_pio_disable(state)) < 0) {
		dprintk("%s: DST PIO disable failed !\n", __FUNCTION__);
		return -1;
	}

	if ((read_dst(state, &reply, GET_ACK) < 0)) {
		dprintk("%s: read verify not successful.\n", __FUNCTION__);
		return -1;
	}
	if (reply != ACK) {
		dprintk("%s: write not acknowledged 0x%02x \n", __FUNCTION__, reply);
		return 0;
	}
	state->diseq_flags |= ATTEMPT_TUNE;

	return dst_get_tuna(state);
}

/*
 * line22k0    0x00, 0x09, 0x00, 0xff, 0x01, 0x00, 0x00, 0x00
 * line22k1    0x00, 0x09, 0x01, 0xff, 0x01, 0x00, 0x00, 0x00
 * line22k2    0x00, 0x09, 0x02, 0xff, 0x01, 0x00, 0x00, 0x00
 * tone        0x00, 0x09, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00
 * data        0x00, 0x09, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00
 * power_off   0x00, 0x09, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00
 * power_on    0x00, 0x09, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00
 * Diseqc 1    0x00, 0x08, 0x04, 0xe0, 0x10, 0x38, 0xf0, 0xec
 * Diseqc 2    0x00, 0x08, 0x04, 0xe0, 0x10, 0x38, 0xf4, 0xe8
 * Diseqc 3    0x00, 0x08, 0x04, 0xe0, 0x10, 0x38, 0xf8, 0xe4
 * Diseqc 4    0x00, 0x08, 0x04, 0xe0, 0x10, 0x38, 0xfc, 0xe0
 */

static int dst_set_diseqc(struct dvb_frontend* fe, struct dvb_diseqc_master_cmd* cmd)
{
	struct dst_state* state = fe->demodulator_priv;
	u8 paket[8] = { 0x00, 0x08, 0x04, 0xe0, 0x10, 0x38, 0xf0, 0xec };

	if (state->dst_type != DST_TYPE_IS_SAT)
		return 0;

	if (cmd->msg_len == 0 || cmd->msg_len > 4)
		return -EINVAL;
	memcpy(&paket[3], cmd->msg, cmd->msg_len);
	paket[7] = dst_check_sum(&paket[0], 7);
	dst_command(state, paket, 8);
	return 0;
}

static int dst_set_voltage(struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{
	int need_cmd;
	struct dst_state* state = fe->demodulator_priv;

	state->voltage = voltage;

	if (state->dst_type != DST_TYPE_IS_SAT)
		return 0;

	need_cmd = 0;
	switch (voltage) {
		case SEC_VOLTAGE_13:
		case SEC_VOLTAGE_18:
			if ((state->diseq_flags & HAS_POWER) == 0)
				need_cmd = 1;
			state->diseq_flags |= HAS_POWER;
			state->tx_tuna[4] = 0x01;
			break;

		case SEC_VOLTAGE_OFF:
			need_cmd = 1;
			state->diseq_flags &= ~(HAS_POWER | HAS_LOCK | ATTEMPT_TUNE);
			state->tx_tuna[4] = 0x00;
			break;

		default:
			return -EINVAL;
	}
	if (need_cmd)
		dst_tone_power_cmd(state);

	return 0;
}

static int dst_set_tone(struct dvb_frontend* fe, fe_sec_tone_mode_t tone)
{
	struct dst_state* state = fe->demodulator_priv;

	state->tone = tone;

	if (state->dst_type != DST_TYPE_IS_SAT)
		return 0;

	switch (tone) {
		case SEC_TONE_OFF:
			state->tx_tuna[2] = 0xff;
			break;

		case SEC_TONE_ON:
			state->tx_tuna[2] = 0x02;
			break;

		default:
			return -EINVAL;
	}
	dst_tone_power_cmd(state);

	return 0;
}

static int dst_send_burst(struct dvb_frontend *fe, fe_sec_mini_cmd_t minicmd)
{
	struct dst_state *state = fe->demodulator_priv;

	if (state->dst_type != DST_TYPE_IS_SAT)
		return 0;

	state->minicmd = minicmd;

	switch (minicmd) {
		case SEC_MINI_A:
			state->tx_tuna[3] = 0x02;
			break;
		case SEC_MINI_B:
			state->tx_tuna[3] = 0xff;
			break;
	}
	dst_tone_power_cmd(state);

	return 0;
}


static int dst_init(struct dvb_frontend* fe)
{
	struct dst_state* state = fe->demodulator_priv;
	static u8 ini_satci_tuna[] = { 9, 0, 3, 0xb6, 1, 0, 0x73, 0x21, 0, 0 };
	static u8 ini_satfta_tuna[] = { 0, 0, 3, 0xb6, 1, 0x55, 0xbd, 0x50, 0, 0 };
	static u8 ini_tvfta_tuna[] = { 0, 0, 3, 0xb6, 1, 7, 0x0, 0x0, 0, 0 };
	static u8 ini_tvci_tuna[] = { 9, 0, 3, 0xb6, 1, 7, 0x0, 0x0, 0, 0 };
	static u8 ini_cabfta_tuna[] = { 0, 0, 3, 0xb6, 1, 7, 0x0, 0x0, 0, 0 };
	static u8 ini_cabci_tuna[] = { 9, 0, 3, 0xb6, 1, 7, 0x0, 0x0, 0, 0 };
	state->inversion = INVERSION_ON;
	state->voltage = SEC_VOLTAGE_13;
	state->tone = SEC_TONE_OFF;
	state->symbol_rate = 29473000;
	state->fec = FEC_AUTO;
	state->diseq_flags = 0;
	state->k22 = 0x02;
	state->bandwidth = BANDWIDTH_7_MHZ;
	state->cur_jiff = jiffies;
	if (state->dst_type == DST_TYPE_IS_SAT) {
		state->frequency = 950000;
		memcpy(state->tx_tuna, ((state->type_flags & DST_TYPE_HAS_NEWTUNE) ? ini_satci_tuna : ini_satfta_tuna), sizeof(ini_satfta_tuna));
	} else if (state->dst_type == DST_TYPE_IS_TERR) {
		state->frequency = 137000000;
		memcpy(state->tx_tuna, ((state->type_flags & DST_TYPE_HAS_NEWTUNE) ? ini_tvci_tuna : ini_tvfta_tuna), sizeof(ini_tvfta_tuna));
	} else if (state->dst_type == DST_TYPE_IS_CABLE) {
		state->frequency = 51000000;
		memcpy(state->tx_tuna, ((state->type_flags & DST_TYPE_HAS_NEWTUNE) ? ini_cabci_tuna : ini_cabfta_tuna), sizeof(ini_cabfta_tuna));
	}

	return 0;
}

static int dst_read_status(struct dvb_frontend* fe, fe_status_t* status)
{
	struct dst_state* state = fe->demodulator_priv;

	*status = 0;
	if (state->diseq_flags & HAS_LOCK) {
		dst_get_signal(state);
		if (state->decode_lock)
			*status |= FE_HAS_LOCK | FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_SYNC | FE_HAS_VITERBI;
	}

	return 0;
}

static int dst_read_signal_strength(struct dvb_frontend* fe, u16* strength)
{
	struct dst_state* state = fe->demodulator_priv;

	dst_get_signal(state);
	*strength = state->decode_strength;

	return 0;
}

static int dst_read_snr(struct dvb_frontend* fe, u16* snr)
{
	struct dst_state* state = fe->demodulator_priv;

	dst_get_signal(state);
	*snr = state->decode_snr;

	return 0;
}

static int dst_set_frontend(struct dvb_frontend* fe, struct dvb_frontend_parameters *p)
{
	struct dst_state* state = fe->demodulator_priv;

	dst_set_freq(state, p->frequency);
	if (verbose > 4)
		dprintk("Set Frequency = [%d]\n", p->frequency);

	dst_set_inversion(state, p->inversion);
	if (state->dst_type == DST_TYPE_IS_SAT) {
		dst_set_fec(state, p->u.qpsk.fec_inner);
		dst_set_symbolrate(state, p->u.qpsk.symbol_rate);
		if (verbose > 4)
			dprintk("Set Symbolrate = [%d]\n", p->u.qpsk.symbol_rate);

	} else if (state->dst_type == DST_TYPE_IS_TERR) {
		dst_set_bandwidth(state, p->u.ofdm.bandwidth);
	} else if (state->dst_type == DST_TYPE_IS_CABLE) {
		dst_set_fec(state, p->u.qam.fec_inner);
		dst_set_symbolrate(state, p->u.qam.symbol_rate);
	}
	dst_write_tuna(fe);

	return 0;
}

static int dst_get_frontend(struct dvb_frontend* fe, struct dvb_frontend_parameters *p)
{
	struct dst_state* state = fe->demodulator_priv;

	p->frequency = state->decode_freq;
	p->inversion = state->inversion;
	if (state->dst_type == DST_TYPE_IS_SAT) {
		p->u.qpsk.symbol_rate = state->symbol_rate;
		p->u.qpsk.fec_inner = dst_get_fec(state);
	} else if (state->dst_type == DST_TYPE_IS_TERR) {
		p->u.ofdm.bandwidth = state->bandwidth;
	} else if (state->dst_type == DST_TYPE_IS_CABLE) {
		p->u.qam.symbol_rate = state->symbol_rate;
		p->u.qam.fec_inner = dst_get_fec(state);
		p->u.qam.modulation = QAM_AUTO;
	}

	return 0;
}

static void dst_release(struct dvb_frontend* fe)
{
	struct dst_state* state = fe->demodulator_priv;
	kfree(state);
}

static struct dvb_frontend_ops dst_dvbt_ops;
static struct dvb_frontend_ops dst_dvbs_ops;
static struct dvb_frontend_ops dst_dvbc_ops;

struct dst_state* dst_attach(struct dst_state *state, struct dvb_adapter *dvb_adapter)
{

	/* check if the ASIC is there */
	if (dst_probe(state) < 0) {
		if (state)
			kfree(state);

		return NULL;
	}
	/* determine settings based on type */
	switch (state->dst_type) {
	case DST_TYPE_IS_TERR:
		memcpy(&state->ops, &dst_dvbt_ops, sizeof(struct dvb_frontend_ops));
		break;

	case DST_TYPE_IS_CABLE:
		memcpy(&state->ops, &dst_dvbc_ops, sizeof(struct dvb_frontend_ops));
		break;

	case DST_TYPE_IS_SAT:
		memcpy(&state->ops, &dst_dvbs_ops, sizeof(struct dvb_frontend_ops));
		break;

	default:
		printk("%s: unknown DST type. please report to the LinuxTV.org DVB mailinglist.\n", __FUNCTION__);
		if (state)
			kfree(state);

		return NULL;
	}

	/* create dvb_frontend */
	state->frontend.ops = &state->ops;
	state->frontend.demodulator_priv = state;

	return state;				/*	Manu (DST is a card not a frontend)	*/
}

EXPORT_SYMBOL(dst_attach);

static struct dvb_frontend_ops dst_dvbt_ops = {

	.info = {
		.name = "DST DVB-T",
		.type = FE_OFDM,
		.frequency_min = 137000000,
		.frequency_max = 858000000,
		.frequency_stepsize = 166667,
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO | FE_CAN_TRANSMISSION_MODE_AUTO | FE_CAN_GUARD_INTERVAL_AUTO
	},

	.release = dst_release,

	.init = dst_init,

	.set_frontend = dst_set_frontend,
	.get_frontend = dst_get_frontend,

	.read_status = dst_read_status,
	.read_signal_strength = dst_read_signal_strength,
	.read_snr = dst_read_snr,
};

static struct dvb_frontend_ops dst_dvbs_ops = {

	.info = {
		.name = "DST DVB-S",
		.type = FE_QPSK,
		.frequency_min = 950000,
		.frequency_max = 2150000,
		.frequency_stepsize = 1000,	/* kHz for QPSK frontends */
		.frequency_tolerance = 29500,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
	/*     . symbol_rate_tolerance	=	???,*/
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QPSK
	},

	.release = dst_release,

	.init = dst_init,

	.set_frontend = dst_set_frontend,
	.get_frontend = dst_get_frontend,

	.read_status = dst_read_status,
	.read_signal_strength = dst_read_signal_strength,
	.read_snr = dst_read_snr,

	.diseqc_send_burst = dst_send_burst,
	.diseqc_send_master_cmd = dst_set_diseqc,
	.set_voltage = dst_set_voltage,
	.set_tone = dst_set_tone,
};

static struct dvb_frontend_ops dst_dvbc_ops = {

	.info = {
		.name = "DST DVB-C",
		.type = FE_QAM,
		.frequency_stepsize = 62500,
		.frequency_min = 51000000,
		.frequency_max = 858000000,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
	/*     . symbol_rate_tolerance	=	???,*/
		.caps = FE_CAN_FEC_AUTO | FE_CAN_QAM_AUTO
	},

	.release = dst_release,

	.init = dst_init,

	.set_frontend = dst_set_frontend,
	.get_frontend = dst_get_frontend,

	.read_status = dst_read_status,
	.read_signal_strength = dst_read_signal_strength,
	.read_snr = dst_read_snr,
};


MODULE_DESCRIPTION("DST DVB-S/T/C Combo Frontend driver");
MODULE_AUTHOR("Jamie Honan, Manu Abraham");
MODULE_LICENSE("GPL");
