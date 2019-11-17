/****************************************************************************
** hid_irmp.c ***************************************************************
*****************************************************************************
*
* Lirc driver for USB IR Remote Receiver based on IRMP:
* https://www.mikrocontroller.net/articles/USB_IR_Remote_Receiver
*
* Copyright (C) 2019 Fabian Lesniak <fabian@lesniak-it.de>
*
* Distribute under GPL version 2 or later.
*
*/

#include <poll.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <signal.h>
#include <errno.h>

#include "lirc_driver.h"

static int hid_irmp_init(void);
static int hid_irmp_deinit(void);
static char* hid_irmp_rec(struct ir_remote* remotes);
static int hid_irmp_decode(struct ir_remote* remote, struct decode_ctx_t* ctx);

static const logchannel_t logchannel = LOG_DRIVER;

enum hid_irmp_type {
	NewIRCodeAvailable = 1
};

static struct hid_irmp_msg {
	uint8_t type;
	uint8_t protocol;
	uint16_t address;
	uint16_t command;
	uint8_t flags;
} msg;


int check_usbid(uint16_t vendor,  uint16_t product)
{
	return vendor == 0x16c0 && product == 0x5df;
}

static int drvctl_func(unsigned int cmd, void* arg)
{
	switch (cmd) {
	case DRVCTL_GET_DEVICES:
		// return drv_enum_glob((glob_t*) arg, "/dev/hidraw*");
		return drv_enum_usb((glob_t*) arg, check_usbid);

	case DRVCTL_FREE_DEVICES:
		drv_enum_free((glob_t*) arg);
		return 0;
	default:
		return DRV_ERR_NOT_IMPLEMENTED;
	}
}

const struct driver hw_hid_irmp = {
	.name		= "hid_irmp",
	.device		= "auto",
	.fd		= -1,
	.features	= LIRC_CAN_REC_LIRCCODE,
	.send_mode	= 0,
	.rec_mode	= LIRC_MODE_LIRCCODE,
	.code_length	= 56,
	.init_func	= hid_irmp_init,
	.deinit_func	= hid_irmp_deinit,
	.open_func	= default_open,
	.close_func	= default_close,
	.send_func	= NULL,
	.rec_func	= hid_irmp_rec,
	.decode_func	= hid_irmp_decode,
	.drvctl_func	= drvctl_func,
	.readdata	= NULL,
	.api_version	= 3,
	.driver_version = "0.0.1",
	.info		= "https://www.mikrocontroller.net/articles/USB_IR_Remote_Receiver",
	.device_hint    = "drvctl"
};

const struct driver* hardwares[] = { &hw_hid_irmp, (const struct driver*)NULL };

static int get_auto_device(char* pathbuf, size_t size)
{
	glob_t glob;

	int r = drvctl_func(DRVCTL_GET_DEVICES, &glob);
	if (r != 0)
		return r;

	if (glob.gl_pathc < 1) {
		log_error("No matching IRMP device found for \"auto\"");
		return ENODEV;
	}
	if (glob.gl_pathc > 1)
		log_warn("Multiple IRMP devices found for \"auto\"");

	strncpy(pathbuf, glob.gl_pathv[0], size - 1);
	globfree(&glob);
	return 0;
}

static int hid_irmp_init(void)
{
	char pathbuf[128];
	int r;

	strncpy(pathbuf, drv.device, sizeof(pathbuf) - 1);
	if (strcmp(pathbuf, "auto") == 0) {
		r = get_auto_device(pathbuf, sizeof(pathbuf));
		if (r != 0)
			return r;
	}

	log_info("initializing '%s'", pathbuf);

	drv.fd = open(pathbuf, O_RDONLY);
	if (drv.fd < 0) {
		log_error("unable to open '%s'", pathbuf);
		return 0;
	}

	return 1;
}

static int hid_irmp_deinit(void)
{
	if (drv.fd != -1) {
		log_info("closing '%s'", drv.device);
		close(drv.fd);
		drv.fd = -1;
	}
	return 1;
}

static char* hid_irmp_rec(struct ir_remote* remotes)
{
	if (read(drv.fd, &msg, sizeof(msg)) == -1) {
		log_error("(%s) could not read remote: %d", __func__, errno);
		hid_irmp_deinit();
		return 0;
	}

	// ignore everything except new IR codes
	if (msg.type != NewIRCodeAvailable) {
		return 0;
	}

	log_trace("hid_irmp: %02x %04x %04x %d",
		msg.protocol, msg.address, msg.command, msg.flags);

	return decode_all(remotes);
}

static int hid_irmp_decode(struct ir_remote* remote, struct decode_ctx_t* ctx)
{
	ir_code code = msg.command;
	code |= msg.address << 16;
	code |= (ir_code)msg.protocol << 32;

	log_trace("%s: map code 0x%010lx", __func__, code);

	if (!map_code(remote, ctx, 0, 0, 40, code, 0, 0))
		return 0;

	return 1;
}
