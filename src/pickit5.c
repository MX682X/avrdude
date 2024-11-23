/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2024 MX682X
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is not affiliated in any way with Microchip®
 */

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "pickit5.h"
#include "pickit5_lut.h"
#include "updi_constants.h"
#include "usbdevs.h"


#if defined(HAVE_USB_H)
#include <usb.h>                // Linux/Mac
#elif defined(HAVE_LUSB0_USB_H)
#include <lusb0_usb.h>          // Windows
#endif

// A USB driver is needed to talk with MacOS USB; it's unclear where to find it
// so remove the support under MacOS for the time being

#if defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)
#define USB_PK5_CMD_READ_EP   0x81
#define USB_PK5_CMD_WRITE_EP  0x02
#define USB_PK5_DATA_READ_EP  0x83
#define USB_PK5_DATA_WRITE_EP 0x04

#define USB_PK5_MAX_XFER    2048

#define CHECK_ERROR        0x01
#define BIST_TEST          0x02
#define BIST_RESULT        0x03

#define PGM_TYPE_PK5       0x00 // Default
#define PGM_TYPE_PK4       0x01 // PICkit4
#define PGM_TYPE_SNAP      0x02 // SNAP

#define PK_OP_NONE         0x00 // Init
#define PK_OP_FOUND        0x01 // PK is connected to USB
#define PK_OP_RESPONDS     0x02 // Responds to get_fw() requests
#define PK_OP_READY        0x03 // Voltage Set, Clock Set

#define POWER_SOURCE_EXT   0x00
#define POWER_SOURCE_INT   0x01
#define POWER_SOURCE_NONE  0x02

#define ERROR_USB_SEND            -10  // Start at 10 to avoid collisions
#define ERROR_USB_RECV            -11
#define ERROR_SCRIPT_PARAM_SIZE   -12
#define ERROR_BAD_RESPONSE        -13

// Private data for this programmer
struct pdata {
  unsigned char pgm_type;       // Used to skip unsupported functions
  unsigned char pk_op_mode;     // See PK_OP_ defines
  unsigned char power_source;   // 0: external / 1: from PICkit / 2: ignore check
  unsigned char hvupdi_enabled; // 0: no HV / 1: HV generation enabled
  double target_voltage;        // Voltage to supply to target

  double measured_vcc;          // This and below for print_params()
  unsigned int measured_current;
  unsigned int actual_pgm_clk;

  unsigned char nvm_version;    // Used to determine the offset for SIGROW/DevID

  unsigned char dW_switched_isp;

  unsigned char devID[4];       // Last byte has the Chip Revision of the target
  unsigned char app_version[3]; // Buffer for display() sent by get_fw()
  unsigned char fw_info[16];    // Buffer for display() sent by get_fw()
  unsigned char sernum_string[20]; // Buffer for display() sent by get_fw()
  char sib_string[32];
  unsigned char txBuf[2048];     // Buffer for transfers
  unsigned char rxBuf[2048];
  SCRIPT scripts;
};

#define my (*(struct pdata *)(pgm->cookie))


static void pickit5_setup(PROGRAMMER *pgm);
static void pickit5_teardown(PROGRAMMER *pgm);
static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms);
static int pickit5_open(PROGRAMMER *pgm, const char *port);
static void pickit5_close(PROGRAMMER *pgm);

static void pickit5_disable(const PROGRAMMER *pgm);
static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p);
static void pickit5_display(const PROGRAMMER *pgm, const char *p);
static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p);

static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);
static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes);

static int pickit5_write_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char value);
static int pickit5_read_byte(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned long addr, unsigned char *value);
static int pickit5_read_dev_id(const PROGRAMMER *pgm, const AVRPART *p);
static int pickit5_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev);
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

// UPDI-specific functions
static int pickit5_updi_init(const PROGRAMMER *pgm, const AVRPART *p, double v_target);
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value);
static int pickit5_updi_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value);
static int pickit5_updi_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib);
static int pickit5_updi_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value);
static int pickit5_updi_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char *value);

// ISP-specific
static int pickit5_isp_write_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char value);
static int pickit5_isp_read_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char *value);

// debugWire-specific
static int pickit5_dw_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value);
static int pickit5_dw_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value);
static void pickit5_dw_switch_to_isp(const PROGRAMMER *pgm, const AVRPART *p);
static void pickit5_isp_switch_to_dw(const PROGRAMMER *pgm, const AVRPART *p);

// TPI-specific
static int pickit5_tpi_read(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);
static int pickit5_tpi_write(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value);

// JTAG-Specific
static int pickit5_jtag_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value);
static int pickit5_jtag_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value);


// Extra functions
static int pickit5_get_fw_info(const PROGRAMMER *pgm);
static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v);
static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v);
static int pickit5_set_ptg_mode(const PROGRAMMER *pgm);
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod);


// Internal functions
inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num);
inline static unsigned int pickit5_array_to_uint32(unsigned char *buf);
inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type,
  unsigned int msg_len, unsigned int trans_len);
inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len, unsigned int script_len);
inline static int pickit5_check_ret_status(const PROGRAMMER *pgm);

static int pickit5_get_status(const PROGRAMMER *pgm, unsigned char status);
static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param, unsigned int param_len, unsigned int payload_len);
static int pickit5_send_script_done(const PROGRAMMER *pgm);
static int pickit5_read_response(const PROGRAMMER *pgm);
static int pickit5_send_script_cmd(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len);
static int pickit5_upload_data(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *recv_buf, unsigned int recv_len);
static int pickit5_download_data(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *send_buf, unsigned int send_len);


// Extra-USB related functions, because we need more then 2 endpoints
static int usbdev_data_recv(const union filedescriptor *fd, unsigned char *buf, size_t nbytes);
static int usbdev_data_send(const union filedescriptor *fd, const unsigned char *bp, size_t mlen);

inline static void pickit5_uint32_to_array(unsigned char *buf, uint32_t num) {
  *(buf++) = (uint8_t) num;
  *(buf++) = (uint8_t) (num >> 8);
  *(buf++) = (uint8_t) (num >> 16);
  *(buf) = (uint8_t) (num >> 24);
}

inline static unsigned int pickit5_array_to_uint32(unsigned char *buf) {
  unsigned int retval = 0;

  retval |= *(buf++);
  retval |= *(buf++) << 8;
  retval |= *(buf++) << 16;
  retval |= *(buf++) << 24;
  return retval;
}


inline static void pickit5_create_payload_header(unsigned char *buf, unsigned int type,
  unsigned int msg_len, unsigned int trans_len) {
  pickit5_uint32_to_array(&buf[0], type);
  pickit5_uint32_to_array(&buf[4], 0);
  pickit5_uint32_to_array(&buf[8], msg_len);
  pickit5_uint32_to_array(&buf[12], trans_len);
}

inline static void pickit5_create_script_header(unsigned char *buf, unsigned int arg_len, unsigned int script_len) {
  pickit5_uint32_to_array(&buf[0], arg_len);
  pickit5_uint32_to_array(&buf[4], script_len);
}

static void pickit5_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void pickit5_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

static int pickit5_parseextparms(const PROGRAMMER *pgm, const LISTID extparms) {
  LNODEID ln;
  const char *extended_param;
  int rv = 0;

  for(ln = lfirst(extparms); ln; ln = lnext(ln)) {
    extended_param = ldata(ln);

    if(str_starts(extended_param, "vtarg=")) {
      double voltage = -1.0;

      if(sscanf(extended_param, "vtarg=%lf", &voltage) != 1) {

        pmsg_error("invalid voltage parameter %s\n", extended_param);
        rv = -1;
        continue;
      }
      if(voltage < 0.1 && voltage > -1.0) {
        my.power_source = POWER_SOURCE_NONE;   // Voltage check disabled
        continue;
      } else if(voltage < 1.8 || voltage > 5.5) {
        pmsg_error("voltage %1.1lf V outside valid range [1.8 V, 5.5 V]\n", voltage);
        rv = -1;
        continue;
      } else {
        my.power_source = POWER_SOURCE_INT;    // PK supplies power
        my.target_voltage = voltage;
        continue;
      }
    }
    if(str_starts(extended_param, "hvupdi")) {
      my.hvupdi_enabled = 1;
      continue;
    }

    if(str_eq(extended_param, "help")) {
      msg_error("%s -c %s extended options:\n", progname, pgmid);
      msg_error("  -x vtarg=<dbl>  Enable power output; <dbl> must be in [1.8, 5.5] V\n");
      msg_error("  -x hvupdi       Enable high-voltage UPDI initialization\n");
      msg_error("  -x help         Show this help menu and exit\n");
      return LIBAVRDUDE_EXIT;
    }

    pmsg_error("invalid extended parameter %s\n", extended_param);
    rv = -1;
  }
  return rv;
}

// Internal functions

// Type can be CMD, UPLOAD or DOWNLOAD
#define SCR_CMD           0x0100
#define SCR_UPLOAD    0x80000102
#define SCR_DOWNLOAD 0x0C0000101

static int pickit5_send_script(const PROGRAMMER *pgm, unsigned int script_type,
  const unsigned char *script, unsigned int script_len,
  const unsigned char *param, unsigned int param_len, unsigned int payload_len) {

  if(script == NULL) {
    pmsg_error("Invalid script pointer passed!");
    return -3;
  }

  unsigned int header_len = 16 + 8;     // Header info + script header
  unsigned int preamble_len = header_len + param_len;
  unsigned int message_len = preamble_len + script_len;
  pmsg_debug("%s(scr_len: %u, param_len: %u, payload_len: %u)\n", __func__, script_len, param_len, payload_len);

  if(message_len >= 2048){       // Required memory will exceed buffer size, abort
    pmsg_error("Requested message size (%u) too large!", message_len);
    return ERROR_SCRIPT_PARAM_SIZE;     // 2 kB should be enough for everything
  }
  unsigned char *buf = my.txBuf;

  pickit5_create_payload_header(&buf[0], script_type, message_len, payload_len);
  pickit5_create_script_header(&buf[16], param_len, script_len);


  if(param != NULL)
    memcpy(&buf[24], param, param_len);

  memcpy(&buf[preamble_len], script, script_len);

  int ret_val = serial_send(&pgm->fd, buf, message_len);
  if (ret_val < 0) {
    pmsg_error("Sending script failed");
  }
  return ret_val;
}

static int pickit5_read_response(const PROGRAMMER *pgm) {
  unsigned char *buf = my.rxBuf;

  if(serial_recv(&pgm->fd, buf, 512) < 0) {
    pmsg_error("reading from PICkit failed");
    return ERROR_USB_RECV;
  }
  unsigned int status = pickit5_array_to_uint32(&buf[0]);

  if(status != 0x0D) {
    pmsg_error("unexpected response");
    return ERROR_BAD_RESPONSE;
  }

  return 0;
}

static int pickit5_send_script_done(const PROGRAMMER *pgm) {
  unsigned char script_done[16];
  unsigned int script_done_type = 0x0103;

  pickit5_create_payload_header(script_done, script_done_type, 16, 0);
  if(serial_send(&pgm->fd, script_done, 16) < 0) {
    pmsg_error("failed sending script done message");
    return ERROR_USB_SEND;
  }
  return pickit5_read_response(pgm);
}

static int pickit5_open(PROGRAMMER *pgm, const char *port) {
  if(!pgm->cookie)              // Sanity
    return -1;
  pmsg_debug("%s(\"%s\")\n", __func__, port);
  union pinfo pinfo;
  LNODEID usbpid;
  int rv = -1;

#if !defined(HAVE_LIBUSB)
  pmsg_error("need to be compiled with USB or HIDAPI support\n");
  return -1;
#endif

  if(!str_starts(port, "usb")) {
    pmsg_error("port names must start with usb\n");
    return -1;
  }
  unsigned int new_vid = 0, new_pid = 0;
  char *vidp, *pidp;

  /*
   * The syntax for usb devices is defined as:
   *
   * -P usb:vid:pid
   * -P usb::pid
   * -P usb:serialnumber
   * -P usb
   *
   * First we check if we have two colons.
   * Then check the filed between the two colons is empty
   * Parse VID as hex number
   * If it is empty, assume Microchip VID
   * The PID is handled similary but can not be empty
   *
   * If there are fewer than two colons nothing is changed
   */

  vidp = strchr(port, ':');
  if(vidp != NULL) {
    vidp += 1;
    pidp = strchr(vidp, ':');
    if(pidp != NULL) {
      if(vidp != pidp) {        // User specified an VID
        // First: Handle VID input
        if(sscanf(vidp, "%x", &new_vid) != 1) {
          pmsg_error("failed to parse -P VID input %s: unexpected format", vidp);
          return -1;
        }
      } else {                  // VID space empty: default to Microchip
        new_vid = USB_VENDOR_MICROCHIP;
      }

      // Now handle PID input
      if(sscanf(pidp + 1, "%x", &new_pid) != 1) {
        pmsg_error("failed to parse -P PID input %s: unexpected format", pidp+1);
        return -1;
      }

      if((new_vid != 0) && (new_pid != 0)) {
        pmsg_notice("overwriting VID:PID to %04x:%04x\n", new_vid, new_pid);
        port = "usb";           // Overwrite the string to avoid confusing the libusb
      }
    }                           // pidp == NULL means vidp could point to serial number
  }                             // vidp == NULL means just 'usb'

  // If the config entry did not specify a USB PID, insert the default one
  if(lfirst(pgm->usbpid) == NULL)
    ladd(pgm->usbpid, (void *) USB_DEVICE_PICKIT5);

  pinfo.usbinfo.vid = pgm->usbvid? pgm->usbvid: USB_VENDOR_MICROCHIP;

  // PICkit 5 does not have support for HID, so no need to support it
  serdev = &usb_serdev;
  if(new_pid != 0 && new_vid != 0) {    // In case a specific VID/PID was specified
    pinfo.usbinfo.vid = new_vid;
    pinfo.usbinfo.pid = new_pid;
    pinfo.usbinfo.flags = PINFO_FL_SILENT;
    pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
    pgm->fd.usb.rep = USB_PK5_CMD_READ_EP;  // Command read
    pgm->fd.usb.wep = USB_PK5_CMD_WRITE_EP; // Command write
    pgm->fd.usb.eep = 0x00;
    pgm->port = port;
    rv = serial_open(port, pinfo, &pgm->fd);
  } else {                      // Otherwise walk the list of config file PIDs
    for(usbpid = lfirst(pgm->usbpid); rv < 0 && usbpid != NULL; usbpid = lnext(usbpid)) {
      pinfo.usbinfo.flags = PINFO_FL_SILENT;
      pinfo.usbinfo.pid = *(int *) (ldata(usbpid));
      pgm->fd.usb.max_xfer = USB_PK5_MAX_XFER;
      pgm->fd.usb.rep = USB_PK5_CMD_READ_EP;  // Command read
      pgm->fd.usb.wep = USB_PK5_CMD_WRITE_EP; // Command write
      pgm->fd.usb.eep = 0x00;

      pgm->port = port;
      rv = serial_open(port, pinfo, &pgm->fd);
    }
  }

  // Make USB serial number available to programmer
  if(serdev && serdev->usbsn) {
    pgm->usbsn = serdev->usbsn;
    my.pk_op_mode = PK_OP_FOUND;

    if(pinfo.usbinfo.pid == USB_DEVICE_PICKIT5)
      my.pgm_type = PGM_TYPE_PK5;
    else if(pinfo.usbinfo.pid == USB_DEVICE_PICKIT4_PIC_MODE)
      my.pgm_type = PGM_TYPE_PK4;
    else if(pinfo.usbinfo.pid == USB_DEVICE_SNAP_PIC_MODE)
      my.pgm_type = PGM_TYPE_SNAP;
  }

  return rv;
}

static void pickit5_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  pickit5_set_vtarget(pgm, 0.0); // Switches off PICkit voltage regulator if enabled

  serial_close(&pgm->fd);
}

static void pickit5_disable(const PROGRAMMER *pgm) {
  return;
}

static void pickit5_enable(PROGRAMMER *pgm, const AVRPART *p) {
  // Overwrite page sizes so that avrdude uses pages read/writes
  // This will reduce overhead and increase speed
  AVRMEM *mem;

  if((mem = avr_locate_sram(p)))
    mem->page_size = mem->size < 256? mem->size: 256;
  if((mem = avr_locate_eeprom(p)))
    mem->page_size = mem->size < 32? mem->size: 32;

  if((mem = avr_locate_sib(p))) {       // This is mandatory as PICkit is reading all 32 bytes at once
    mem->page_size = 32;
    mem->readsize = 32;
  }
  if(is_debugwire(pgm)) {
    if((mem = avr_locate_flash(p))) {
      mem->page_size = 1024;  // The Flash Write function needs 1600 bytes
      mem->readsize = 1024;   // this reduces overhead and speeds things up
    }
  }
}

static void pickit5_display(const PROGRAMMER *pgm, const char *p) {
  unsigned char *app = my.app_version;
  unsigned char *sn = my.sernum_string;

  if(pickit5_get_fw_info(pgm) < 0) {
    msg_error("failed to get firmware info\n");
    return;
  }

  msg_info("Application version   : %02x.%02x.%02x\n", app[0], app[1], app[2]);
  msg_info("Serial number         : %s\n", sn);
  my.pk_op_mode = PK_OP_RESPONDS;
}

static void pickit5_print_parms(const PROGRAMMER *pgm, FILE *fp) {
  pickit5_get_vtarget(pgm, NULL);
  char pgm_str [24];
  // using snprintf to generate a String with variable length
  // this allows to use a left-aligning with padding to align the output
  snprintf(pgm_str, 21, "%s %s", dev_prog_modes(pgm->prog_modes), "clock");
  fmsg_out(fp, "%-21s : %u kHz\n", pgm_str, my.actual_pgm_clk / 1000);
  fmsg_out(fp, "Target Vcc            : %1.2f V\n", my.measured_vcc);
  fmsg_out(fp, "Target current        : %3u mA\n", my.measured_current);
}

static int pickit5_updi_init(const PROGRAMMER *pgm, const AVRPART *p, double v_target) {
  // Get SIB so we can get the NVM Version
  if(pickit5_program_enable(pgm, p) < 0)
    return -1;
  
  if(pickit5_updi_read_sib(pgm, p, my.sib_string) < 0) {
      pmsg_error("failed to obtain System Info Block\n");
      return -1;
  }
  

  if(pickit5_read_dev_id(pgm, p) < 0) {
    pmsg_error("failed to obtain device ID\n");
    return -1;
  }

  double bitclock = pgm->bitclock;
  unsigned int baud = pgm->baudrate;

  if(baud == 200000) {          // If baud unchanged
    if(bitclock > 0.0) {
      baud = (unsigned int) (1.0 / pgm->bitclock); // Bitclock in us
    }
  } else {
    if(bitclock > 0.0) {
      pmsg_error("both -b baudrate and -B bitclock given; please use only one, aborting\n");
      return -1;
    }
  }

  if(baud < 300) {              // Better be safe than sorry
    pmsg_warning("UPDI needs a higher clock for operation, increasing UPDI to 300 Hz\n");
    baud = 300;
  }
  if(baud > 225000) {
    if(v_target < 2.9) {
      pmsg_warning("UPDI needs a voltage of more than 2.9 V for a faster baudrate, limiting UPDI to 225 kHz\n");
      baud = 225000;
    } else {
      if(baud > 900000) {
        pmsg_warning("requested clock %u Hz too high, limiting UPDI to 900 kHz\n", baud);
        baud = 900000;
      }
      pickit5_set_sck_period(pgm, 1.0 / 100000);       // Start with 200 kHz
      pickit5_updi_write_cs_reg(pgm, UPDI_ASI_CTRLA, 0x01); // Change UPDI clock to 16 MHz
      unsigned char ret_val = 0;

      pickit5_updi_read_cs_reg(pgm, UPDI_ASI_CTRLA, &ret_val);
      if(ret_val != 0x01) {
        pmsg_warning("failed to change UPDI clock, falling back to 225 kHz\n");
        baud = 225000;
      }
    }
  }
  if(pickit5_set_sck_period(pgm, 1.0 / baud) >= 0) {
    pmsg_notice("UPDI speed set to %i kHz\n", baud / 1000);
    my.actual_pgm_clk = baud;
  } else {
    pmsg_warning("failed to set UPDI speed, continuing\n");
    my.actual_pgm_clk = 100000;       // Default clock?
  }
  return 1;
}


static int pickit5_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  if(!pgm->cookie)
    return -1;

  if(my.pk_op_mode < PK_OP_FOUND) {
    pmsg_error("failed to find a connected PICkit\n");
    return -1;
  }

  int rc = -1;

  if(both_debugwire(pgm, p)) {
    rc = get_pickit_dw_script(&(my.scripts), p->desc);
  } else if(both_isp(pgm, p)) {
    rc = get_pickit_isp_script(&(my.scripts), p->desc);
  } else if(both_jtag(pgm, p) || both_xmegajtag(pgm, p)) {
    rc = get_pickit_jtag_script(&(my.scripts), p->desc);
  } else if(both_updi(pgm, p)) {
    rc = get_pickit_updi_script(&(my.scripts), p->desc);
  } else if(both_tpi(pgm, p)) {
    rc = get_pickit_tpi_script(&(my.scripts), p->desc);
  } else if(both_pdi(pgm, p)) {
    rc = get_pickit_pdi_script(&(my.scripts), p->desc);
  }

  if(rc == -1) {
    pmsg_error("no matching prog_mode found, aborting\n");
    return -1;
  }
  if(rc == -2) {
    pmsg_error("failed to match scripts to %s, aborting\n", p->desc);
    return -1;
  }

  if(my.hvupdi_enabled > 0) {
    if(p->hvupdi_variant == 0)
      pmsg_notice("high-voltage SYSCFG0 override on UPDI Pin enabled\n");
    if(p->hvupdi_variant == 2)
      pmsg_notice("high-voltage SYSCFG0 override on RST Pin enabled\n");
  }

  if(my.pk_op_mode < PK_OP_RESPONDS) {
    if(pickit5_get_fw_info(pgm) < 0)    // PK responds: we can try to enable voltage
      return -1;
    my.pk_op_mode = PK_OP_RESPONDS;
  }

  pickit5_set_ptg_mode(pgm);
  pickit5_set_vtarget(pgm, 0.0);        // Avoid the edge case when avrdude was CTRL+C'd but still provides power

  // Now we try to figure out if we have to supply power from PICkit
  double v_target;

  pickit5_get_vtarget(pgm, &v_target);
  if(v_target < 1.8) {
    if(my.power_source == POWER_SOURCE_NONE) {
      pmsg_warning("no external voltage detected but continuing anyway\n");
    } else if(my.power_source == POWER_SOURCE_INT) {
      pmsg_notice("no extenal Voltage detected; trying to supply from PICkit\n");
        if(both_xmegajtag(pgm, p) || both_pdi(pgm, p)) {
          if(my.target_voltage > 3.6) {
            pmsg_error("xmega part selected but requested voltage is over 3.6V, aborting.");
            return -1;
          }
        }

      if(pickit5_set_vtarget(pgm, my.target_voltage) < 0)
        return -1;              // Set requested voltage

      if(pickit5_get_vtarget(pgm, &v_target) < 0)
        return -1;              // Verify voltage

      if(v_target < my.target_voltage - 0.5   // Voltage supply is not accurate: allow some room
        || v_target > my.target_voltage + 0.15) {
        pmsg_error("target voltage out of range, aborting\n");
        return -1;
      }
    } else {
      pmsg_error("no external voltage detected, aborting; overwrite this check with -x vtarg=0\n");
      return -1;
    }
  } else {
    my.power_source = POWER_SOURCE_EXT;        // Overwrite user input
    pmsg_notice("external Voltage detected: will not supply power\n");
  }

  my.pk_op_mode = PK_OP_READY;
  my.dW_switched_isp = 0;

  if(is_updi(pgm) && (pickit5_updi_init(pgm, p, v_target) < 0)) {
    return -1;  // UPDI got it's own init as it is well enough documented to select the
  } else {      //  CLKDIV based on the voltage and requested baud
    double bitclock = pgm->bitclock;
    unsigned int baud = pgm->baudrate;

    if(baud == 125000) {          // If baud unchanged
      if(bitclock > 0.0) {
        baud = (unsigned int) (1.0 / pgm->bitclock); // Bitclock in us
      }
    } else {
      if(bitclock > 0.0) {
        pmsg_error("both -b baudrate and -B bitclock given; please use only one, aborting\n");
        return -1;
      }
    }

    // JTAG __requires__ setting the speed before program enable
    pickit5_set_sck_period(pgm, 1.0 / baud);

    if(pickit5_program_enable(pgm, p) < 0) {
      pmsg_error("Failed to enable programming mode\n");
      return -1;
    }
    if(pickit5_read_dev_id(pgm, p) < 0) {
      pmsg_error("failed to obtain device ID\n");
      return -1;
    }
  }

  pickit5_program_enable(pgm, p);
  return 0;
}

static int pickit5_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return -2;
}

static int pickit5_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *enter_prog = my.scripts.EnterProgMode;
  unsigned int enter_prog_len = my.scripts.EnterProgMode_len;

  if(my.hvupdi_enabled && (my.pgm_type != PGM_TYPE_SNAP)) {   // SNAP has no HV generation
    if(p->hvupdi_variant == HV_UPDI_VARIANT_0) {        // High voltage generation on UPDI line
      enter_prog = my.scripts.EnterProgModeHvSp;
      enter_prog_len = my.scripts.EnterProgModeHvSp_len;
    } else if(p->hvupdi_variant == HV_UPDI_VARIANT_2) { // High voltage generation on RST line
      enter_prog = my.scripts.EnterProgModeHvSpRst;
      enter_prog_len = my.scripts.EnterProgModeHvSpRst_len;
    }
  }
  if(my.pk_op_mode == PK_OP_READY) {
    if(pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if(pickit5_read_response(pgm) < 0)
      return -1;
  }
  return 0;
}

static int pickit5_program_disable(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *enter_prog = my.scripts.ExitProgMode;
  unsigned int enter_prog_len = my.scripts.ExitProgMode_len;

  if(my.pk_op_mode == PK_OP_READY) {
    if(pickit5_send_script(pgm, SCR_CMD, enter_prog, enter_prog_len, NULL, 0, 0) < 0)
      return -1;

    if(pickit5_read_response(pgm) < 0)
      return -1;
  }
  return 0;
}

static int pickit5_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);

  pickit5_program_enable(pgm, p);
  if (is_debugwire(pgm)) {    // dW Chip erase doesn't seem to be working, use ISP
    pickit5_dw_switch_to_isp(pgm, p);
  }
  const unsigned char *chip_erase = my.scripts.EraseChip;
  unsigned int chip_erase_len = my.scripts.EraseChip_len;

  if(pickit5_send_script(pgm, SCR_CMD, chip_erase, chip_erase_len, NULL, 0, 0) >= 0) {
    if(pickit5_read_response(pgm) >= 0) {
      if(pickit5_array_to_uint32(&(my.rxBuf[16])) == 0x00) {
        pmsg_info("target successfully erased\n");
        my.pk_op_mode = PK_OP_READY;
        pickit5_program_enable(pgm, p);
        return 0;
      }
    }
  }

  pmsg_error("chip erase failed\n");
  return -1;
}

static int pickit5_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {
  return pickit5_read_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

static int pickit5_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem,
  unsigned int page_size, unsigned int address, unsigned int n_bytes) {
  return pickit5_write_array(pgm, p, mem, address, n_bytes, &mem->buf[address]);
}

// Sets Frequency in kHz
static int pickit5_set_sck_period(const PROGRAMMER *pgm, double sckperiod) {
  pmsg_debug("%s()\n", __func__);
  double frq = (0.001 / sckperiod) + 0.5;       // 1ms/period = kHz; round up
  const unsigned char *set_speed = my.scripts.SetSpeed;
  unsigned int set_speed_len = my.scripts.SetSpeed_len;
  unsigned char buf[4];
  if (set_speed_len > 0) {  // debugWire is fun . . .
    pickit5_uint32_to_array(buf, frq);
    if(pickit5_send_script(pgm, SCR_CMD, set_speed, set_speed_len, buf, 4, 0) >= 0) {
      if(pickit5_read_response(pgm) >= 0)
      return 0;
    }
  }
  pmsg_error("Failed to set speed.\n");
  return -1;
}

static int pickit5_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {
  int rc = 0;
  if(mem_is_in_fuses(mem)) {
    if(is_isp(pgm)){
      rc = pickit5_isp_write_fuse(pgm, mem, value);
    } else if(is_debugwire(pgm)){
      rc = pickit5_dw_write_fuse(pgm, p, mem, value);
    } else if(both_jtag(pgm, p)){
      rc = pickit5_jtag_write_fuse(pgm, p, mem, value);
    }
  }
  if (rc == 0) {
    rc = pickit5_write_array(pgm, p, mem, addr, 1, &value);
  }

  if(rc < 0)
    return rc;
  return 0;
}

static int pickit5_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  int rc = 0;
  if(mem_is_in_fuses(mem)) {
    if(is_isp(pgm)){
      rc = pickit5_isp_read_fuse(pgm, mem, value);
    } else if(is_debugwire(pgm)){
      rc = pickit5_dw_read_fuse(pgm, p, mem, value);
    } else if(both_jtag(pgm, p)){
      rc = pickit5_jtag_read_fuse(pgm, p, mem, value);
    }
  }
  if(rc == 0) {
    rc = pickit5_read_array(pgm, p, mem, addr, 1, value);
  }

  if(rc < 0)
    return rc;
  return 0;
}

// UPDI Specific function providing a reduced overhead when writing a single byte
static int pickit5_updi_write_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char value) {

  if(mem->size < 1 || addr > (unsigned long) mem->size) {
    pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned) addr, mem->desc, mem->size);
    return -1;
  }
  addr += mem->offset;
  pmsg_debug("%s(0x%4X, %i)\n", __func__, (unsigned) addr, value);
  // This script is based on WriteCSreg; reduces overhead by avoiding writing data EP
  const unsigned char h_len = 24;       // 16 + 8
  const unsigned char p_len = 8;
  const unsigned char s_len = 8;
  const unsigned char m_len = h_len + p_len + s_len;

  unsigned char write8_fast[] = {
    0x00, 0x01, 0x00, 0x00,     // [0]  SCR_CMD
    0x00, 0x00, 0x00, 0x00,     // [4]  always 0
    m_len, 0x00, 0x00, 0x00,    // [8]  message length = 16 + 8 + param (8) + script (8) = 40
    0x00, 0x00, 0x00, 0x00,     // [12] keep at 0 to receive the data in the "response"

    p_len, 0x00, 0x00, 0x00,    // [16] param length: 8 bytes
    s_len, 0x00, 0x00, 0x00,    // [20] length of script: 8 bytes

    0x00, 0x00, 0x00, 0x00,     // [24] param: address to write to, will be overwritten
    0x00, 0x00, 0x00, 0x00,     // [28] param: byte to write, will be overwritten

    // Script itself:
    0x91, 0x00,                 // Copy first 4 bytes of param to reg 0
    0x91, 0x01,                 // Copy second 4 bytes of param to reg 1
    0x1E, 0x06, 0x00, 0x01,     // Store to address in reg 0 the byte in reg 1
  };
  write8_fast[24] = (((unsigned char *) &addr)[0]);
  write8_fast[25] = (((unsigned char *) &addr)[1]);
  write8_fast[28] = value;

  serial_send(&pgm->fd, write8_fast, m_len);
  unsigned char *buf = my.rxBuf;

  if(serial_recv(&pgm->fd, buf, 512) >= 0) { // Read response
    if(buf[0] == 0x0D) {
      return 0;
    }
  }
  return -1;
}

// UPDI-specific function providing a reduced overhead when reading a single byte
static int pickit5_updi_read_byte(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, unsigned char *value) {
  if(mem_is_sram(mem) || mem_is_io(mem)
    || mem_is_lock(mem) || mem_is_in_fuses(mem)) {
    if(mem->size < 1 || addr > (unsigned long) mem->size) {
      pmsg_error("address %i out of range for %s [0, %i]\n", (unsigned) addr, mem->desc, mem->size);
      return -1;
    }
    addr += mem->offset;
    pmsg_debug("%s(0x%4X)\n", __func__, (unsigned int) addr);
    // This script is based on ReadSIB; reduces overhead by avoiding readind data EP
    const unsigned char h_len = 24;     // 16 + 8
    const unsigned char p_len = 4;
    const unsigned char s_len = 6;
    const unsigned char m_len = h_len + p_len + s_len;

    unsigned char read8_fast[] = {
      0x00, 0x01, 0x00, 0x00,   // [0]  SCR_CMD
      0x00, 0x00, 0x00, 0x00,   // [4]  always 0
      m_len, 0x00, 0x00, 0x00,  // [8]  message length = 16 + 8 + param (4) + script (6) = 34
      0x00, 0x00, 0x00, 0x00,   // [12] keep at 0 to receive the data in the "response"

      p_len, 0x00, 0x00, 0x00,  // [16] param length: 4 bytes
      s_len, 0x00, 0x00, 0x00,  // [20] length of script: 6 bytes

      0x00, 0x00, 0x00, 0x00,   // [24] param: address to read from, will be overwritten

      // Script itself:
      0x91, 0x00,               // Copy first 4 bytes of param to reg 0
      0x1E, 0x03, 0x00,         // Load byte from address in reg 0
      0x9F                      // Send data from 0x1E to "response"
    };
    read8_fast[24] = (((unsigned char *) &addr)[0]);
    read8_fast[25] = (((unsigned char *) &addr)[1]);

    serial_send(&pgm->fd, read8_fast, m_len);
    unsigned char *buf = my.rxBuf;

    if(serial_recv(&pgm->fd, buf, 512) >= 0) {  // Read response
      if(buf[0] == 0x0D) {
        if(buf[20] == 0x01) {
          *value = buf[24];
          return 0;
        }
      }
    }
    return -1;
  } else {                      // Fall back to standard function
    int rc = pickit5_read_array(pgm, p, mem, addr, 1, value);

    if(rc < 0)
      return rc;
    return 0;
  }
}

// Return numbers of byte written
static int pickit5_write_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)", __func__, mem->desc, (unsigned int) addr, mem->offset, len);

  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot write to %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot write to %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  if(is_debugwire(pgm) && !mem_is_in_flash(mem)) {  // for flash programming, stay in 
    pickit5_isp_switch_to_dw(pgm, p);
  }
  if(is_tpi(pgm)) {
    pickit5_tpi_write(pgm, p, mem, addr, len, value);
  }

  const unsigned char *write_bytes = NULL;
  unsigned int write_bytes_len = 0;

  if((mem_is_in_flash(mem) && (len == mem->page_size))) {
    write_bytes     = my.scripts.WriteProgmem;
    write_bytes_len = my.scripts.WriteProgmem_len;
  } else if (mem_is_io(mem) && my.scripts.WriteMemIO != NULL) {
    write_bytes     = my.scripts.WriteMemIO;
    write_bytes_len = my.scripts.WriteMemIO_len;
  } else if(mem_is_eeprom(mem) && my.scripts.WriteDataEEmem != NULL) {
    write_bytes     = my.scripts.WriteDataEEmem;
    write_bytes_len = my.scripts.WriteDataEEmem_len;
  } else if(mem_is_in_fuses(mem) && my.scripts.WriteConfigmemFuse != NULL) {
    write_bytes     = my.scripts.WriteConfigmemFuse;
    write_bytes_len = my.scripts.WriteConfigmemFuse_len;
  } else if (mem_is_lock(mem) && my.scripts.WriteConfigmemLock != NULL) {
    write_bytes     = my.scripts.WriteConfigmemLock;
    write_bytes_len = my.scripts.WriteConfigmemLock_len;
  } else if(mem_is_user_type(mem) && my.scripts.WriteIDmem != NULL) {
    write_bytes     = my.scripts.WriteIDmem;
    write_bytes_len = my.scripts.WriteIDmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK
    if((len == 1) && is_updi(pgm)) {
      return pickit5_updi_write_byte(pgm, p, mem, addr, value[0]);
    }
    write_bytes = my.scripts.WriteMem8;
    write_bytes_len = my.scripts.WriteMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }


  addr += mem->offset;

  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  int rc = pickit5_download_data(pgm, write_bytes, write_bytes_len, buf, 8, value, len);
  if(rc == -1) {
    pmsg_error("sending script failed\n");
  }
  if(rc == -2) {
    pmsg_error("reading script response failed\n");
  }
  if(rc == -3) {
    pmsg_error("failed when sending data\n");
  }
  if(rc == -4) {
    pmsg_error("error check failed\n");
  }
  if(rc == -5) {
    pmsg_error("sending script done message failed\n");
  }
  if(rc < 0) {
    return -1;
  } else {
    return len; 
  }
}

// Return numbers of byte read
static int pickit5_read_array(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {
  pmsg_debug("%s(%s, addr: 0x%04x, offset: %i, len: %i)", __func__, mem->desc, (unsigned int) addr, mem->offset, len);


  if(len > mem->size || mem->size < 1) {
    pmsg_error("cannot read from %s %s owing to its size %d\n", p->desc, mem->desc, mem->size);
    return -1;
  }
  if(addr >= (unsigned long) mem->size) {
    pmsg_error("cannot read from %s %s as address 0x%04lx outside range [0, 0x%04x]\n",
      p->desc, mem->desc, addr, mem->size - 1);
    return -1;
  }

  if(mem_is_signature(mem)) { // DeviceID is read only once and buffered
    if(len == 1) {
      *value = my.devID[addr];
      return 0;
    }
    return -1;
  }

  if(is_debugwire(pgm)) {
    pickit5_isp_switch_to_dw(pgm, p);
  }
  if(is_tpi(pgm)) {
    return pickit5_tpi_read(pgm, p, mem, addr, len, value);
  }

  const unsigned char *read_bytes = NULL;
  unsigned int read_bytes_len = 0;

  if(mem_is_in_flash(mem)) {
    read_bytes      = my.scripts.ReadProgmem;
    read_bytes_len  = my.scripts.ReadProgmem_len;
  } else if (mem_is_calibration(mem) && my.scripts.ReadCalibrationByte != NULL) {
    read_bytes      = my.scripts.ReadCalibrationByte;
    read_bytes_len  = my.scripts.ReadCalibrationByte_len;
  } else if (mem_is_io(mem) && my.scripts.ReadMemIO != NULL) {
    read_bytes      = my.scripts.ReadMemIO;
    read_bytes_len  = my.scripts.ReadMemIO_len;
  } else if(mem_is_eeprom(mem) && my.scripts.ReadDataEEmem != NULL) {
    read_bytes      = my.scripts.ReadDataEEmem;
    read_bytes_len  = my.scripts.ReadDataEEmem_len;
  } else if(mem_is_in_fuses(mem) && my.scripts.ReadConfigmemFuse != NULL) {
    read_bytes      = my.scripts.ReadConfigmemFuse;
    read_bytes_len  = my.scripts.ReadConfigmemFuse_len;
  } else if (mem_is_lock(mem) && my.scripts.ReadConfigmemLock != NULL) {
    read_bytes      = my.scripts.ReadConfigmemLock;
    read_bytes_len  = my.scripts.ReadConfigmemLock_len;
  } else if(mem_is_user_type(mem) && my.scripts.ReadIDmem != NULL) {
    read_bytes      = my.scripts.ReadIDmem;
    read_bytes_len  = my.scripts.ReadIDmem_len;
  } else if(mem_is_sib(mem)) {
    if(len == 1) {
      *value = my.sib_string[addr];
      return 0;
    } else if(len == 32) {
      memcpy(value, my.sib_string, 32);
      return 32;
    }
    return -1;
  } else if((mem_is_in_sigrow(mem) || mem_is_user_type(mem) || mem_is_a_fuse(mem))
            && my.scripts.ReadConfigmem != NULL) {
    read_bytes      = my.scripts.ReadConfigmem;
    read_bytes_len  = my.scripts.ReadConfigmem_len;
  } else if(!mem_is_readonly(mem)) { // SRAM, IO, LOCK, USERROW
    if((len == 1) && is_updi(pgm)) {
      return pickit5_updi_read_byte(pgm, p, mem, addr, value);
    }
    read_bytes      = my.scripts.ReadMem8;
    read_bytes_len  = my.scripts.ReadMem8_len;
  } else {
    pmsg_error("unsupported memory %s\n", mem->desc);
    return -2;
  }

  addr += mem->offset;
  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  int rc = pickit5_upload_data(pgm, read_bytes, read_bytes_len, buf, 8, value, len);

  if(rc == -1) {
    pmsg_error("sending script failed\n");
  } else if (rc == -2) {
    pmsg_error("unexpected read response\n");
  } else if (rc == -3) {
    pmsg_error("reading data memory failed\n");
  } else if (rc == -4) {
    pmsg_error("sending script done message failed\n");
  }
  
  if(rc < 0) {
    return -1;
  } else {
    return len; 
  }
}


static int pickit5_read_dev_id(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_id = my.scripts.GetDeviceID; // Defaults
  unsigned int read_id_len = my.scripts.GetDeviceID_len;

  if(is_updi(pgm)) {
    if(my.nvm_version >= '0' && my.nvm_version <= '9') {
      read_id = get_devid_script_by_nvm_ver(my.nvm_version); // Only address changes, not length
    }
  } else if (is_debugwire(pgm)) {
    unsigned char scr [] = {0x7D, 0x00, 0x00, 0x00};  // Not sure what this does
    unsigned int scr_len = sizeof(scr);
    pickit5_send_script_cmd(pgm, scr, scr_len, NULL, 0);
    pickit5_program_enable(pgm, p);
    if (my.rxBuf[17] == 0x0E) {   // Errors figured out during 6 hours of failing to get it to work
      if(my.rxBuf[16] == 0x10) {  // with the serial/bootloader auto-reset circuit.
        pmsg_error("Debug Wire transmission error, Aborting. (Is the Pullup >=10 kOhms?)");
      } else if(my.rxBuf[16] == 58) {
        pmsg_error("Debug Wire transmission error, Aborting. (Please remove any caps on RESET)");
      } else {
        pmsg_error("Error: %d", my.rxBuf[16]);
      }
      return -1;
    }
    const unsigned char get_sig [] = {  // *screams* why was this function not in the scripts?????
      0x90, 0x0C, 0x03, 0x00, 0x00, 0x00, // Set reg to 0x03
      0x1e, 0x45, 0x0C,                   // Send 0xF0 + reg and receive 2 bytes (found by trial and error)
      0x9D,                               // place word into status response
    };
    if(pickit5_send_script_cmd(pgm, get_sig, sizeof(get_sig), NULL, 0) >= 0) {
      unsigned char len = my.rxBuf[20];
      if(len == 0x02) {  // debugWire
        my.devID[0] = 0x1E;     // doesn't send the first byte, fill it in
        my.devID[1] = my.rxBuf[25]; // Flip byte order
        my.devID[2] = my.rxBuf[24];
        return 0;
      }
    }
    return -1;
  }

  if(pickit5_send_script_cmd(pgm, read_id, read_id_len, NULL, 0) < 0) {
    return -1;
  }

  if(my.rxBuf[0] == 0x0D) {
    unsigned char len = my.rxBuf[20];
    if(len == 0x03 || len == 0x04) {  // just DevId or UPDI with revision
      memcpy(my.devID, &my.rxBuf[24], len);
    } else {
      if(my.hvupdi_enabled && p->hvupdi_variant == HV_UPDI_VARIANT_2) {
        pmsg_info("failed to get DeviceID with activated HV Pulse on RST\n");
        pmsg_info("if the wiring is correct, try connecting a 16 V, 1 uF cap between RST and GND\n");
      } else {
        pmsg_error("Length (%u) mismatch of returned Device ID.\n", len);
      }
      return -1;
    }
  }
  return 0;
}

static int pickit5_updi_read_sib(const PROGRAMMER *pgm, const AVRPART *p, char *sib) {
  pmsg_debug("%s()\n", __func__);
  const unsigned char *read_sib = my.scripts.ReadSIB;
  unsigned int read_sib_len = my.scripts.ReadSIB_len;

  if(pickit5_send_script_cmd(pgm, read_sib, read_sib_len, NULL, 0) < 0) {
    return -1;
  }
  unsigned int ret_len = pickit5_array_to_uint32(&(my.rxBuf[20]));

  if(ret_len == 32) {
    memcpy(sib, &my.rxBuf[24], 32);
    sib[31] = 0x00;             // Known zero-terminator
    my.nvm_version = sib[10];
    return 0;
  }
  my.nvm_version = 0xFF;
  return -1;
}

static int pickit5_read_chip_rev(const PROGRAMMER *pgm, const AVRPART *p, unsigned char *chip_rev) {
  pmsg_debug("%s()\n", __func__);
  *chip_rev = my.devID[3];
  return 0;
}

static int pickit5_updi_write_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char value) {
  pmsg_debug("%s(%u, %i)", __func__, addr, value);
  const unsigned char *write_cs = my.scripts.WriteCSreg;
  unsigned int write_cs_len = my.scripts.WriteCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }

  unsigned char buf[2];

  buf[0] = addr;
  buf[1] = value;

  if (pickit5_send_script_cmd(pgm, write_cs, write_cs_len, buf, 2) < 0) {
    pmsg_error("CS Reg write failed\n");
    return -1;
  }

  return 1;
}

static int pickit5_updi_read_cs_reg(const PROGRAMMER *pgm, unsigned int addr, unsigned char *value) {
  pmsg_debug("%s(%i)\n", __func__, addr);
  const unsigned char *read_cs = my.scripts.ReadCSreg;
  unsigned int read_cs_len = my.scripts.ReadCSreg_len;

  if(addr > 0x0C) {
    pmsg_error("CS reg %i out of range [0x00, 0x0C], addr\n", addr);
    return -1;
  }
  unsigned char buf[1];

  buf[0] = addr;
  int ret_val = pickit5_upload_data(pgm, read_cs, read_cs_len, buf, 1, value, 1);

  if(ret_val == -1) {
    pmsg_error("sending script failed\n");
    return -1;
  } else if(ret_val == -2) {
    pmsg_error("unexpected read response\n");
    return -1;
  } else if(ret_val == -3) {
    pmsg_error("reading CS memory failed\n");
    return -1;
  } else if(ret_val == -4) {
    pmsg_error("sending script done message failed\n");
    return -1;
  }
  return 0;
}


static void pickit5_dw_switch_to_isp(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_debug("%s(%u)", __func__, my.dW_switched_isp);
  if(my.dW_switched_isp == 0) {
    if(pickit5_send_script_cmd(pgm, my.scripts.switchtoISP, my.scripts.switchtoISP_len, NULL, 0) >= 0) {
      my.dW_switched_isp = 1;
      pickit5_program_disable(pgm, p);
      if(get_pickit_isp_script(&(my.scripts), p->desc) < 0) {
        pmsg_error("Failed switching scripts, aborting.\n");
        return;
      }
      pickit5_program_enable(pgm, p);
    }
  }
}

static void pickit5_isp_switch_to_dw(const PROGRAMMER *pgm, const AVRPART *p) {
  if(my.dW_switched_isp) {
    // dW_switched_isp is set when accessing fuses where dW has to be switched to ISP
    // we have to power cycle to switch back to dW so that the Scripts work
    // For now, only support power-cycling through PICkit
    if(my.power_source == POWER_SOURCE_INT) {
      pickit5_program_disable(pgm, p);
      pickit5_set_vtarget(pgm, 0.0);
      if(get_pickit_dw_script(&(my.scripts), p->desc) < 0) {
        pmsg_error("Failed switching scripts, aborting.\n");
        return;
      }
      pickit5_set_vtarget(pgm, my.target_voltage);
      pickit5_program_enable(pgm, p);
      my.dW_switched_isp = 0;
    } else {
      pmsg_error("Pickit 5 switched the part to ISP mode when writing fuses.");
      pmsg_error("To continue, the part has to be power cycled and the operation restarted.");
    }
  }
}


static int pickit5_isp_write_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char value) {
  unsigned char write_fuse_isp [] = {
    0x90, 0x00, 0x32, 0x00, 0x00, 0x00, // load 0x32 to r00
    0x1E, 0x37, 0x00,                   // Enable Programming?
    0x90, 0x01, 0x00, 0x00, 0x00, 0x00, // load programming command to r01 (set later)
    0x1E, 0x34, 0x01,                   // Execute write command placed in r01
  };
  unsigned int write_fuse_isp_len = sizeof(write_fuse_isp);
  unsigned int cmd;
  avr_set_bits(mem->op[AVR_OP_WRITE], (unsigned char*)&cmd);
  avr_set_addr(mem->op[AVR_OP_WRITE], (unsigned char*)&cmd, mem_fuse_offset(mem));
  avr_set_input(mem->op[AVR_OP_WRITE], (unsigned char*)&cmd, value);
  cmd = __builtin_bswap32(cmd);         // Swap bitorder
  pickit5_uint32_to_array(&write_fuse_isp[11], cmd);  // fill programming command

  if(pickit5_send_script_cmd(pgm, write_fuse_isp, write_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("Write Fuse Script failed");
    return -1;
  }
  return 1;
}

static int pickit5_isp_read_fuse(const PROGRAMMER *pgm, const AVRMEM *mem, unsigned char *value) {
  // Original Script only supports doing all three at once,
  // doing a custom script felt easier to integrate into avrdude,
  // especially as we already have all the programming commands
  unsigned char read_fuse_isp [] = {
    0x90, 0x00, 0x32, 0x00, 0x00, 0x00, // load 0x32 to r00
    0x1E, 0x37, 0x00,                   // Enable Programming?
    0x90, 0x01, 0x00, 0x00, 0x00, 0x00, // load programming command to r01 (set later)
    0x9B, 0x02, 0x03,                   // load 0x03 to r02
    0x9B, 0x03, 0x00,                   // load 0x00 to r03
    0x1E, 0x35, 0x01, 0x02, 0x03,       // Execute Command placed in r01
    0x9F                                // Send data from temp_reg to host
  };
  unsigned int read_fuse_isp_len = sizeof(read_fuse_isp);
  unsigned int cmd;
  avr_set_bits(mem->op[AVR_OP_READ], (unsigned char*)&cmd);
  avr_set_addr(mem->op[AVR_OP_READ], (unsigned char*)&cmd, mem_fuse_offset(mem));
  cmd = __builtin_bswap32(cmd); // Swap bitorder
  pickit5_uint32_to_array(&read_fuse_isp[11], cmd); // fill programming command

  if(pickit5_send_script_cmd(pgm, read_fuse_isp, read_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("Read Fuse Script failed");
    return -1;
  }
  if (0x01 != my.rxBuf[20]) { // length
    return -1;
  }
  *value = my.rxBuf[24];      // return value
  return 1;
}

// debugWire cannot write nor read fuses, have to change to ISP for that.
// Luckily, there is a custom script doing fuse access on ISP anyway,
// so no need to switch between script sets
static int pickit5_dw_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value) {
  pickit5_dw_switch_to_isp(pgm, p);
  return pickit5_isp_write_fuse(pgm, mem, value);
}

static int pickit5_dw_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value) {
  pickit5_dw_switch_to_isp(pgm, p);
  return pickit5_isp_read_fuse(pgm, mem, value);
}


// gave JTAG also a custom script to make integration into avrdude
// easier. Also encodes all data in script itself instead of using paramters
static int pickit5_jtag_write_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char value) {
  unsigned char write_fuse_jtag [] = {
    0x9b, 0x00, 0x33,                   // Parameter address to r04
    0x9b, 0x0B, 0xFF,                   // Parameter value to r11
    0x9b, 0x0a, 0x0f,                   // Set r10 to 0x0F
    0x9b, 0x02, 0x05,                   // Set r02 to 0x05    (PROG_COMMANDS)
    0x1e, 0x66, 0x02,                   // JTAG Write to Instruction Reg the value in r02
    0x90, 0x02, 0x40, 0x23, 0x00, 0x00, // set r02 to 0x2340  (Enter Fuse Write)
    0x1e, 0x67, 0x02, 0x0a,             // JTAG: Write to Data Reg the value in r02 with a length in r0A(16)
    0x90, 0x03, 0x00, 0x13, 0x00, 0x00, // set r03 to 0x1300  (Load Data Low Byte)
    0x6e, 0x03, 0x0b,                   // r03 += r11         (Set low byte)
    0x1e, 0x67, 0x03, 0x0a,             // JTAG: Write to Data Reg the value in r03 with a length in r0A(16)

    0x60, 0x01, 0x00,                   // Copy r00 to r01    (Write fuse command)
    0x68, 0x01, 0x08,                   // Left Shift r01 by 8
    0x69, 0x00, 0x02, 0x00, 0x00, 0x00, // r00 -= 2
    0x68, 0x00, 0x08,                   // Left Shift r00 by 8

    0x1e, 0x67, 0x01, 0x0a,             // JTAG: Write to D../build_linux/src/avrdude -qq -c pickit5_jtag -xvtarg=4.5 -p m32u4 -Ueeprom:w:0x55:m -Ueeprom:w:0xaa:mata Reg the value in r01 with a length in r0A(16)
    0x1e, 0x67, 0x00, 0x0a,             // JTAG: Write to Data Reg the value in r00 with a length in r0A(16)
    0x1e, 0x67, 0x01, 0x0a,             // JTAG: Write to Data Reg the value in r01 with a length in r0A(16)
    0x1e, 0x67, 0x01, 0x0a,             // JTAG: Write to Data Reg the value in r01 with a length in r0A(16)
    0xa2,                               // do
    0x1e, 0x6b, 0x01, 0x0a,             // JTAG: Write/read Data Reg the value in r01 with a length in r0A(16)
    0xa5, 0x00, 0x02, 0x00, 0x00,       // while ((temp_reg & 0x200) != 0x200)
    0x00, 0x02, 0x00, 0x00, 0x0a, 0x00, //
  };
  unsigned int write_fuse_isp_len = sizeof(write_fuse_jtag);
  if(mem_is_hfuse(mem)) {
    write_fuse_jtag[2] = 0x3B;
  } else if (mem_is_efuse(mem)) {
    write_fuse_jtag[2] = 0x37;
  }
  write_fuse_jtag[5] = value;
  if(pickit5_send_script_cmd(pgm, write_fuse_jtag, write_fuse_isp_len, NULL, 0) < 0) {
    pmsg_error("Write Fuse Script failed");
    return -1;
  }
  return 1;
}

static int pickit5_jtag_read_fuse(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *mem, unsigned char *value) {
  unsigned char read_fuse_jtag [] = {
    0x9b, 0x06, 0x32,                     // load lfuse address to r06 (replaced fo hfuse/efuse)
    0x9b, 0x07, 0x0f,                     // set r07 to 0x07
    0x9b, 0x09, 0x05,                     // set r09 to 0x05 (PROG COMMANDS)
    0x1e, 0x66, 0x09,                     // Write JTAG Instruction in r09
    0x90, 0x09, 0x04, 0x23, 0x00, 0x00,   // set r09 to 0x2304 (Enter Fuse Bit Read)
    0x1e, 0x67, 0x09, 0x07,               // Write JTAG instruction in r09 with length in r07 (7 bits)
    0x60, 0x09, 0x06,                     // copy r06 to r09 (0x32)
    0x68, 0x09, 0x08,                     // left-shift r09 by 8 
    0x1e, 0x67, 0x09, 0x07,               // Write JTAG instruction in r09 with length in r07 (7 bits)
    0x60, 0x09, 0x06,                     // copy r06 to r09
    0x6e, 0x09, 0x02,                     // r09 += 2 (set LSB or so (writing only 7 bits))
    0x68, 0x09, 0x08,                     // left-shift r09 by 8
    0x1E, 0x6B, 0x09, 0x07,               // Write JTAG instruction in r09 with length in r07 (7 bits) and shift data in
    0x9F,                                 // Send temp-reg to return status
  };
  unsigned int read_fuse_jtag_len = sizeof(read_fuse_jtag);
  if(mem_is_hfuse(mem)) {
    read_fuse_jtag[2] = 0x3E;
  } else if (mem_is_efuse(mem)) {
    read_fuse_jtag[2] = 0x3A;
  }

  if(pickit5_send_script_cmd(pgm, read_fuse_jtag, read_fuse_jtag_len, NULL, 0) < 0) {
    pmsg_error("Read Fuse Script failed");
    return -1;
  }
  if (0x01 != my.rxBuf[20]) { // length
    return -1;
  }
  *value = my.rxBuf[24];      // return value
  return 1;

}


// TPI has an unified memory space, meaning that any memory (even SRAM) 
// can be accessed by the same command, meaning that we don't need the
// decision tree found in the "read/write array" functions
static int pickit5_tpi_write(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {

  const unsigned char* write_bytes = my.scripts.WriteProgmem;
  unsigned int write_bytes_len = my.scripts.WriteProgmem_len;
  addr += mem->offset;

  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);

  int rc = pickit5_download_data(pgm, write_bytes, write_bytes_len, buf, 8, value, len);
  if(rc == -1) {
    pmsg_error("sending script failed\n");
  }
  if(rc == -2) {
    pmsg_error("reading script response failed\n");
  }
  if(rc == -3) {
    pmsg_error("failed when sending data\n");
  }
  if(rc == -4) {
    pmsg_error("error check failed\n");
  }
  if(rc == -5) {
    pmsg_error("sending script done message failed\n");
  }
  if(rc < 0) {
    return -1;
  } else {
    return len; 
  }
}

static int pickit5_tpi_read(const PROGRAMMER *pgm, const AVRPART *p,
  const AVRMEM *mem, unsigned long addr, int len, unsigned char *value) {

  const unsigned char* read_bytes = my.scripts.ReadProgmem;
  unsigned int read_bytes_len = my.scripts.ReadProgmem_len;

  addr += mem->offset;
  unsigned char buf[8];

  pickit5_uint32_to_array(&buf[0], addr);
  pickit5_uint32_to_array(&buf[4], len);
  int rc = pickit5_upload_data(pgm, read_bytes, read_bytes_len, buf, 8, value, len);

  if(rc == -1) {
    pmsg_error("sending script failed\n");
  } else if (rc == -2) {
    pmsg_error("unexpected read response\n");
  } else if (rc == -3) {
    pmsg_error("reading data memory failed\n");
  } else if (rc == -4) {
    pmsg_error("sending script done message failed\n");
  }
  if(rc < 0) {
    return -1;
  } else {
    return len; 
  }
}


static int pickit5_send_script_cmd(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len) {

  if(pickit5_send_script(pgm, SCR_CMD, scr, scr_len, param, param_len, 0) < 0) {
    pmsg_error("sending script failed\n");
    return -1;
  }
  if(pickit5_read_response(pgm) < 0) {
    pmsg_error("reading script response failed\n");
    return -2;
  }
  return 0;
}


static int pickit5_download_data(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *send_buf, unsigned int send_len) {

  if(pickit5_send_script(pgm, SCR_DOWNLOAD, scr, scr_len, param, param_len, send_len) < 0) {
    return -1;
  }
  if(pickit5_read_response(pgm) < 0) {
    return -2;
  }
  if(usbdev_data_send(&pgm->fd, send_buf, send_len) < 0) {
    return -3;
  }
  if(pickit5_get_status(pgm, CHECK_ERROR) < 0) {
    return -4;
  }
  if(pickit5_send_script_done(pgm) < 0) {
    return -5;
  }
  return 0;
}

static int pickit5_upload_data(const PROGRAMMER *pgm, const unsigned char *scr, unsigned int scr_len,
  const unsigned char *param, unsigned int param_len, unsigned char *recv_buf, unsigned int recv_len) {

  if(pickit5_send_script(pgm, SCR_UPLOAD, scr, scr_len, param, param_len, recv_len) < 0) {
    return -1;
  }
  if(pickit5_read_response(pgm) < 0) {
    return -2;
  }
  if(usbdev_data_recv(&pgm->fd, recv_buf, recv_len) < 0) {
    return -3;
  }
  if(pickit5_send_script_done(pgm) < 0) {
    return -4;
  }
  return 0;
}

static int pickit5_get_fw_info(const PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);
  unsigned char *buf = my.rxBuf;
  const unsigned char get_fw[] = { 0xE1 };

  if(serial_send(&pgm->fd, get_fw, 1) < 0) {
    pmsg_error("sending command via serial_send() failed\n");
    return -1;
  }

  if(serial_recv(&pgm->fd, buf, 512) < 0) {
    pmsg_error("receiving FW response failed\n");
    return -1;
  }

  if(buf[0] != 0xE1) {
    pmsg_error("unexpected device response for get firmware info command\n");
    return -1;
  }

  memcpy(my.app_version, &(my.rxBuf[3]), 3);
  memcpy(my.fw_info, &(my.rxBuf[7]), 16);
  memcpy(my.sernum_string, &(my.rxBuf[32]), 20);
  my.sernum_string[19] = 0;    // Known zero-terminator
  return 0;
}

static int pickit5_set_vtarget(const PROGRAMMER *pgm, double v) {
  unsigned char set_vtarget[] = {
    0x40,
    0x00, 0x00, 0x00, 0x00,     // Vdd
    0x00, 0x00, 0x00, 0x00,     // Vpp
    0x00, 0x00, 0x00, 0x00,     // Vpp_op
    0x42, 0x43,
  };
  unsigned char power_source[] = {
    0x46, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char disable_power[] = {
    0x44
  };

  if(my.pgm_type >= PGM_TYPE_SNAP) { // SNAP can't supply power, ignore
    return 0;
  }

  if(v < 1.0) {                 // Anything below 1 V equals disabling Power
    pmsg_debug("%s(disable)\n", __func__);
    if (pickit5_send_script_cmd(pgm, power_source, 5, NULL, 0) < 0)
      return -1;

    if (pickit5_send_script_cmd(pgm, disable_power, 1, NULL, 0) < 0)
      return -1;
    usleep(50000);              // There might be some caps, let them discharge
  } else {
    pmsg_debug("%s(%1.2f V)\n", __func__, v);
    power_source[1] = 0x01;
    if (pickit5_send_script_cmd(pgm, power_source, 5, NULL, 0) < 0)
      return -1;

    int vtarg = (int) (v * 1000.0);

    pickit5_uint32_to_array(&set_vtarget[1], vtarg);
    pickit5_uint32_to_array(&set_vtarget[5], vtarg);
    pickit5_uint32_to_array(&set_vtarget[9], vtarg);

    if (pickit5_send_script_cmd(pgm, set_vtarget, 15, NULL, 0) < 0)
      return -1;
  }
  return 0;
}

static int pickit5_get_vtarget(const PROGRAMMER *pgm, double *v) {
  const unsigned char get_vtarget[] = { 0x47, };
  unsigned char *buf = my.rxBuf;

  pmsg_debug("%s()\n", __func__);

  if (pickit5_send_script_cmd(pgm, get_vtarget, 1, NULL, 0) < 0)
      return -1;

  // 24 - internal Vdd [mV]
  // 28 - target Vdd [mV]
  // 48 - Vdd Current Sense [mA]
  my.measured_vcc = pickit5_array_to_uint32(&buf[28]) / 1000.0;
  my.measured_current = pickit5_array_to_uint32(&buf[48]);

  pmsg_notice("target Vdd: %1.2f V, target current: %u mA\n", my.measured_vcc, my.measured_current);

  if(v != NULL)
    *v = my.measured_vcc;
  return 0;
}

static int pickit5_set_ptg_mode(const PROGRAMMER *pgm) {
  if(my.pgm_type >= PGM_TYPE_SNAP)  // Don't bother if Programmer doesn't support PTG
    return 0;                       // Side note: Bitmask would be probably better in the future

  unsigned char ptg_mode[] = {
    0x5E, 0x00, 0x00, 0x00, 0x00,
  };
  unsigned char buf[8];

  pmsg_debug("%s()\n", __func__);

  if (pickit5_upload_data(pgm, ptg_mode, 5, NULL, 0, buf, 4)) {
    return -1;
  }
  return 0;
}

static int pickit5_get_status(const PROGRAMMER *pgm, unsigned char status) {
  unsigned char *buf = my.txBuf;
  const unsigned int type = 0x0105;
  unsigned int key_len = 0;

  if(CHECK_ERROR == status) {
    key_len = strlen("ERROR_STATUS_KEY") + 1;
    memcpy(&buf[16], "ERROR_STATUS_KEY", key_len);
  } else if(BIST_TEST == status) {
    key_len = strlen("BIST Tested") + 1;
    memcpy(&buf[16], "BIST Tested", key_len);
  } else if(BIST_RESULT == status) {
    key_len = strlen("BIST Results") + 1;
    memcpy(&buf[16], "BIST Results", key_len);
  }
  if(0 == key_len) {
    pmsg_error("unknown key type %d passed to %s()", status, __func__);
    return -1;
  }
  unsigned int msg_len = 16 + key_len;

  pickit5_create_payload_header(buf, type, msg_len, 0);
  serial_send(&pgm->fd, buf, msg_len);
  serial_recv(&pgm->fd, my.rxBuf, 512);
  if(pickit5_check_ret_status(pgm) < 0) {
    return -1;
  }
  unsigned int status_len = pickit5_array_to_uint32(&(my.rxBuf[8]));

  if(status_len > 64)
    status_len = 64;
  my.rxBuf[16 + status_len] = 0x00;    // Known zero-terminator
  if(str_starts((const char *) &(my.rxBuf[16]), "NONE") == 0) {
    pmsg_error("PICkit error status report: %s", buf);
    return -1;
  }
  return 0;
}

inline static int pickit5_check_ret_status(const PROGRAMMER *pgm) {
  unsigned char ret = my.rxBuf[0];

  if(0x0D != ret) {
    pmsg_error("PICkit5 bad response %i", ret);
    return -1;
  }
  return 0;
}


void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");

  // Mandatory functions
  pgm->initialize = pickit5_initialize;
  pgm->parseextparams = pickit5_parseextparms;
  pgm->display = pickit5_display;
  pgm->enable = pickit5_enable;
  pgm->disable = pickit5_disable;
  pgm->program_enable = pickit5_program_enable;
  pgm->chip_erase = pickit5_chip_erase;
  pgm->cmd = pickit5_cmd;
  pgm->open = pickit5_open;
  pgm->close = pickit5_close;
  pgm->write_byte = pickit5_write_byte;
  pgm->read_byte = pickit5_read_byte;

  // Optional functions
  pgm->paged_write = pickit5_paged_write;
  pgm->paged_load = pickit5_paged_load;
  pgm->setup = pickit5_setup;
  pgm->teardown = pickit5_teardown;
  pgm->set_sck_period = pickit5_set_sck_period;
  pgm->end_programming = pickit5_program_disable;
  pgm->read_sib = pickit5_updi_read_sib;
  pgm->read_chip_rev = pickit5_read_chip_rev;
  pgm->set_vtarget = pickit5_set_vtarget;
  pgm->get_vtarget = pickit5_get_vtarget;
  pgm->print_parms = pickit5_print_parms;

}

#else                           // defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0)

static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name);

static int pickit5_nousb_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("no usb support; please compile again with libusb installed\n");

  return -1;
}

void pickit5_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "pickit5");
  pgm->open = pickit5_nousb_open;
}
#endif                          // defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)


const char pickit5_desc[] = "Microchip's PICkit 5 Programmer/Debugger";
