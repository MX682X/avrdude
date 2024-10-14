/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * avrdude support for CH341A/B
 * Copyright (C) 2016 Alexey Sadkov, paged access by smr 2023
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
 */

// Interface to the CH341A programmer

#include <ac_cfg.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "avrdude.h"
#include "libavrdude.h"

#include "ch341a.h"
#include "usbdevs.h"

#if defined(HAVE_USB_H)
#include <usb.h>                // Linux/Mac
#elif defined(HAVE_LUSB0_USB_H)
#include <lusb0_usb.h>          // Windows
#endif


#include <sys/time.h>

#if defined(HAVE_USB_H) || defined(HAVE_LUSB0_USB_H)

// Private data for this programmer
struct pdata {
  int sckfreq_hz;
  int USB_init;                 // Used in ch341a_open()
};

#define my (*(struct pdata *) (pgm->cookie))

// ----------------------------------------------------------------------

static void ch341a_setup(PROGRAMMER *pgm);
static void ch341a_teardown(PROGRAMMER *pgm);
static int ch341a_open(PROGRAMMER *pgm, const char *port);
static void ch341a_close(PROGRAMMER *pgm);
static int ch341a_initialize(const PROGRAMMER *pgm, const AVRPART *p);
static int ch341a_spi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res);
static int ch341a_spi(const PROGRAMMER *pgm, const unsigned char *in, unsigned char *out, int size);
static int ch341a_spi_program_enable(const PROGRAMMER *pgm, const AVRPART *p);
static int ch341a_spi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p);

static void ch341a_disable(const PROGRAMMER *pgm);
static void ch341a_enable(PROGRAMMER *pgm, const AVRPART *p);
static void ch341a_display(const PROGRAMMER *pgm, const char *p);

// ch341 requires LSB first: invert the bit order before sending and after receiving
static unsigned char swap_byte(unsigned char byte) {
  byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
  byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
  byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);

  return byte;
}

static int CH341USBTransferPart(const PROGRAMMER *pgm, char dir,
  unsigned char *buff, unsigned int size) {

  if (dir & 0x80) {
    if (serial_recv(&pgm->fd, buff, size) < 0) {
      pmsg_error("serial_recv() failed\n");
    };
  } else {
    if (serial_send(&pgm->fd, buff, size) < 0) {
      pmsg_error("serial_send() failed\n");
    }
  }

  return size;
}

static bool CH341USBTransfer(const PROGRAMMER *pgm, char dir,
  unsigned char *buff, unsigned int size) {

  int pos = 0, bytestransferred;

  while(size) {
    bytestransferred = CH341USBTransferPart(pgm, dir, buff + pos, size);
    if(bytestransferred <= 0)
      return false;
    pos += bytestransferred;
    size -= bytestransferred;
  }

  return true;
}

/*
 * Below the assumed map between UIO command bits, pins on CH341A chip and pins
 * on SPI chip. The UIO stream commands only have 6 bits of output, D6/D7 are
 * SPI inputs.
 *
 * UIO  CH341A pin/name  AVR target
 * -------------------------------------------
 *  D0           15/CS0  RESET
 *  D1           16/CS1  (unused)
 *  D2           17/CS2  (unused)
 *  D3           18/DCK  SCK
 *  D4         19/DOUT2  (unused)
 *  D5          20/DOUT  SDI
 *  D6          21/DIN2  (unused)
 *  D7           22/DIN  SDO
 */

bool CH341ChipSelect(const PROGRAMMER *pgm, unsigned int cs, bool enable) {
  unsigned char cmd[4], res[4];

  memset(cmd, 0, sizeof(cmd));
  memset(res, 0, sizeof(res));
  pmsg_trace("ch341a_ChipSelect()\n");
  if(cs > 2) {
    pmsg_error("invalid CS pin %d, 0~2 are available\n", cs);
    return false;
  }
  cmd[0] = CH341A_CMD_UIO_STREAM;
  if(enable)
    cmd[1] = CH341A_CMD_UIO_STM_OUT | (0x37 & ~(1 << cs));
  else
    cmd[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
  cmd[2] = CH341A_CMD_UIO_STM_DIR | 0x3F;
  cmd[3] = CH341A_CMD_UIO_STM_END;

  return CH341USBTransferPart(pgm, 0x00, cmd, 4) > 0;
}

static int ch341a_open(PROGRAMMER *pgm, const char *port) {
  if(!pgm->cookie)              // Sanity
    return -1;
  pmsg_debug("%s(\"%s\")\n", __func__, port);
  union pinfo pinfo;
  LNODEID usbpid;
  int rv = -1;

  serdev = &usb_serdev;

  pinfo.usbinfo.vid = CH341A_VID;
  pinfo.usbinfo.pid = CH341A_PID;
  pgm->fd.usb.max_xfer = CH341A_PACKET_LENGTH;
  pgm->fd.usb.rep = 0x82; // IN
  pgm->fd.usb.wep = 0x02; // OUT
  pgm->port = "usb";

  rv = serial_open(port, pinfo, &pgm->fd);

  if(serdev && serdev->usbsn) {
    pgm->usbsn = serdev->usbsn;
  }

  return rv;
}

static void ch341a_close(PROGRAMMER *pgm) {
  pmsg_debug("%s()\n", __func__);

  int cs = intlog2(pgm->pin[PIN_AVR_RESET].mask[0]);

  if(cs < 0)
    cs = 0;

  CH341ChipSelect(pgm, cs, false);

  serial_close(&pgm->fd);
}

static int ch341a_initialize(const PROGRAMMER *pgm, const AVRPART *p) {
  pmsg_trace("ch341a_initialize()\n");

  int cs = intlog2(pgm->pin[PIN_AVR_RESET].mask[0]);

  if(cs < 0)
    cs = 0;

  if(!CH341ChipSelect(pgm, cs, false)) {
    pmsg_error("CH341ChipSelect(..., false) failed\n");
    return -1;
  }
  usleep(20*1000);
  if(!CH341ChipSelect(pgm, cs, true)) {
    pmsg_error("CH341ChipSelect(..., true) failed\n");
    return -1;
  }

  return pgm->program_enable(pgm, p);
}

static int ch341a_spi(const PROGRAMMER *pgm, const unsigned char *in, unsigned char *out, int size) {
  unsigned char pkt[CH341A_PACKET_LENGTH];

  if(!size)
    return 0;

  if(size > CH341A_PACKET_LENGTH - 1)
    size = CH341A_PACKET_LENGTH - 1;

  pkt[0] = CH341A_CMD_SPI_STREAM;

  for(int i = 0; i < size; i++)
    pkt[i + 1] = swap_byte(in[i]);

  if(!CH341USBTransfer(pgm, 0x00, pkt, size + 1)) {
    pmsg_error("failed to transfer data to CH341\n");
    return -1;
  }

  if(!CH341USBTransfer(pgm, 0x80, pkt, size)) {
    pmsg_error("failed to transfer data from CH341\n");
    return -1;
  }

  for(int i = 0; i < size; i++)
    out[i] = swap_byte(pkt[i]);

  return size;
}

static int ch341a_spi_cmd(const PROGRAMMER *pgm, const unsigned char *cmd, unsigned char *res) {
  return pgm->spi(pgm, cmd, res, 4);
}

static int ch341a_spi_chip_erase(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char cmd[4];
  unsigned char res[4];

  if(p->op[AVR_OP_CHIP_ERASE] == NULL) {
    pmsg_error("chip erase instruction not defined for part %s\n", p->desc);
    return -1;
  }
  memset(cmd, 0, sizeof(cmd));
  avr_set_bits(p->op[AVR_OP_CHIP_ERASE], cmd);
  pgm->cmd(pgm, cmd, res);
  usleep(p->chip_erase_delay);
  pgm->initialize(pgm, p);
  return 0;
}

// Fall back on bytewise write (followed by write page if flash)
static int ch341a_spi_paged_write(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int isflash = mem_is_in_flash(m);

  if(n_bytes) {
    if(!isflash && !mem_is_eeprom(m))
      return -2;

    // Always called with addr at page boundary and n_bytes == m->page_size
    for(unsigned int end = addr + n_bytes; addr < end; addr++)
      if(pgm->write_byte(pgm, p, m, addr, m->buf[addr]) < 0)
        return -1;
  }

  if(isflash && avr_write_page(pgm, p, m, addr - n_bytes) < 0)
    return -1;

  return n_bytes;
}

// Fall back on bytewise read
static int ch341a_spi_paged_load(const PROGRAMMER *pgm, const AVRPART *p, const AVRMEM *m,
  unsigned int page_size, unsigned int addr, unsigned int n_bytes) {

  int isflash = mem_is_in_flash(m);

  if(n_bytes) {
    if(!isflash && !mem_is_eeprom(m))
      return -2;

    // Always called with addr at page boundary and n_bytes == m->page_size
    if(isflash && m->op[AVR_OP_LOAD_EXT_ADDR]) {
      unsigned char cmd[4], res[4];

      memset(cmd, 0, sizeof cmd);
      avr_set_bits(m->op[AVR_OP_LOAD_EXT_ADDR], cmd);
      avr_set_addr(m->op[AVR_OP_LOAD_EXT_ADDR], cmd, addr/2);
      if(pgm->cmd(pgm, cmd, res) < 0)
        return -1;
    }

    for(unsigned int end = addr + n_bytes; addr < end; addr++)
      if(pgm->read_byte(pgm, p, m, addr, m->buf + addr) < 0)
        return -1;
  }

  return n_bytes;
}

static int ch341a_spi_program_enable(const PROGRAMMER *pgm, const AVRPART *p) {
  unsigned char res[4];
  unsigned char cmd[4];

  memset(cmd, 0, sizeof(cmd));
  memset(res, 0, sizeof(res));

  cmd[0] = 0;
  pmsg_trace("ch341a_program_enable() %p\n", p->op[AVR_OP_PGM_ENABLE]);

  if(p->op[AVR_OP_PGM_ENABLE] == NULL) {
    pmsg_error("program enable instruction not defined for part %s\n", p->desc);
    return -1;
  }
  avr_set_bits(p->op[AVR_OP_PGM_ENABLE], cmd);
  pgm->cmd(pgm, cmd, res);

  pmsg_debug("%s resp = %02x %02x %02x %02x\n", __func__, res[0], res[1], res[2], res[3]);
  // Check for sync character
  if(res[2] != cmd[1])
    return -2;
  return 0;
}

// Interface management
static void ch341a_setup(PROGRAMMER *pgm) {
  pgm->cookie = mmt_malloc(sizeof(struct pdata));
}

static void ch341a_teardown(PROGRAMMER *pgm) {
  mmt_free(pgm->cookie);
  pgm->cookie = NULL;
}

// Dummy functions
static void ch341a_disable(const PROGRAMMER *pgm) {
  return;
}

static void ch341a_enable(PROGRAMMER *pgm, const AVRPART *p) {
  return;
}

static void ch341a_display(const PROGRAMMER *pgm, const char *p) {
  return;
}

void ch341a_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "ch341a");

  // Mandatory functions
  pgm->initialize = ch341a_initialize;
  pgm->display = ch341a_display;
  pgm->enable = ch341a_enable;
  pgm->disable = ch341a_disable;
  pgm->program_enable = ch341a_spi_program_enable;
  pgm->chip_erase = ch341a_spi_chip_erase;
  pgm->cmd = ch341a_spi_cmd;
  pgm->spi = ch341a_spi;
  pgm->open = ch341a_open;
  pgm->close = ch341a_close;
  pgm->read_byte = avr_read_byte_default;
  pgm->write_byte = avr_write_byte_default;

  // Optional functions
  pgm->paged_write = ch341a_spi_paged_write;
  pgm->paged_load = ch341a_spi_paged_load;
  pgm->setup = ch341a_setup;
  pgm->teardown = ch341a_teardown;
}

// ----------------------------------------------------------------------
#else                           // !defined(HAVE_LIBUSB_1_0)

static int ch341a_nousb_open(PROGRAMMER *pgm, const char *name) {
  pmsg_error("no usb support, please compile again with libusb installed\n");
  return -1;
}

void ch341a_initpgm(PROGRAMMER *pgm) {
  strcpy(pgm->type, "ch341a");
  pgm->open = ch341a_nousb_open;
}
#endif                          // !defined(HAVE_LIBUSB_1_0)

const char ch341a_desc[] = "Chip CH341A: AVR must have min F_CPU of 6.8 MHz";
