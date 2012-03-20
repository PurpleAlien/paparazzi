/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2005-2006 Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "std.h"
#include "mcu_periph/spi.h"

#ifdef SPI_MASTER

#ifdef USE_SPI0

extern struct spi_periph spi0;

extern void spi0_init(void) {
  spi_init(&spi0);
  spi0_hw_init();
}

#endif

#ifdef USE_SPI1

extern struct spi_periph spi1;

extern void spi1_init(void) {
  spi_init(&spi1);
  spi1_hw_init();
}

#endif

#ifdef USE_SPI2

extern struct spi_periph spi2;

extern void spi2_init(void) {
  spi_init(&spi2);
  spi2_hw_init();
}

#endif

extern void spi_init(struct spi_periph* p) {
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = SPIIdle;
}

#endif /* SPI_MASTER */

#ifdef SPI_SLAVE

uint8_t* spi_buffer_input;
uint8_t* spi_buffer_output;
uint8_t spi_buffer_length;
volatile bool_t spi_message_received;

#endif

