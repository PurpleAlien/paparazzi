/*  $Id$
 *
 * Copyright (C) 2003-2005  Pascal Brisset, Antoine Drouin
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

/** \brief handling of arm7 SPI hardware
 *  for now only SPI1 ( aka SSP )
 */

#ifndef SPI_ARCH_H
#define SPI_ARCH_H

#include "std.h"
#include "LPC21xx.h"
#include BOARD_CONFIG

#ifdef USE_SPI0

extern void spi0_hw_init(void);

#endif /* USE_SPI0 */

// SSP is on SPI1 on lpc
#if defined USE_SSP & !defined USE_SPI1
#define USE_SP11 1
// TODO other defines ?
#endif

#ifdef USE_SPI1

extern void spi1_hw_init(void);

#endif /* USE_SPI1 */


#ifdef SPI_SLAVE

extern volatile uint8_t spi_tx_idx;
extern volatile uint8_t spi_rx_idx;

#endif /* SPI_SLAVE */



#ifdef SPI_MASTER


#endif /* SPI_MASTER */


#endif /* SPI_ARCH_H */
