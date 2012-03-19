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

#include "mcu_periph/spi.h"

#include "std.h"
#include "LPC21xx.h"
#include "armVIC.h"

// FIXME
// current implementation only works for SPI1 (SSP)

#define SpiTransmit() {                             \
    while (spi_tx_idx < spi_buffer_length                       \
       && bit_is_set(SSPSR, TNF)) {					\
      SpiSend(spi_buffer_output[spi_tx_idx]);                           \
      spi_tx_idx++;                             \
    }                                                           \
    if (spi_tx_idx == spi_buffer_length)                    \
      SpiDisableTxi();                                                  \
}

__attribute__ ((always_inline)) static inline void SpiEnable(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->cr1)), SSE);
}

__attribute__ ((always_inline)) static inline void SpiDisable(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->cr1)), SSE);
}

__attribute__ ((always_inline)) static inline void SpiEnableRti(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->imsc)), RTIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableRti(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->imsc)), RTIM);
}

__attribute__ ((always_inline)) static inline void SpiClearRti(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->icr)), RTIC);
}

__attribute__ ((always_inline)) static inline void SpiEnableTxi(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->imsc)), TXIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableTxi(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->imsc)), TXIM);
}

__attribute__ ((always_inline)) static inline void SpiEnableRxi(struct spi_periph* p) {
  SetBit(((sspRegs_t *)(p->imsc)), RXIM);
}

__attribute__ ((always_inline)) static inline void SpiDisableRxi(struct spi_periph* p) {
  ClearBit(((sspRegs_t *)(p->imsc)), RXIM);
}

__attribute__ ((always_inline)) static inline void SpiSend(struct spi_periph* p, uint8_t c) {
  ((sspRegs_t *)(p->dr)) = c;
}

__attribute__ ((always_inline)) static inline void SpiRead(struct spi_periph* p, uint8_t* c) {
  *c = ((sspRegs_t *)(p->dr));
}

__attribute__ ((always_inline)) static inline void SpiTransmit(struct spi_periph* p, struct spi_transaction* t) {
  while (p->tx_idx_buf < t->length && bit_is_set(((sspRegs_t *)(p->sr)), TNF)) {
    SpiSend(p, t->output_buf[p->tx_idx_buf]);
    p->tx_idx_buf++;
  }
  if (p->tx_idx_buf == t->length) {
    SpiDisableTxi(p);
  }
}

__attribute__ ((always_inline)) static inline void SpiReceive(struct spi_periph* p, struct spi_transaction* t) {
  while (bit_is_set(((sspRegs_t *)(p->sr)), RNE)) {
    if (p->rx_idx_buf < t->length) {
      SpiRead(p, &(t->input_buf[p->rx_idx_buf]));
      p->rx_idx_buf++;
    }
    else {
      uint8_t foo;
      SpiRead(p, &foo);
    }
  }
}

__attribute__ ((always_inline)) static inline void SpiInitBuf(struct spi_periph* p, struct spi_transaction* t) {
  p->rx_idx_buf = 0;
  p->tx_idx_buf = 0;
  // t->status = ??
  SpiTransmit(p,t); // fill fifo
}


__attribute__ ((always_inline)) static inline void SpiStart(struct spi_periph* p, struct spi_transaction* t) {
  SpiEnable(p);
  SpiInitBuf(p,t);
  SpiEnableTxi(p); // enable tx fifo half empty interrupt
}

__attribute__ ((always_inline)) static inline void SpiAutomaton(struct spi_periph* p) {
  struct spi_transaction* trans = p->trans[p->trans_extract_idx];
  if (bit_is_set(((sspRegs_t *)(p->mis)), TXMIS)) {  /*  Tx fifo is half empty */
    SpiTransmit(p, trans);
    SpiReceive(p, trans);
    SpiEnableRti(p);
  }

  if (bit_is_set(((sspRegs_t *)(p->mis)), RTMIS)) { /* Rx fifo is not empty and no receive took place in the last 32 bits period */
    // TODO Handle slave unselect
    //SpiUnselectCurrentSlave();
    SpiReceive(p, trans);
    SpiDisableRti(p);
    SpiClearRti(p);                /* clear interrupt */
    SpiDisable(p);
    // end transaction with success
    trans->status = SPITransSuccess;
    // handle transaction fifo here
    p->trans_extract_idx++;
    if (p->trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
      p->trans_extract_idx = 0;
    // if no more transaction to process, stop here, else start next transaction
    if (p->trans_extract_idx == p->trans_insert_idx) {
      p->status = SPIIdle;
    }
    else {
      SpiStart(p,p->trans[p->trans_extract_idx]);
    }
  }

#ifdef SPI_SLAVE

volatile uint8_t spi_tx_idx;
volatile uint8_t spi_rx_idx;

void SPI1_ISR(void) __attribute__((naked));

/* set SSP input clock, PCLK / CPSDVSR = 468.75kHz */

#if (PCLK == 15000000)
#define CPSDVSR    32
#else

#if (PCLK == 30000000)
#define CPSDVSR    64
#else

#if (PCLK == 60000000)
#define CPSDVSR    128
#else

#error unknown PCLK frequency
#endif
#endif
#endif

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size            : 8 bits   */
#define SSP_FRF  0x00 << 4  /* frame format         : SPI      */
#define SSP_CPOL 0x00 << 6  /* clock polarity       : idle low */
#define SSP_CPHA 0x01 << 7  /* clock phase          : 1        */
#define SSP_SCR  0x0F << 8  /* serial clock rate    : 29.3kHz, SSP input clock / 16 */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode        : disabled */
#define SSP_SSE  0x00 << 1  /* SSP enable           : disabled */
#define SSP_MS   0x01 << 2  /* master slave mode    : slave    */
#define SSP_SOD  0x00 << 3  /* slave output disable : disabled */

void spi_slave_init( void ) {
  /* setup pins for SSP (SCK, MISO, MOSI, SS) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI | PINSEL1_SSEL;

  /* setup SSP  */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = CPSDVSR; /* Prescaler, UM10120_1.pdf page 167 */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   // SPI1 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI1);     // SPI1 interrupt enabled
  VICVectCntl7 = VIC_ENABLE | VIC_SPI1;
  VICVectAddr7 = (uint32_t)SPI1_ISR;    // address of the ISR

  /* enable SPI */
  //  SpiEnable();
}

void SPI1_ISR(void) {
 ISR_ENTRY();

 if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx half empty */
   SpiTransmit();
   SpiReceive();
   SpiEnableRti();
 }

 if ( bit_is_set(SSPMIS, RTMIS)) { /* Rx timeout      */
   SpiReceive();
   SpiClearRti();                  /* clear interrupt */
   SpiDisableRti();
   SpiDisable();
   spi_message_received = TRUE;
 }

 VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
 ISR_EXIT();
}

#endif /* SPI_SLAVE */


/*
 *
 * SPI Master code
 *
 *
 */

#ifdef SPI_MASTER

#include "led.h"  /* FIXME remove that */

#ifdef USE_SPI0

// void spi0_ISR(void) __attribute__((naked));
//
// void spi0_ISR(void) {
//   ISR_ENTRY();
//   VICVectAddr = 0x00000000;
//   ISR_EXIT();
// }

void spi0_hw_init(void) {

  spi0.reg_addr SPI0;

  // TODO set spi0 and interrupt vector
}
#endif


#ifdef USE_SPI1

/** default initial settings */
#ifndef SPI1_VIC_SLOT
#define SPI1_VIC_SLOT 7
#endif

/* SSP (SPI1) pins (UM10120_1.pdf page 76)
   P0.17 SCK    PINSEL1 2 << 2
   P0.18 MISO   PINSEL1 2 << 4
   P0.19 MOSI   PINSEL1 2 << 6
   P0.20 SS     PINSEL1 2 << 8
*/
#define PINSEL1_SCK  (2 << 2)
#define PINSEL1_MISO (2 << 4)
#define PINSEL1_MOSI (2 << 6)
#define PINSEL1_SSEL (2 << 8)

/* SSPCR0 settings */
#define SSP_DSS  0x07 << 0  /* data size         : 8 bits        */
#define SSP_FRF  0x00 << 4  /* frame format      : SPI           */
#define SSP_CPOL 0x00 << 6  /* clock polarity    : SCK idles low */
#define SSP_CPHA 0x01 << 7  /* clock phase       : data captured on second clock transition */
#define SSP_SCR  0x00 << 8  /* serial clock rate   */

/* SSPCR1 settings */
#define SSP_LBM  0x00 << 0  /* loopback mode     : disabled                  */
#define SSP_SSE  0x00 << 1  /* SSP enable        : disabled                  */
#define SSP_MS   0x00 << 2  /* master slave mode : master                    */
#define SSP_SOD  0x00 << 3  /* slave output disable : don't care when master */

#ifndef SSPCPSR_VAL
#define SSPCPSR_VAL 0x20
#endif

void spi1_ISR(void) __attribute__((naked));

void spi1_ISR(void) {
  ISR_ENTRY();

  struct spi_transaction* trans = p->trans[p->trans_extract_idx];
  if (bit_is_set(((sspRegs_t *)(p->mis)), TXMIS)) {  /*  Tx fifo is half empty */
    SpiTransmit();
    SpiReceive();
    SpiEnableRti();
  }

  if (bit_is_set(SSPMIS, RTMIS)) { /* Rx fifo is not empty and no receive took place in the last 32 bits period */
#if !SPI_NO_UNSELECT_SLAVE
    SpiUnselectCurrentSlave();
#endif
    SpiReceive();
    SpiDisableRti();
    SpiClearRti();                /* clear interrupt */
    SpiDisable();
    spi_message_received = TRUE;
  }

  VICVectAddr = 0x00000000;
  ISR_EXIT();
}

void spi1_hw_init(void) {

  spi1.reg_addr SPI1;

  /* setup pins for SSP (SCK, MISO, MOSI) */
  PINSEL1 |= PINSEL1_SCK | PINSEL1_MISO | PINSEL1_MOSI;

  /* setup SSP */
  SSPCR0 = SSP_DSS | SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
  SSPCR1 = SSP_LBM | SSP_MS | SSP_SOD;
  SSPCPSR = SSPCPSR_VAL; /* Prescaler */

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI1);   /* SPI1 selected as IRQ */
  VICIntEnable = VIC_BIT(VIC_SPI1);     /* SPI1 interrupt enabled */
  _VIC_CNTL(SPI1_VIC_SLOT) = VIC_ENABLE | VIC_SPI1;
  _VIC_ADDR(SPI1_VIC_SLOT) = (uint32_t)spi1_ISR;    /* address of the ISR */

  // TODO handle slave pin config
}

#endif


extern bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t) {

  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if (idx == p->trans_extract_idx) {
    t->status = SPITransFailed
    return FALSE; /* queue full */
  }
  t->status = SPITransPending;
  //*(t->ready) = 0; ???
  // Disable interrupts
  int_disable();
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;
  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle) {
    SpiStart(p,p->trans[p->trans_extract_idx]);
  }
  int_enable();

  return TRUE;
}


/* interrupt handler */
void SPI1_ISR(void) __attribute__((naked));

void spi_init( void ) {
#if defined USE_SPI_SLAVE0
  /* setup slave0_select pin */
  SPI_SELECT_SLAVE0_IODIR |= 1 << SPI_SELECT_SLAVE0_PIN;  /* slave0_select is output */
  SpiUnselectSlave0();  /* slave0 is unselected    */
#endif

#if defined USE_SPI_SLAVE1
  /* setup slave1_select pin */
  PINSEL2 &= ~(_BV(3)); /* P1.25-16 are used as GPIO */
  SPI_SELECT_SLAVE1_IODIR |= 1 << SPI_SELECT_SLAVE1_PIN;    /* slave1_select is output   */
  SpiUnselectSlave1();    /* slave1 is unselected      */
#endif
}

void SPI1_ISR(void) {
  ISR_ENTRY();

  if (bit_is_set(SSPMIS, TXMIS)) {  /*  Tx fifo is half empty */
    SpiTransmit();
    SpiReceive();
    SpiEnableRti();
  }

  if (bit_is_set(SSPMIS, RTMIS)) { /* Rx fifo is not empty and no receive took place in the last 32 bits period */
#if !SPI_NO_UNSELECT_SLAVE
    SpiUnselectCurrentSlave();
#endif
    SpiReceive();
    SpiDisableRti();
    SpiClearRti();                /* clear interrupt */
    SpiDisable();
    spi_message_received = TRUE;
  }

  VICVectAddr = 0x00000000; /* clear this interrupt from the VIC */
  ISR_EXIT();
}

#endif /** SPI_MASTER */
