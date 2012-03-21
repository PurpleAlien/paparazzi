
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/f1/dma.h>

#include "mcu_periph/spi.h"

struct spi_transaction* slave0;

// SPI2 Slave Selection

#define SPI2_SLAVE0_PORT  GPIOB
#define SPI2_SLAVE0_PIN   GPIO12

#define SPI2_SLAVE1_PORT  GPIOB
#define SPI2_SLAVE1_PIN   GPIO5

#define SPI2_SLAVE2_PORT  GPIOB
#define SPI2_SLAVE2_PIN   GPIO3

static inline void Spi2SlaveUnselect(uint8_t slave)
{
  switch(slave) {
    case 0:
      GPIO_BSRR(SPI2_SLAVE0_PORT) = SPI2_SLAVE0_PIN;
      break;
#if USE_SPI2_SLAVE1
    case 1:
      GPIO_BSRR(SPI2_SLAVE1_PORT) = SPI2_SLAVE1_PIN;
      break;
#endif //USE_SPI2_SLAVE1
#if USE_SPI2_SLAVE2
    case 2:
      GPIO_BSRR(SPI2_SLAVE2_PORT) = SPI2_SLAVE2_PIN;
      break;
#endif //USE_SPI2_SLAVE2

    default:
      break;
  }
}


static inline void Spi2SlaveSelect(uint8_t slave)
{
  switch(slave) {
    case 0:
      GPIO_BRR(SPI2_SLAVE0_PORT) = SPI2_SLAVE0_PIN;
      break;
#if USE_SPI2_SLAVE1
    case 1:
      GPIO_BRR(SPI2_SLAVE1_PORT) = SPI2_SLAVE1_PIN;
      break;
#endif //USE_SPI2_SLAVE1
#if USE_SPI2_SLAVE2
    case 2:
      GPIO_BRR(SPI2_SLAVE2_PORT) = SPI2_SLAVE2_PIN;
      break;
#endif //USE_SPI2_SLAVE2
    default:
      break;
  }
}

// spi dma end of rx handler
// XXX: should be provided by libopencm3?
void dma1_channel4_isr(void);

void spi_arch_int_enable(void) {

  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void spi_arch_int_disable(void) {

  // Enable DMA1 channel4 IRQ Channel ( SPI RX)
  nvic_disable_irq(NVIC_DMA1_CHANNEL4_IRQ);
}

void spi_init(void) {

  // Enable SPI2 Periph and gpio clocks -------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

  // Configure GPIOs: SCK, MISO and MOSI  --------------------------------
  gpio_set_mode(GPIO_BANK_SPI2_SCK, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_SPI2_SCK |
	                                        GPIO_SPI2_MISO |
	                                        GPIO_SPI2_MOSI);

  // reset SPI
  spi_reset(SPI2);

  // Disable SPI peripheral
  spi_disable(SPI2);

  // configure SPI
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  // Enable SPI2 periph.
  spi_enable(SPI2);

  // Enable SPI_2 DMA clock ---------------------------------------------------
  rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_DMA1EN);

  // SLAVE 0
  // set accel slave select as output and assert it ( on PB12)
  Spi2SlaveUnselect(0);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, SPI2_SLAVE0_PIN);

  // SLAVE 1
  Spi2SlaveUnselect(1);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, SPI2_SLAVE1_PIN);

  // SLAVE 2
  Spi2SlaveUnselect(1);
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
	        GPIO_CNF_OUTPUT_PUSHPULL, SPI2_SLAVE2_PIN);
//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //Slave2 is on JTDO pin, so disable JTAG DP


  spi2.trans_insert_idx = 0;
  spi2.trans_extract_idx = 0;
  spi2.status = SPIIdle;

  spi_arch_int_enable();
}


void spi_rw(struct spi_transaction  * _trans)
{
  // Store local copy to notify of the results
  slave0 = _trans;
  slave0->status = SPITransRunning;
  spi2.status = SPIRunning;
  Spi2SlaveSelect(slave0->slave_idx);


  // SPI2_Rx_DMA_Channel configuration ------------------------------------

  dma_channel_reset(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)slave0->miso_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, slave0->length);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL4);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL4);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL4, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);

  // SPI2_Tx_DMA_Channel configuration ------------------------------------
  dma_channel_reset(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (u32)&SPI2_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)slave0->mosi_buf);
  dma_set_number_of_data(DMA1, DMA_CHANNEL5, slave0->length);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL5);
  //dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL5);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
  //dma_set_mode(DMA1, DMA_CHANNEL5, DMA_???_NORMAL);
  dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_MEDIUM);

  // Enable DMA1 Channel4
  dma_enable_channel(DMA1, DMA_CHANNEL4);
  // Enable SPI_2 Rx request
  spi_enable_rx_dma(SPI2);

  // Enable DMA1 Channel5
  dma_enable_channel(DMA1, DMA_CHANNEL5);
  // Enable SPI_2 Tx request
  spi_enable_tx_dma(SPI2);

  // Enable DMA1 Channel4 Transfer Complete interrupt
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

}

bool_t spi_submit(struct spi_periph* p, struct spi_transaction* t)
{
  uint8_t idx;
  idx = p->trans_insert_idx + 1;
  if (idx >= SPI_TRANSACTION_QUEUE_LEN) idx = 0;
  if (idx == p->trans_extract_idx) {
    t->status = SPITransFailed;
    return FALSE; /* queue full */
  }
  t->status = SPITransPending;
  // FIXME: still needed?
  //*(t->ready) = 0;
  //Disable interrupts to avoid race conflict with end of DMA transfer interrupt
  __disable_irq();
  p->trans[p->trans_insert_idx] = t;
  p->trans_insert_idx = idx;

  /* if peripheral is idle, start the transaction */
  if (p->status == SPIIdle) {
    spi_rw(p->trans[p->trans_extract_idx]);
  }
  __enable_irq();
  return TRUE;
}


// Accel end of DMA transferred
void dma1_channel4_isr(void)
{

  Spi2SlaveUnselect(spi2.trans[spi2.trans_extract_idx]->slave_idx);

  if ((DMA1_ISR & DMA_ISR_TCIF4) != 0) {
    // clear int pending bit
    DMA1_IFCR |= DMA_IFCR_CTCIF4;

    // mark as available
    spi_message_received = TRUE;
  }
  // disable DMA Channel
  dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

  // Disable SPI_2 Rx and TX request
  spi_disable_rx_dma(SPI2);
  spi_disable_tx_dma(SPI2);

  // Disable DMA1 Channel4 and 5
  dma_disable_channel(DMA1, DMA_CHANNEL4);
  dma_disable_channel(DMA1, DMA_CHANNEL5);

  slave0->status = SPITransSuccess;
  *(slave0->ready) = 1;
  spi2.trans_extract_idx++;

  // Check if there is another pending SPI transaction
  if (spi2.trans_extract_idx >= SPI_TRANSACTION_QUEUE_LEN)
    spi2.trans_extract_idx = 0;
  if (spi2.trans_extract_idx == spi2.trans_insert_idx)
    spi2.status = SPIIdle;
  else
    spi_rw(spi2.trans[spi2.trans_extract_idx]);
}



