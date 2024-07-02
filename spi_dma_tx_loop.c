/***************************************************************************************
   Abusing the ESP32S3's GPSPI2 for Continuous / Endless Transmit (TX) Loop of
   Octal SPI (OSPI) output

   This enables us not just to be limited to the LCD device with the DMA loop output of
   data to whatever peripheral we want to have connected to continuously receive a
   stream of information.

   Code modified from: https://github.com/MCJack123/craftos-esp/tree/master/main/driver
   Credit to: https://github.com/MCJack123/

 ****************************************************************************************/

#if !defined CONFIG_IDF_TARGET_ESP32S3
#pragma warning Not an ESP32S3! What are you doing?
#endif

// Need this to get debug information to show on GPIO outp
#include <Arduino.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_private/spi_common_internal.h> // need to latest version alpha of esp32-idf for this file
#include <freertos/queue.h>
#include <hal/spi_hal.h>
#include <hal/spi_ll.h>
#include <hal/gdma_ll.h>
#include <soc/lldesc.h>  // for lldesc_get_required_num and LLDESC_MAX_NUM_PER_DESC
#include <soc/gdma_struct.h>
#include <soc/io_mux_reg.h>

// Custom includes
#include "spi_dma_tx_loop.h"
#include "app_constants.hpp"

//#define PERFORM_SPI_TRANS_CHK 1
static const char* const TAG = "gpspi2_spi_dma_tx_loop";

/* 
 * Notes on using SPI & DMA in a continuous loop:
 *
 * 1) Any single SPI transmission must be less than 32kB! If it needs to be larger then have it
 *    split into seperate 23kB transmissions.
 *
 * 2) DMA desciptors can only point to about ~4KB of payload, so need approx. 32/4 (~8) linked
 *    DMA desciptors to send a full 32kB SPI transmission using DMA.
 *
 * 3) If the DMA payload (total length) is ever LESS than what the CONF state tranmission length is
 *    is set to (that being ' spi_seg_conf_1[1] = (sizeof(spi_tx_payload_testchunk) * 8);' for example)
 *    then the SPI peripheral will get pissed off and there will be a Watchdog fault occur.
 *    On the flip side, if the DMA data is > SPI transmission length, all will be ok, but payload will
 *    be truncated.
 *
 * 4) At minimum there needs to be at least TWO DMA descriptors, one for the CONF array and
 *    the remaining for the payload DMA linked-list descriptors.
 *
 */

#include <payload.h>

DMA_ATTR static uint32_t spi_seg_conf_2[4];

DMA_ATTR static lldesc_t* dma_data_lldesc = NULL; // for the links required (to be defined)



/**************************************************************************************/
static const char* const SPI_TAG = "spi_master_custom";

typedef struct spi_device_t spi_device_t;
static spi_device_handle_t spi_device;


/**************************************************************************************/
/* from spi_master.c; Apache 2.0 */
#define spi_dma_ll_tx_reset(dev, chan)  gdma_ll_tx_reset_channel(&GDMA, chan);
#define spi_dma_ll_tx_start(dev, chan, addr) do { \
    gdma_ll_tx_set_desc_addr(&GDMA, chan, (uint32_t)addr); \
    gdma_ll_tx_start(&GDMA, chan); \
  } while (0)


/**************************************************************************************/

volatile int transfer_count = 0;
static IRAM_ATTR void dma_isr3(void* param) {

  transfer_count++;

  spi_ll_clear_intr(&GPSPI2, SPI_LL_INTR_TRANS_DONE); // need this for the transacion to start
  spi_ll_clear_intr(&GPSPI2, SPI_LL_INTR_SEG_DONE); // need this for the transacion to start

}
 


/**************************************************************************************/
// Linked List Stuff

// Step 1
int lldesc_setup_chunk(lldesc_t *dmadesc, const void *data, int len, int offset) {
    int n = offset;
    while (len) {
        int dmachunklen = len;
        if (dmachunklen > LLDESC_MAX_NUM_PER_DESC) {
            dmachunklen = LLDESC_MAX_NUM_PER_DESC;
        }
            dmadesc[n].size = dmachunklen;
            dmadesc[n].length = dmachunklen;

        dmadesc[n].buf = (uint8_t *)data;
        dmadesc[n].eof = 0;
        dmadesc[n].sosf = 0;
        dmadesc[n].owner = 1;
        dmadesc[n].qe.stqe_next = NULL;
        //dmadesc[n].qe.stqe_next = &dmadesc[n + 1];
        len -= dmachunklen;
        data += dmachunklen;

        ESP_LOGD(TAG, "Configured dmadesc at pos %d, memory location %08x, pointing to data of size %d", n, (uintptr_t)&dmadesc[n], (int) dmadesc[n].size);
        n++;
    }

    //ESP_LOGI(TAG, "lldesc_setup_chunk created %d descriptors!", n);

    return n;
}

// Step 2
void lldesc_setup_chain(lldesc_t *dmadesc, int dma_desc_count, bool loop) {

  dma_desc_count--;

  // Link last to first
  dmadesc[dma_desc_count].eof = 1;      

  if ( loop == true) {
    dmadesc[dma_desc_count].qe.stqe_next = &dmadesc[0];     
  }
    
  int n = dma_desc_count;
  while (n > 0)
  {
    dmadesc[n-1].eof = 0;    
    dmadesc[n-1].qe.stqe_next = &dmadesc[n];     

        ESP_LOGD(TAG, "desc %d points to desc %d", n-1, n);          
    
    n--;
  }

   ESP_LOGI(TAG, "lldesc_setup_chain() completed.");

}
/**************************************************************************************/
int spi_get_transfer_count ()
{
  return transfer_count;
}


int dma_lldesc_required = 0; // for CONF dma lldesc 
esp_err_t spi_setup(void)
{
  esp_err_t err;

  // From: https://github.com/MCJack123/craftos-esp/blob/master/main/driver/vga.c
  //
  // The DMA driver can only send 32k at a time, but we need to send 185k. How fix?
  // The concept is to use multiple chained DMA transfers, using the CONF
  // buffer feature to split up the large data buffer into multiple transfers.
  // This is because the maximum size of any single transfer is 32kB (256kb),
  // so by splitting it up into multiple transfers which trigger each other,
  // we can achieve larger and faster transfers.
  // To save on CPU time, we also make the transfer loop back to itself, which
  // makes the framebuffer essentially act as an infinite transfer.
  // Also, to trigger Vsync, we'll have the first and last transfers in the
  // loop trigger an interrupt, which will deassert and assert the Vsync line,
  // respectively.

  // To set this up without rewriting the entire driver and HAL, we'll abuse
  // some internals of the driver to get the raw data we need without having
  // to generate it all manually. We'll start by creating linked lists for the
  // non-first chunks. We'll then initiate a transfer for the first chunk, but
  // in the pre-transfer callback, we'll link the rest of the generated chunks
  // onto the end of the generated transfer list. We'll also set SPI_USER_CONF
  // to enable configure-segmented transfers. After that, we should be good to
  // let it run.

  // NOTES:
  // [Guru Meditation Error: Core  1 panic'ed (Interrupt wdt timeout on CPU1). 
  //  .... will occur when DMA payload < what SPI payload device is expecting. !

  // If there is ANY difference in the spi_seg_conf_X[1] size VS what the DMA engine 
  // sends, there will be a timeout / error with SPI.

  ESP_LOGI(TAG, "Initializing DMA Linked-List Descriptors.");


  // Initialize config words
  spi_seg_conf_2[0]  = 0xA0000000UL;  // set magic value
  /*
  spi_seg_conf_2[0] |= (0b0000000000000000 | ( 1 << 6) | ( 1 << 12) | (1 << 13));  // SPI_MS_DLEN_REG (bit 6) + SPI_DMA_INT_ENA_REG (bit 12) + SPI_DMA_INT_CLR_REG (bit 13)
  
  // SPI_MS_DLEN_REG
  spi_seg_conf_2[1] = (sizeof(spi_tx_payload_testchunk2) * 8) ; // must match exactly the dma payload total chunk size -1

  // SPI_DMA_INT_ENA_REG  
  spi_seg_conf_2[2] = 0 | SPI_TRANS_DONE_INT_ENA | SPI_DMA_SEG_TRANS_DONE_INT_ENA | SPI_SEG_MAGIC_ERR_INT_ENA;

  // SPI_DMA_INT_CLR_REG
  spi_seg_conf_2[3] = 0 | SPI_TRANS_DONE_INT_CLR | SPI_DMA_SEG_TRANS_DONE_INT_CLR | SPI_SEG_MAGIC_ERR_INT_CLR; 
  */

  // Set up linked lists for next descriptors
  // lldesc_setup_link is in soc/lldesc.c

  dma_lldesc_required = 1; // for CONF dma lldesc 
  dma_lldesc_required += lldesc_get_required_num(sizeof(spi_tx_payload_testchunk2)); 
  ESP_LOGI(TAG, "%d SPI DMA descriptors required for cover spi_tx_payload_chunk2 data.", dma_lldesc_required);   

  // Allocate memory
  dma_data_lldesc = (lldesc_t*) heap_caps_malloc(sizeof(lldesc_t) * dma_lldesc_required, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);  

  // Future note: Each SPI segment must be < 32kB.
  lldesc_setup_chunk(dma_data_lldesc, &spi_seg_conf_2, 4*1, 0); // setup dma link list descriptor for CONF data
  lldesc_setup_chunk(dma_data_lldesc, &spi_tx_payload_testchunk2, sizeof(spi_tx_payload_testchunk2), 1); // setup dma link list descriptors for payload

  lldesc_setup_chain(dma_data_lldesc, dma_lldesc_required, true); // link them all together



  // If the payload keeps looping on itself without an appropriate header, it'll call a watchdog reset

  // Initialize SPI host
  ESP_LOGD(TAG, "Initializing SPI bus");
  
  spi_bus_config_t host_conf;
  // ensure GND is connected on the logic analyser!!
  // Don't use MOSI as it has a differnt value
  host_conf.data0_io_num = MBI_SRCLK; // MOSI / Not used (for debug)
  host_conf.data1_io_num = -1; // MISO / Not Used
  host_conf.data2_io_num = MBI_GCLK; // gclk
  host_conf.data3_io_num = ADDR_A_PIN; // a
  host_conf.data4_io_num = ADDR_B_PIN; // b
  host_conf.data5_io_num = ADDR_C_PIN; // c
  host_conf.data6_io_num = ADDR_D_PIN; // d 
  host_conf.data7_io_num = ADDR_E_PIN; // e
  host_conf.sclk_io_num  = -1; // clock not used  (therefore don't have SPICOMMON_BUSFLAG_SCLK below )
  host_conf.max_transfer_sz = 32768; //32768 is the max for S3
  host_conf.flags = SPICOMMON_BUSFLAG_OCTAL | SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_MASTER;
  host_conf.intr_flags = 0;
  host_conf.isr_cpu_id = INTR_CPU_ID_AUTO;

  CHECK_CALLE(spi_bus_initialize(SPI2_HOST, &host_conf, SPI_DMA_CH_AUTO), "Could not initialize SPI bus");

  // Initialize device
  ESP_LOGD(TAG, "Initializing SPI device");
  spi_device_interface_config_t device_conf;
  device_conf.command_bits  = 0;
  device_conf.address_bits  = 0;
  device_conf.dummy_bits    = 0;
  device_conf.mode          = 0;
  device_conf.clock_source    = SPI_CLK_SRC_DEFAULT;

  // Set the GCLK Frequency
  // Note: The frequency of GCLK must be higher than 20% of DCLK to get the correct gray scale data.  
  //device_conf.clock_speed_hz  = SPI_MASTER_FREQ_8M/2; // 4Mhz
  device_conf.clock_speed_hz  = 8 * 1000 * 1000; // 3Mhz
  
  device_conf.duty_cycle_pos  = 0;
  device_conf.cs_ena_pretrans = device_conf.cs_ena_posttrans = 0;
  device_conf.spics_io_num  = -1; // not used
  device_conf.flags         = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
  device_conf.queue_size    = 1;
  device_conf.pre_cb        = NULL;
  device_conf.post_cb       = NULL;
  CHECK_CALLE(spi_bus_add_device(SPI2_HOST, &device_conf, &spi_device), "Could not initialize SPI device");

  ESP_LOGD(TAG, "spi_setup() complete");  

  return err;

}

// Setup interrupt handler
  
esp_err_t spi_transfer_loop_start()
{

  // Setup interrupt
  static intr_handle_t intr;
  esp_intr_alloc        (ETS_SPI2_DMA_INTR_SOURCE,  ESP_INTR_FLAG_SHARED, dma_isr3, NULL, &intr);
  esp_intr_alloc        (ETS_SPI2_INTR_SOURCE,      ESP_INTR_FLAG_SHARED, dma_isr3, NULL, &intr);  
  esp_intr_enable       (intr);

  dma_isr3(0); // increase count to 1


  esp_err_t err;

  // Dispatch transaction; link_trans will finish it off
  ESP_LOGD(TAG, "Initial Transfer");

  static spi_transaction_t trans;
  trans.flags     = SPI_TRANS_MODE_OCT;

  // What do we do here? TO DO, send null packet just to kick off transfer?
  // https://www.esp32.com/viewtopic.php?p=123221#p123211
  trans.tx_buffer = spi_tx_payload_testchunk2; 
  trans.length = (sizeof(spi_tx_payload_testchunk2)) * 8;
  trans.rx_buffer = NULL; 
  trans.rxlength = 0;

  CHECK_CALLE(spi_device_polling_start(spi_device, &trans, portMAX_DELAY), "Could not start SPI transfer");

  //esp_register_shutdown_handler(vga_deinit);
  ESP_LOGD(TAG, "spi_transfer_loop_start() complete");

  return ESP_OK;
}


esp_err_t spi_transfer_loop_start_2()
{
  // Need to do first start first to set registers.
  ESP_LOGD(TAG, "Part 2");

  // TODO; Figure out a way to get interrupts working, and when the GCLK segement has completed and about to restert - then allow a DCLK / Greyscale transfer
  //       AT THE SAME TIME
 // esp_intr_alloc_intrstatus(ETS_SPI2_INTR_SOURCE, ESP_INTR_FLAG_SHARED, (uint32_t)&GPSPI2.dma_int_st.val, SPI_DMA_SEG_TRANS_DONE_INT_ENA_M, dma_isr2,  NULL, &intr); 

  GPSPI2.slave.dma_seg_magic_value = 0xA;

  const spi_bus_attr_t* bus_attr = spi_bus_get_attr(SPI2_HOST); 

  // hal->dmadesc_tx = &dma_data_lldesc[0]; // START OFF A DESCRIPTOR 0
  spi_dma_ll_tx_reset(&GDMA, bus_attr->tx_dma_chan);
  spi_ll_dma_tx_fifo_reset(&GPSPI2);
  spi_ll_outfifo_empty_clr(&GPSPI2);
  spi_ll_dma_tx_enable(&GPSPI2, 1);
  spi_dma_ll_tx_start(&GDMA, bus_attr->tx_dma_chan, &dma_data_lldesc[0]);  



  GPSPI2.slave.usr_conf = 1;  // Enable user conf for segmented transfers 
  GPSPI2.user.usr_conf_nxt = 1;  
  
  GPSPI2.misc.cs0_dis = 1;    // turn off client select pin

  // 13. Set SPI_DMA_SEG_TRANS_DONE_INT_ENA to enable the SPI_DMA_SEG_TRANS_DONE_INT interrupt.
  //     Configure other interrupts if needed according to Section 30.10.
  //esp_intr_free(spi_device->host->intr);
  spi_ll_clear_intr(&GPSPI2, SPI_LL_INTR_TRANS_DONE); // need this for the transacion to start
  spi_ll_clear_intr(&GPSPI2, SPI_LL_INTR_SEG_DONE); // need this for the transacion to start

  GPSPI2.dma_int_ena.dma_seg_trans_done = 1; // Doesn't work, so can't use it. :-(
  GPSPI2.dma_int_ena.trans_done = 1; // Doesn't work, so can't use it. :-(
    
  GPSPI2.cmd.conf_bitlen = 0;

  // Configure interrupt
  // TODO; Figure out a way to get interrupts working, and when the GCLK segement has completed and about to restert - then allow a DCLK / Greyscale transfer
  //       AT THE SAME TIME
 // esp_intr_alloc_intrstatus(ETS_SPI2_INTR_SOURCE, ESP_INTR_FLAG_SHARED, (uint32_t)&GPSPI2.dma_int_st.val, SPI_DMA_SEG_TRANS_DONE_INT_ENA_M, dma_isr2,  NULL, &intr); 

  //esp_intr_free(spi_device->host->intr);
  //spi_ll_clear_int_stat (spi_device->host->hal.hw);


  GPSPI2.cmd.update = 1;
  while (GPSPI2.cmd.update);    //waiting config applied  

  // start trans
  GPSPI2.cmd.usr = 1;  

  //esp_register_shutdown_handler(vga_deinit);
  ESP_LOGD(TAG, "spi_transfer_loop_start_2() complete");

  return ESP_OK;
}







// Doesn't quite clean up cleanly.
esp_err_t spi_transfer_loop_restart(void) {

  const spi_bus_attr_t* bus_attr = spi_bus_get_attr(SPI2_HOST); 

  ESP_LOGD(TAG, "Allocated DMA Channel is %d", bus_attr->tx_dma_chan);

  dma_data_lldesc[dma_lldesc_required-1].eof = 0;
  dma_data_lldesc[dma_lldesc_required-1].qe.stqe_next = &dma_data_lldesc[0];  

  gdma_ll_tx_start(&GDMA, bus_attr->tx_dma_chan);

return ESP_OK;

}


// Doesn't quite clean up cleanly.
esp_err_t spi_transfer_loop_stop(void) {

  const spi_bus_attr_t* bus_attr = spi_bus_get_attr(SPI2_HOST);   

  // Disconnect the looping DMA transfer to cause the transfer to finish
  ESP_LOGD(TAG, "Calling spi_transfer_loop_stop()");  

  // Point to end, must ensure we give the SPI perip all the data it expects, so only
  // set the chain to null at the end of the payload dma descriptors.
  // If we don't do this, and don't send all the payload in DMA memory to the SPI device
  // we'll get a CPU fault / watchdog reset.

  //spi_seg_conf_2[0] = 0;

  dma_data_lldesc[dma_lldesc_required-1].eof = 1;
  dma_data_lldesc[dma_lldesc_required-1].qe.stqe_next = NULL;

  return ESP_OK;
}
