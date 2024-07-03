#include <Arduino.h>
#include <esp_task_wdt.h>
#include "spi_dma_tx_loop.h"
#include <app_constants.hpp>
#include <driver/gpio.h>

extern volatile int transfer_count;

// Start the App
void setup(void)
{
    Serial.begin(112500);
    delay(2000);
    Serial.println("Starting....");
    esp_task_wdt_deinit();


    gpio_reset_pin((gpio_num_t)MBI_LAT);                        // some pins are not in gpio mode after reset => https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#gpio-summary
    gpio_set_direction((gpio_num_t)MBI_LAT, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MBI_LAT, LOW);
    delay(50);
    gpio_set_level((gpio_num_t)MBI_LAT, HIGH);    
    delay(50);
    gpio_set_level((gpio_num_t)MBI_LAT, LOW);    
    delay(50);
    gpio_set_level((gpio_num_t)MBI_LAT, HIGH);    


    // Start GCLK via SPI
    spi_setup();
    delay(10);    

    //log_e("Transfer count: %d", spi_get_transfer_count());
    spi_transfer_loop_start(); // send blank transaction
    //Serial.println(get_gpspi2_intr_val(), BIN);

    spi_transfer_loop_stop(); // send blank transaction
    //log_e("Transfer count: %d", spi_get_transfer_count());
    //Serial.println(get_gpspi2_intr_val(), BIN);
 
    //Serial.println("Now we wait until completion");

    spi_transfer_loop_start(); // send blank transaction
    gpio_set_level((gpio_num_t)MBI_LAT, LOW);

    gpio_set_level((gpio_num_t)MBI_LAT, HIGH);
    //Serial.println(get_gpspi2_intr_val(), BIN);
    spi_transfer_loop_stop(); // send blank transaction   


    //while (!spi_seg_transfer_is_complete());
    gpio_set_level((gpio_num_t)MBI_LAT, LOW);

    Serial.println("Completed");    

    log_e("Transfer count: %d", spi_get_transfer_count());
    

}


void loop() {



}
