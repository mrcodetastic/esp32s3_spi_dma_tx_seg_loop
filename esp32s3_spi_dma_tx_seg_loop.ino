#include <Arduino.h>
#include <esp_task_wdt.h>
#include "spi_dma_tx_loop.h"

extern volatile int transfer_count;

// Start the App
void setup(void)
{
    Serial.begin(112500);
    delay(2);
    Serial.println("Starting....");
    esp_task_wdt_deinit();

    // Start GCLK via SPI
    spi_setup();
    spi_transfer_loop_start(); // send blank transaction
    delay(100);

    spi_transfer_loop_start_2(); // start GCLK + Adress toggling    
    delay(1500);
    spi_transfer_loop_stop();
    delay(1500);  
    spi_transfer_loop_restart();

   // delay(1500);
    //spi_transfer_loop_restart();    

}

void loop() {

  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  if ((currentTime - lastTime) > 1000)
  {
    // Every Second
  //  log_e("Time since last loop: %lu ms", currentTime - lastTime);

    // Serial logging pauses stuff and causes flicker
    log_e("Transfer count: %d", spi_get_transfer_count());

    lastTime = currentTime;

  }


 }
