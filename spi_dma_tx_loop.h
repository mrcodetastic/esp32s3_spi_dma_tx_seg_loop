// Code modified from: https://github.com/MCJack123/craftos-esp/tree/master/main/driver
//          Credit to: https://github.com/MCJack123/


#ifndef VGA_H
#define VGA_H

#include <esp_err.h>
#include <esp_log.h>
#include <esp_event.h>

#define CHECK_CALLW(call, msg) if ((err = call) != ESP_OK) {ESP_LOGW(TAG, msg ": %s (%d)", esp_err_to_name(err), err); return 0;}
#define CHECK_CALLE(call, msg) if ((err = call) != ESP_OK) {ESP_LOGE(TAG, msg ": %s (%d)", esp_err_to_name(err), err); return err;}


#ifdef __cplusplus
extern "C"
{
#endif

esp_err_t spi_setup(void);
esp_err_t spi_transfer_loop_start(void);
esp_err_t spi_transfer_loop_stop(void); // generates interrupt

int      spi_get_transfer_count();
bool     spi_seg_transfer_is_complete();
uint32_t get_gpspi2_intr_val();


#ifdef __cplusplus
}
#endif


#endif
