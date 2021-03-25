// #include "vma311.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// static vma311_t vma311;

// /* static function prototypes */
// static        void            vma311_send_start_signal();
// static        int             vma311_wait(uint16_t, int);
// static        int             vma311_check_response();
// static inline vma311_status_t vma311_read_byte(uint8_t *);
// static        vma311_status_t vma311_check_crc(uint8_t *);

// /**
//  * Initialize VMA311.
//  *
//  * 1. Reset the GPIO pins
//  *    (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv414gpio_reset_pin10gpio_num_t)
//  * 2. Delay 1 second until the sensor becomes stable
//  */
// void vma311_init(gpio_num_t num)
// {
//     vma311.num = num;
//     vma311.last_read_time = -2000000;
//     /* 1. */
//     /* 2. */
// }

// /**
//  * Get the values send by the VMA311 sensor.
//  * (Page 5 of https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)
//  * Complete:
//  * 1. The field affectation
//  */
// vma311_data_t vma311_get_values()
// {
//     vma311_data_t error_data = {VMA311_TIMEOUT_ERROR, -1, -1, -1, -1};
//     uint8_t data[5] = {0, 0, 0, 0, 0};

//     if (esp_timer_get_time() - 2000000 < vma311.last_read_time)
//     {
//         return vma311.data;
//     }
//     vma311.last_read_time = esp_timer_get_time();
//     vma311_send_start_signal();
//     if (vma311_check_response() == VMA311_TIMEOUT_ERROR)
//     {
//         return error_data;
//     }
//     for (int i = 0; i < 5; ++i)
//     {
//         if (vma311_read_byte(&data[i]))
//         {
//             return error_data;
//         }
//     }
//     if(vma311_check_crc(data) != VMA311_CRC_ERROR)
//     {
//       /* 1. */
//     }
//     return vma311.data;
// }

// /**
//  * Send start signal to VMA311.
//  */
// void vma311_send_start_signal()
// {
//     gpio_set_direction(vma311.num, GPIO_MODE_OUTPUT);
//     gpio_set_level(vma311.num, 0);
//     ets_delay_us(20 * 1000);
//     gpio_set_level(vma311.num, 1);
//     ets_delay_us(40);
// }

// /**
//  * Wait for a number of microseconds at a given level.
//  */
// int vma311_wait(uint16_t us, int level)
// {
//     int us_ticks = 0;

//     while(gpio_get_level(vma311.num) == level)
//     { 
//         if(us_ticks++ > us) 
//         {
//             return VMA311_TIMEOUT_ERROR;
//         }
//         ets_delay_us(1);
//     }
//     return us_ticks;
// }

// /**
//  * Check the response of the VMA311 sensor.
//  * (Page 6 of https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)
//  * Complete:
//  * 1. The duration
//  * 2. The level
//  */
// int vma311_check_response()
// {
//     gpio_set_direction(vma311.num, GPIO_MODE_INPUT);
//     if(vma311_wait(/* 1. */, /* 2. */) == VMA311_TIMEOUT_ERROR)
//     {
//         return VMA311_TIMEOUT_ERROR;
//     }
//     if(vma311_wait(/* 1. */, /* 2. */) == VMA311_TIMEOUT_ERROR) 
//     {
//         return VMA311_TIMEOUT_ERROR;
//     }
//     return VMA311_OK;
// }

// /**
//  * Read 8 bits from the GPIO.
//  * (Page 7 and 8 of https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)
//  * Complete:
//  * 1. The duration
//  * 2. The level
//  * 3. The max duration of a low signal
//  */
// vma311_status_t vma311_read_byte(uint8_t *byte)
// {
//     for (int i = 0; i < 8; ++i)
//     {
//         if(vma311_wait(/* 1. */, /* 2. */) == VMA311_TIMEOUT_ERROR)
//         {
//             return VMA311_TIMEOUT_ERROR;
//         }
//         if(vma311_wait(/* 1. */, /* 2. */) > /* 3. */)
//         {
//             *byte |= (1 << (7 - i));
//         }
//     }
//     return VMA311_OK;
// }

// /**
//  * 
//  * Chech the checksum to verify that data are consistent.
//  * Complete:
//  * 1. Compute the checksum and return VMA311_OK or VMA311_CRC_ERROR
//  * (Page 5 of https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf)
//  */
// vma311_status_t vma311_check_crc(uint8_t data[])
// {
//   /* 1. */
// }