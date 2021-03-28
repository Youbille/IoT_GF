
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "wifi.h"
#include "aio.h"
#include "mcp9700.h"
#include "mqtt.h"
#include "time.h"


/* -- use following constants to define the example mode ----------- */

#define SPI_USED

/* -- includes ----------------------------------------------------- */

#include "bme680.h"

/* -- platform dependent definitions ------------------------------- */



// SPI interface definitions for ESP32
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  18
#define SPI_MOSI_GPIO 23
#define SPI_MISO_GPIO 19
#define SPI_CS_GPIO   15

// I2C interface defintions for ESP32 and ESP8266
// #define I2C_BUS       0
// #define I2C_SCL_PIN   14
// #define I2C_SDA_PIN   13
// #define I2C_FREQ      I2C_FREQ_100K

/*-----------------------*/
#define BLINK_GPIO 2
#define CHANNEL ADC_CHANNEL_6

static bme680_sensor_t* sensor = 0;

void bme_init(){
   spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);
   sensor = bme680_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);
   if (sensor)
    {
      //Here we can tinker with the different configurations for each measurement
    }
    else
        printf("Could not initialize BME680 sensor\n");
}
void app_main(void)
{
   bme680_values_float_t values;
   mcp9700_init(1,CHANNEL);
   wifi_init("LiveEslafi","Fontenay2016");
    mqtt_init("mqtts://iot.devinci.online/","gf171016","RFt4Zqz3");
   bme_init();
   uint32_t duration = bme680_get_measurement_duration(sensor);
   aio_init("Youbi","aio_vSZS826ZIvPK2s1YTkfKdmZpSQdg");
    while(true){
        if (bme680_force_measurement (sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay (duration);

            // get the results and do something with them
            if (bme680_get_results_float (sensor, &values))
                printf("%.3f BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       (double)sdk_system_get_time()*1e-3,
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
        }
    
    char value[11];
    
    sprintf(value,"%f",values.temperature);
    aio_create_data(value, "alex");
    mqtt_publish("gf171016/bme680/temp",value);
    sprintf(value,"%f",values.humidity);
    aio_create_data(value, "alexhumidity");
    mqtt_publish("gf171016/bme680/humidity",value);
    sprintf(value,"%f",values.pressure);
    mqtt_publish("gf171016/bme680/pressure",value);
    sprintf(value,"%f",values.gas_resistance);
    mqtt_publish("gf171016/bme680/air_quality",value);
    sprintf(value,"%d",(mcp9700_get_value()-500)/30);
    mqtt_publish("gf171016/mcp9700/temp",value);

      vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}
