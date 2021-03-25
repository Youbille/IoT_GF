/*
 * BSD 2-Clause License
 * 
 * Copyright (c) 2021, ESILV
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include "mcp9700.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"


static esp_adc_cal_characteristics_t *adc_chars;
static mcp9700_t mcp9700;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;


/**
 * Initialize the ADC to use it with the MCP9700 sensor.
 *
 * 1. Set fields of the static mcp9700 structure to be able to access them
 *    in other function of this library.
 * 2. Configure the ADC: its width and its attenuation.
 *    (https://github.com/espressif/esp-idf/blob/2bfdd036b2dbd07004c8e4f2ffc87c728819b737/examples/peripherals/adc/main/adc1_example_main.c)
 * 3. Get the characteristics of the ADC stored in the eFUSE.
 */
void mcp9700_init(adc_unit_t unit,adc_channel_t channel)
{
    /* 1. */
    if (mcp9700.unit == ADC_UNIT_1)
    {
       adc1_config_width(width);//width and attenuation to be found
        adc1_config_channel_atten(channel, atten);
    }
    else if (mcp9700.unit == ADC_UNIT_2) 
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t)); //the calloc function allocates a place in memory
    // with all bits equal to 0, here we indicate to the pointer adc_chars, the adress of this place in memory
    esp_adc_cal_characterize(unit, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, adc_chars); //this function stores the ADC
    //voltage curve based on the characterisitics (parameters of the function) of the ADC, at the adress pointed by adc_chars
    mcp9700.unit = unit;//we assign the parameter values to the mcp9700 structure values
    mcp9700.channel = channel;
    mcp9700.adc_chars = *adc_chars;

    /* 3. */
}

/**
 * Get the temperature value measured by the MCP9700 by applying multi-sampling.
 *
 * 1. Sum the values converted by the ADC.
 * 2. Average these value to get a only one sample.
 *    (https://github.com/espressif/esp-idf/blob/2bfdd036b2dbd07004c8e4f2ffc87c728819b737/examples/peripherals/adc/main/adc1_example_main.c)
 * 3. Convert the voltage using the ESP dedicated function.
 *    (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#_CPPv426esp_adc_cal_raw_to_voltage8uint32_tPK29esp_adc_cal_characteristics_t)
 * 4. Compute and return the temperature value.
 *    (Page 8 of https://ww1.microchip.com/downloads/en/DeviceDoc/20001942G.pdf)
 */
int32_t mcp9700_get_value()
{
    uint32_t adc_reading = 0;
    uint32_t voltage;

    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (mcp9700.unit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        else /* mcp9700.unit == ADC_UNIT_2 */
        {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
    }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
        return(voltage);
        
}