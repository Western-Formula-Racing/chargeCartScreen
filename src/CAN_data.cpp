#include "driver/twai.h"
#include "CAN_data.h"
#include <Arduino.h>

RawCanData can_data = {};

//Use this to calculate the average cell temps. 
#define MAX_THERMISTORS 90


void setup_can() {
    Serial.println("Initializing CAN...");
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_43, GPIO_NUM_44, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed.");
    }
    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started.");
    }
}

QueueHandle_t canQueue;  // define the global queue here


void parse_can_message(twai_message_t* message) {


    uint8_t* data = message->data;

    switch (message->identifier) {

        // === Cell Temps from BO_10X TORCH_MZ_TY ===
        case 0x407 ... 0x41F: {            

            static const uint8_t num_thermistors[5] = {4, 4, 4, 4, 2};
            uint8_t module_index = (message->identifier - 0x407) / 5;
            uint8_t message_index = (message->identifier - 0x407) % 5;

            for (int i = 0; i < num_thermistors[message_index] * 2; i += 2) {
                uint16_t raw = ((uint16_t)data[i + 1] << 8) | data[i];
                float temp = raw * 0.001f;
                uint8_t thermistor_index = (message_index * 4) + (i / 2);
                can_data.moduleData[module_index][1][thermistor_index] = temp;
            }
            break;
        }

        // === Cell Voltages from BO_1006 to BO_1030 ===
        case 0x03EE ... 0x0406: {
            uint8_t* data = message->data;

            uint8_t module_index = (message->identifier - 0x03EE) / 5;
            uint8_t message_index = (message->identifier - 0x03EE) % 5;

            for (int i = 0; i < 8; i += 2) {
                uint16_t raw = ((uint16_t)data[i + 1] << 8) | data[i];
                float voltage = raw * 0.0001f;
                uint8_t cell_index = message_index * 4 + (i / 2);
                can_data.moduleData[module_index][0][cell_index] = voltage;
            }
            break;
        }


        // === Pack Status and Current ==//
        case 0x0420: {
            // PackCurrent: 0|16@1+ (0.1, -3276) => little endian, signed
            int16_t raw_current = (int16_t)((data[1] << 8) | data[0]);
            can_data.Pack_Current = (raw_current * 0.1f) - 3276.0f;

            // PackStatus: 40|8@1+ => byte index 5
            can_data.Pack_Status = data[5];

            break;
        }   

        // == 
        case 0x98FF50E5: {  
        {  
            if (message->extd) 
            {
                    uint16_t raw_voltage = ((uint16_t)data[1] << 8) | data[0];
                    uint16_t raw_current = ((uint16_t)data[3] << 8) | data[2];

                    can_data.Elcon_Output_Voltage = raw_voltage * 0.1f;
                    can_data.Elcon_Output_Current = raw_current * 0.1f;
            }
            break;
        }
    }
}
}