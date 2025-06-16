#ifndef CAN_DATA_H
#define CAN_DATA_H
#include "driver/twai.h"
#include "freertos/queue.h"

extern QueueHandle_t canQueue; 
void parse_can_message(twai_message_t* msg); 


typedef struct {
    float moduleData[5][2][20];
    float Pack_Current;
    int Pack_Status;
    float Elcon_Output_Voltage;
    float Elcon_Output_Current;

} RawCanData;

extern RawCanData can_data;

void setup_can();
bool receive_can_message();

#endif


/*
    All cell temps and voltages
    Highest and lowest temp/voltage
*/






