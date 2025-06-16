#include "Data.h"
#include "CAN_data.h"
#include <limits> // for FLT_MAX, FLT_MIN


const char* Data::getPackStatus()
{
    int status = can_data.Pack_Status;
    switch (status)
    {
        case 0: return "IDLE";
        case 1: return "PRECHRG START";
        case 2: return "PRECHARGING";
        case 3: return "ACTIVE";
        case 4: return "CHARGING";
        case 5: return "CHRG COMPLETE";
        case 6: return "FAULT";
        default: return "UNKNOWN STATUS";
    }
}

int Data::getPackStatusCode()
{
    return can_data.Pack_Status;
}

float Data::getCellTemp(int module, int index) 
{
    if (module < 0 || module >= 5 || index < 0 || index >= 18) return -1;
    return can_data.moduleData[module][1][index];
}

float Data::getCellVoltage(int module, int index) 
{
    if (module < 0 || module >= 5 || index < 0 || index >= 20) return -1;
    return can_data.moduleData[module][0][index];
}


float Data::getMaxCellTemp() {
    float max_temp = -std::numeric_limits<float>::infinity();
    for (int mod = 0; mod < 5; ++mod) {
        for (int i = 0; i < 18; ++i) {
            float val = can_data.moduleData[mod][1][i];
            if (val > max_temp) max_temp = val;
        }
    }
    return max_temp;
}

float Data::getMinCellTemp() {
    float min_temp = std::numeric_limits<float>::infinity();
    for (int mod = 0; mod < 5; ++mod) {
        for (int i = 0; i < 18; ++i) {
            float val = can_data.moduleData[mod][1][i];
            if (val < min_temp) min_temp = val;
        }
    }
    return min_temp;
}

float Data::getMaxCellVoltage() {
    float max_voltage = -std::numeric_limits<float>::infinity();
    for (int mod = 0; mod < 5; ++mod) {
        for (int i = 0; i < 20; ++i) {
            float val = can_data.moduleData[mod][0][i];
            if (val > max_voltage) max_voltage = val;
        }
    }
    return max_voltage;
}

float Data::getMinCellVoltage() {
    float min_voltage = std::numeric_limits<float>::infinity();
    for (int mod = 0; mod < 5; ++mod) {
        for (int i = 0; i < 20; ++i) {
            float val = can_data.moduleData[mod][0][i];
            if (val < min_voltage) min_voltage = val;
        }
    }
    return min_voltage;
}

int Data::getElconVoltage() {
    return static_cast<int>(can_data.Elcon_Output_Voltage);
}

int Data::getElconCurrent() {
    return static_cast<int>(can_data.Elcon_Output_Current);
}
