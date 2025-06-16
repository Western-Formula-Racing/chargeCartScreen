#ifndef DATA_H
#define DATA_H

class Data
{
public:
    static float getCellVoltage(int module, int cell);
    static float getCellTemp(int module, int sensor);

    static float getMaxCellTemp();
    static float getMinCellTemp();

    static float getMaxCellVoltage();
    static float getMinCellVoltage();


    static const char* getPackStatus();
    int getPackStatusCode();
    static int getPackCurrent();

    static int getElconVoltage();
    static int getElconCurrent();



};

#endif
