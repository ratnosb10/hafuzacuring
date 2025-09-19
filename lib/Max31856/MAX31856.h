#ifndef MAX31856_H
#define MAX31856_H

#include <Adafruit_MAX31856.h>
#include <math.h>

class MAX31856Sensor
{
private:
    Adafruit_MAX31856 thermo;
    float kalman_gain = 0.05;
    float kalman_estimate = 0.0;

    static const int movAvgWindow = 50;
    float movAvgBuffer[movAvgWindow];
    int movAvgIndex = 0;

public:
    MAX31856Sensor(uint8_t cs_pin) : thermo(cs_pin) {}
    MAX31856Sensor(uint8_t a, uint8_t b, uint8_t c, uint8_t d) : thermo(a, b, c, d) {}

    bool begin()
    {
        if (!thermo.begin())
            return false;
        thermo.setThermocoupleType(MAX31856_TCTYPE_J);

        float temp = readRawTemperature();
        kalman_estimate = temp;
        kalman_estimate += kalman_gain * (temp - kalman_estimate);
        for (int i = 0; i < movAvgWindow; i++)
        {
            movAvgBuffer[i] = kalman_estimate;
        }

        return true;
    }

    uint8_t checkErrors()
    {
        uint8_t fault = thermo.readFault();

        if (fault)
        {
            Serial.print("FAULT: ");
            if (fault & MAX31856_FAULT_CJRANGE)
                Serial.println("Cold Junction Range Fault");
            if (fault & MAX31856_FAULT_TCRANGE)
                Serial.println("Thermocouple Range Fault");
            if (fault & MAX31856_FAULT_CJHIGH)
                Serial.println("Cold Junction High Fault");
            if (fault & MAX31856_FAULT_CJLOW)
                Serial.println("Cold Junction Low Fault");
            if (fault & MAX31856_FAULT_TCHIGH)
                Serial.println("Thermocouple High Fault");
            if (fault & MAX31856_FAULT_TCLOW)
                Serial.println("Thermocouple Low Fault");
            if (fault & MAX31856_FAULT_OVUV)
                Serial.println("Over/Under Voltage Fault");
            if (fault & MAX31856_FAULT_OPEN)
                Serial.println("Thermocouple Open Circuit");
        }

        return fault; // Return bitmask error
    }

    float readRawTemperature()
    {
        float temp = thermo.readThermocoupleTemperature();
       // Serial.print("Raw:");
       // Serial.print(temp);
        // if (temp > 250.0 || temp < -50.0)
        //     return NAN; // range error
        float suhucj = thermo.readCJTemperature();

       // Serial.print("  CJ:");
       // Serial.println(suhucj);
        if (suhucj > 28)
        {
            float offsetsuhu = suhucj - 26;
            temp -= offsetsuhu;
        }
        return temp;
    }

    float applyKalman(float measurement)
    {
        kalman_estimate += kalman_gain * (measurement - kalman_estimate);
        return kalman_estimate;
    }

    float applyMovingAverage(float input)
    {

        movAvgBuffer[movAvgIndex] = input;
        movAvgIndex = (movAvgIndex + 1) % movAvgWindow;

        float sum = 0;
        for (int i = 0; i < movAvgWindow; i++)
        {
            sum += movAvgBuffer[i];
        }
        return sum / movAvgWindow;
    }

    float getFilteredTemperature()
    {
        float raw = readRawTemperature();
        if (isnan(raw))
            return NAN;

        float kalman = applyKalman(raw);
        return applyMovingAverage(kalman);
    }
    byte cekerrormax(float suhu)
    {
        byte error_ = 0;
        if (suhu == 0 || suhu > 500)
            error_ = 1;
        return 8;
    }
};

#endif
