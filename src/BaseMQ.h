/****************************************************************************/
//  Function:       Header file for BaseMQ
//  Hardware:       MQ2, MQ3, MQ4, MQ5, MQ6, MQ7, MQ8, MQ9, MQ135
//  Arduino IDE:    Arduino 1.8.3
//  Author:         Igor Dementiev
//  Date:           Jan 19, 2018
//  Version:        v1.1
//  by www.amperka.ru
/****************************************************************************/

#ifndef BaseMQ_H
#define BaseMQ_H
#include <Arduino.h>
// кол-во считываний значений в цикле
#define MQ_SAMPLE_TIMES     5
// задержка после каждого считывания датчика
#define MQ_SAMPLE_INTERVAL  20
// время нагрева датчика
#define MQ_HEATING_TIME     6000
// время охлаждение датчика
#define MQ_COOLANCE_TIME    9000
// разрядность АЦП
#define ADC_BIT             10
// масимальное значение АЦП
#define ADC_VALUE_MAX       pow(2, ADC_BIT)

/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018
#define CORE -0.003333333
#define CORF -0.001923077
#define CORG 1.130128205

#if defined(ARDUINO_ARCH_ESP32)
#define analogWrite ledcWrite
#endif

#if defined(__AVR__)
#define OPERATING_VOLTAGE   5.0

#else
#define OPERATING_VOLTAGE   3.3
#endif

class BaseMQ {
public:
    BaseMQ(uint8_t pin);
    BaseMQ(uint8_t pin, uint8_t pinHeater);
    void calibrate();
    void calibrate(float ro);
    void heaterPwrHigh();
    void heaterPwrLow();
    void heaterPwrOff();
    void cycleHeat();
    bool atHeatCycleEnd();
    bool heatingCompleted() const;
    bool coolanceCompleted() const;
    float readRatio() const;
    inline bool isCalibrated() const { return _stateCalibrate; };
    inline float getRo() const { return _ro; };

protected:
    float readScaled(float a, float b) const;
    float getCorrectionFactor(float t, float h) const;
    virtual float getRoInCleanAir() const = 0;
    virtual float getRL() const = 0;

private:
    bool _heater = false;
    bool _cooler = false;
    bool _stateCalibrate = false;
    unsigned long _prMillis = 0;
    float _ro = 1.0f;
    uint8_t _pin;
    uint8_t _pinHeater;
    float readRs() const;
    float calculateResistance(int sensorADC) const;
};

#endif 
