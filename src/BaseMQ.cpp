/****************************************************************************/
//  Function:       cpp file for BaseMQ
//  Hardware:       MQ2, MQ3, MQ4, MQ5, MQ6, MQ7, MQ8, MQ9, MQ135
//  Arduino IDE:    Arduino 1.8.3
//  Author:         Igor Dementiev
//  Date:           Jan 19, 2018
//  Version:        v1.1
//  by www.amperka.ru
/****************************************************************************/

#include "BaseMQ.h"

BaseMQ::BaseMQ(uint8_t pin) {
  _pin = pin;
}

BaseMQ::BaseMQ(uint8_t pin, uint8_t pinHeater) {
  _pin = pin;
  _pinHeater = pinHeater;
  pinMode(_pinHeater, OUTPUT);
}

// фиксированая калибровка датчика
// при знании сопративления датчика на чистом воздухе
void BaseMQ::calibrate(float ro) {
  _ro = ro;
  _stateCalibrate = true;
}

// калибровка датчика
// считывания показаний сопративление датчика на чистом воздухе
// далее фиксированая калибровка датчика
void BaseMQ::calibrate() {
  float rs = readRs();
  float ro = rs / getRoInCleanAir();
  calibrate(ro);
}

void BaseMQ::calibrate(float t, float h) {
  float rs = readCorrectedRs(t, h);
  float ro = rs / getRoInCleanAir();
  calibrate(ro);
}

/**************************************************************************/
/*!
@brief  Get the correction factor to correct for temperature and humidity
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The calculated correction factor
*/
/**************************************************************************/
float BaseMQ::getCorrectionFactor(float t, float h) const {
    // Linearization of the temperature dependency curve under and above 20 degree C
    // below 20degC: fact = a * t * t - b * t - (h - 33) * d
    // above 20degC: fact = a * t + b * h + c
    // this assumes a linear dependency on humidity
    if(t < 20){
        return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
    } else {
        return CORE * t + CORF * h + CORG;
    }
}

/**************************************************************************/
/*!
@brief  Get the resistance of the sensor, ie. the measurement value corrected
        for temp/hum
@param[in] t  The ambient air temperature
@param[in] h  The relative humidity
@return The corrected sensor resistance kOhm
*/
/**************************************************************************/
float BaseMQ::readCorrectedRs(float t, float h) const {
  return readRs()/getCorrectionFactor(t, h);
}

// включение нагревателя на 100%
void BaseMQ::heaterPwrHigh() {
  digitalWrite(_pinHeater, HIGH);
  _heater = true;
  _prMillis = millis();
}

// включение нагревателья на 20%
void BaseMQ::heaterPwrLow() {
  analogWrite(_pinHeater, 75);
  _heater = true;
  _cooler = true;
  _prMillis = millis();
}

// выключение нагревателя
void BaseMQ::heaterPwrOff() {
  digitalWrite(_pinHeater, LOW);
  _heater = false;
}

// циклическое считывание сопративления датчика
float BaseMQ::readRs() const {
  float rs = 0;
  for (int i = 0; i < MQ_SAMPLE_TIMES; i++) {
    rs += calculateResistance(analogRead(_pin));
    delay(MQ_SAMPLE_INTERVAL);
  }
  rs = rs / MQ_SAMPLE_TIMES;
  return rs;
}


// сопротивление датчика
float BaseMQ::calculateResistance(int sensorADC) const {
  float sensorVoltage = sensorADC * (OPERATING_VOLTAGE / ADC_VALUE_MAX);
  float sensorResistance = (OPERATING_VOLTAGE - sensorVoltage) / sensorVoltage * getRL();
  return sensorResistance;
}

float BaseMQ::readScaled(float a, float b) const {
  float ratio = readRatio();
  return exp((log(ratio) - b) / a);
}

float BaseMQ::readCorrectedScaled(float a, float b, float t, float h) const {
  float ratio = readCorrectedRs(t, h) / getRo();
  return exp((log(ratio) - b) / a);
}

float BaseMQ::readRatio() const {
  return readRs() / getRo();
}

bool BaseMQ::heatingCompleted() const {
  if ((_heater) && (!_cooler) && (millis() - _prMillis > MQ_HEATING_TIME)) {
    return true;
  } else {
    return false;
  }
}

bool BaseMQ::coolanceCompleted() const {
  if ((_heater) && (_cooler) && (millis() - _prMillis > MQ_COOLANCE_TIME)) {
    return true;
  } else {
    return false;
  }
}

void BaseMQ::cycleHeat() {
  _heater = false;
  _cooler = false;
  heaterPwrHigh();
//  Serial.println("Heated sensor");
}

bool BaseMQ::atHeatCycleEnd() {
  if (heatingCompleted()) {
    heaterPwrLow();
//    Serial.println("Cool sensor");
    return false;
  } else if (coolanceCompleted()) {
    heaterPwrOff();
    return true;
  } else {
    return false;
  }
}
