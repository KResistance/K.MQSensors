#include "MQ135.h"

MQ135::MQ135(uint8_t pin)
  : BaseMQ(pin) {
}

MQ135::MQ135(uint8_t pin, uint8_t pinHeater)
  : BaseMQ(pin, pinHeater) {
}

float MQ135::readCO2() {
  return readScaled(-0.42 , 1.92);
}

float MQ135::readCorrectedCO2(float t, float h) {
  return readCorrectedScaled(-0.42 , 1.92, t, h);
}