#include <Arduino.h>
#include <Wire.h>
#include "AW21036.h"

// Physical connections: I2C SDA = 18, I2C SCL = 19, EN = 5

AW21036 driver;

void setup() {

  Wire.begin(18, 19, AW21036_I2C_FREQ);

  driver.begin(&Wire, AW21036_AD_GND, 5);
  driver.set_global_current(255);
  driver.set_led_rgb_mode(true);  
  driver.set_RGB_for_White_Balance(255, 85, 150);

  driver.set_PWM(0, 255);
  driver.update_PWM();
}

void loop() {

  driver.set_RGB_color(0, 0xFF0000);
  delay(5000);

  driver.set_RGB_color(0, 0x00FF00);
  delay(5000);

  driver.set_RGB_color(0, 0x0000FF);
  delay(5000);
}