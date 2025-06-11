#include <Arduino.h>
#include <Wire.h>
#include "AW21036.h"

// Physical connections: I2C SDA = 18, I2C SCL = 19, EN = 5

AW21036 driver;

void setup() {

  Serial.begin(115200);
  Wire.begin(18, 19, AW21036_I2C_FREQ);

  Serial.println("AW21036 Minimal Example starts...");
  driver.begin(&Wire, AW21036_AD_GND, 5);
  driver.set_global_current(255);
  driver.set_led_rgb_mode(true);  
  driver.set_RGB_for_White_Balance(255, 85, 150);

  uint8_t brightness_array[12] = {10, 20, 45, 70, 95, 120, 145, 170, 190, 210, 230, 255};
  driver.set_PWM_Block(0, brightness_array, 12);
  driver.update_PWM();
}

void loop() {

  uint32_t color_array[12];
  
  for(uint8_t i = 0; i < 12; i++) color_array[i] = 0xFF0000;
  driver.set_RGB_color_Block(0, color_array, 12);
  delay(5000);

  for(uint8_t i = 0; i < 12; i++) color_array[i] = 0x00FF00;
  driver.set_RGB_color_Block(0, color_array, 12);
  delay(5000);

  for(uint8_t i = 0; i < 12; i++) color_array[i] = 0x0000FF;
  driver.set_RGB_color_Block(0, color_array, 12);
  delay(5000);
}