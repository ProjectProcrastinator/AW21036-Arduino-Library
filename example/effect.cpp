#include <Arduino.h>
#include <Wire.h>
#include "AW21036.h"

// Physical connections: I2C SDA = 18, I2C SCL = 19, EN = 5
// effect for 4 AW21036 -> 48 RGB-LEDs

// LED driver settings
constexpr uint8_t DRIVERS_COUNT    = 4;
constexpr uint8_t LEDS_PER_DRIVER  = 12;
constexpr uint8_t TOTAL_LEDS       = DRIVERS_COUNT * LEDS_PER_DRIVER; // 48 LEDs
constexpr uint8_t EN_PIN           = 5;
constexpr uint8_t GLOBAL_CURRENT   = 255;

// I2C addresses for AW21036 drivers
const uint8_t addresses[DRIVERS_COUNT] = {
  AW21036_AD_GND,
  AW21036_AD_VDD,
  AW21036_AD_SCL,
  AW21036_AD_SDA
};

// Instantiate drivers
AW21036 drivers[DRIVERS_COUNT];
AW21036 drivers_broadcast;

namespace Palettes {
  // Cyberpunk inspiriert
  const uint32_t CYBERPUNK[] = {
    0xFF2D00,  // Neon Rot
    0x00FF8B,  // Cyan
    0xFF0099,  // Hot Pink
    0x00FFE1,  // Electric Blue
    0xFF8B00   // Neon Orange
  };
  
  // Vaporwave Aesthetik
  const uint32_t VAPORWAVE[] = {
    0xFF58C6,  // Rosa
    0x796AFF,  // Soft Purple
    0x00FFD4,  // Türkis
    0xFF8FB1,  // Soft Pink
    0x7B61FF   // Violet
  };
  
  // Neo Tokyo
  const uint32_t NEOTOKYO[] = {
    0xFF0062,  // Deep Pink
    0x00FF8B,  // Matrix Green
    0x7000FF,  // Electric Purple
    0xFF001A,  // Blood Red
    0x00FFFF   // Bright Cyan
  };
  
  // Outrun/Synthwave
  const uint32_t SYNTHWAVE[] = {
    0xFF0099,  // Hot Pink
    0x00FFFF,  // Cyan
    0xFF00FF,  // Magenta
    0x9900FF,  // Purple
    0xFF3366   // Neon Rose
  };
}

uint32_t interpolateColor(uint32_t color1, uint32_t color2, float ratio) {
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;
  
  uint8_t r = r1 + (r2 - r1) * ratio;
  uint8_t g = g1 + (g2 - g1) * ratio;
  uint8_t b = b1 + (b2 - b1) * ratio;
  
  return (uint32_t)r << 16 | (uint32_t)g << 8 | b;
}

void cyberGlitter(uint16_t duration_seconds = 30, uint32_t color = 0x00FF30, uint32_t next_color = 0x00FF30, uint8_t intensity = 255, bool keep_state = false) {
  static uint8_t drops[DRIVERS_COUNT * 12] = {0};
  uint8_t bright[12];
  uint32_t color_block[12];
  uint32_t start_time = millis();
  uint32_t current_time;
  uint32_t total_duration = duration_seconds * 1000;
  
  while((current_time = millis() - start_time) < total_duration) {
    float transition_ratio = (float)current_time / total_duration;
    uint32_t current_color = interpolateColor(color, next_color, transition_ratio);
    for(uint8_t i = 0; i < DRIVERS_COUNT * 12; i++) {
      if(drops[i] < 50 && random(15) == 0) {
        drops[i] = intensity;
      }
      if(drops[i] > 0) {
        uint8_t fade_speed = random(10, 25);
        drops[i] = drops[i] > fade_speed ? drops[i] - fade_speed : 0;
      }
    }
    // Fülle color_block mit der aktuellen Farbe (für alle Treiber gleich)
    for(uint8_t i = 0; i < 12; i++) color_block[i] = current_color;
    drivers_broadcast.set_RGB_color_Block(0, color_block, 12);
    drivers_broadcast.sync_color_to_broadcast(drivers, DRIVERS_COUNT, 0, 12);
    // PWM individuell pro Treiber
    for(uint8_t driver = 0; driver < DRIVERS_COUNT; driver++) {
      for(uint8_t i = 0; i < 12; i++) bright[i] = drops[driver * 12 + i];
      drivers[driver].set_PWM_Block(0, bright, 12);
      drivers[driver].update_PWM();
    }
    delay(20);
  }
  if (!keep_state) {
    memset(drops, 0, sizeof(drops));
  }
}

void cyberGlitterSequence(const uint32_t* palette, uint8_t palette_size, uint16_t sequence_time = 30) {
  uint16_t time_per_transition = (sequence_time * 1000) / (palette_size - 1);
  time_per_transition /= 1000; // Convert to seconds
  
  // Durchlaufe die Palette
  for(uint8_t i = 0; i < palette_size - 1; i++) {
    cyberGlitter(time_per_transition, palette[i], palette[i + 1], 255, true);
  }
  // Schließe den Kreis
  cyberGlitter(time_per_transition, palette[palette_size - 1], palette[0], 255);
}

void setup() {

  Wire.begin(18, 19, AW21036_I2C_FREQ);

  for (uint8_t i = 0; i < DRIVERS_COUNT; ++i) {
    drivers[i].begin(&Wire, addresses[i], EN_PIN);
    drivers[i].set_global_current(GLOBAL_CURRENT);
    drivers[i].set_led_rgb_mode(true);
  }

  drivers_broadcast.begin_lite(&Wire, AW21036_BROADCAST);
}

void loop() {

  cyberGlitterSequence(Palettes::CYBERPUNK, 5, 30);
  cyberGlitterSequence(Palettes::VAPORWAVE, 5, 30);
  cyberGlitterSequence(Palettes::NEOTOKYO, 5, 30);
  cyberGlitterSequence(Palettes::SYNTHWAVE, 5, 30);
}