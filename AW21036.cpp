#include "AW21036.h"

/**
 * @brief Instantiates the AW21036 class object.
 */
AW21036::AW21036()  {}

/**
 * @brief Initializes the AW21036 with I2C bus, address, and enable pin.
 *
 * @param i2c_bus The I2C bus to communicate with the AW21036.
 * @param address The I2C address of the AW21036.
 * @param enablePin The EN pin of the IC, HIGH = enabled.
 */
void AW21036::begin(TwoWire* i2c_bus, uint8_t address, uint8_t enablePin) {

  _enable_pin = enablePin;
  pinMode(_enable_pin, OUTPUT);
  hard_enable();

  if (address == AW21036_AD_GND || address == AW21036_AD_VDD || address == AW21036_AD_SCL || address == AW21036_AD_SDA || address == AW21036_BROADCAST)
    {
      _i2c_address = address;
    }

  _i2c_dev = new Adafruit_I2CDevice(_i2c_address, i2c_bus);
  _i2c_dev->begin();

  reset();
  soft_enable();
  if (_MHPA0808RGBDT == true)     set_RGB_for_White_Balance(255, 85, 150);
}

/**
 * @brief Initializes the AW21036 with I2C bus and address only.
 *
 * @param i2c_bus The I2C bus to communicate with the AW21036.
 * @param address The I2C address of the AW21036.
 */
void AW21036::begin_lite(TwoWire* i2c_bus, uint8_t address) {

  if (address == AW21036_AD_GND || address == AW21036_AD_VDD || address == AW21036_AD_SCL || address == AW21036_AD_SDA || address == AW21036_BROADCAST)
    {
      _i2c_address = address;
    }

  _i2c_dev = new Adafruit_I2CDevice(_i2c_address, i2c_bus);
  _i2c_dev->begin();
}

/**
 * @brief Software enable for AW21036

 */
void AW21036::soft_enable() {

  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(_i2c_dev, GCR);
    Adafruit_BusIO_RegisterBits   enable_bit  = Adafruit_BusIO_RegisterBits(&control_reg, 1, 0);

  enable_bit.write(1);
}

/**
 * @brief Software Disable for AW21036

 */
void AW21036::soft_disable() {

  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(_i2c_dev, GCR);
    Adafruit_BusIO_RegisterBits   enable_bit  = Adafruit_BusIO_RegisterBits(&control_reg, 1, 0);

  enable_bit.write(0);
}

/**
 * @brief Get status of software enable/disable
 * 
 * @return bool true = EN-Register_Bit set to 1 (AW21036 Software enabled)

 */
bool AW21036::get_soft_en_status() {

  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(_i2c_dev, GCR);
    Adafruit_BusIO_RegisterBits   enable_bit  = Adafruit_BusIO_RegisterBits(&control_reg, 1, 0);

  return enable_bit.read();
}

/**
 * @brief Enable AW21036 through EN-Pin

 */
void AW21036::hard_enable() {

  digitalWrite(_enable_pin, HIGH);
}

/**
 * @brief Disable AW21036 through EN-Pin

 */
void AW21036::hard_disable() {

  digitalWrite(_enable_pin, LOW);
}

/**
 * @brief Get status of EN-Pin
 * 
 * @return bool true = EN is HIGH (AW21036 Hardware enabled)

 */
bool AW21036::get_hard_en_status()  {

  return digitalRead(_enable_pin);
}

/**
 * @brief Reset all registers to default value

 */
void AW21036::reset() {

  Adafruit_BusIO_Register reset_reg = Adafruit_BusIO_Register(_i2c_dev, RESET);
  reset_reg.write(0x00);
}

/**
 * @brief Get the chip version
 * 
 * @return 8-bit version number

 */
uint8_t AW21036::get_version() {

  Adafruit_BusIO_Register version_reg = Adafruit_BusIO_Register(_i2c_dev, VER);
  uint8_t version;
  version_reg.read(&version);

  return version;
}

/**
 * @brief Oscillator frequency corresponds with PWM frequency: 16 MHz => 62kHz, 8Mhz => 32kHz, 1Mhz => 4kHz, 512kHz => 2kHz, 256kHz => 1kHz, 125kHz => 500Hz, 62.5kHz => 244Hz, 31.25kHz => 122Hz
 * 
 * @param freq value between 0 and 7; 0 = 16Mhz, 1 = 8Mhz, 2 = 1Mhz, 3 = 512kHz, 4 = 256kHz, 5 = 125kHz, 6 = 62.5kHz, 7 = 31.25kHz 

 */
void AW21036::set_osc_freq(uint8_t freq)  {

  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(_i2c_dev, GCR);
    Adafruit_BusIO_RegisterBits   osc_bits  = Adafruit_BusIO_RegisterBits(&control_reg, 3, 4);

  if (freq >= 0 && freq <= 7) {
 
    osc_bits.write(freq);
  }
}

/**
 * @brief Value corresponds with PWM frequency: 0 => 62kHz, 1 => 32kHz, 2 => 4kHz, 3 => 2kHz, 4 => 1kHz, 5 => 500Hz, 6 => 244Hz, 7 => 122Hz
 * 
 * @return value from 0 to 7

 */
uint8_t AW21036::get_osc_freq() {

  Adafruit_BusIO_Register control_reg = Adafruit_BusIO_Register(_i2c_dev, GCR);
    Adafruit_BusIO_RegisterBits   osc_bits  = Adafruit_BusIO_RegisterBits(&control_reg, 3, 4);

  return osc_bits.read();
}


/**
 * @brief Set 8-bit global current 
 * 
 * @param current 0-255 global current value

 */
void AW21036::set_global_current(uint8_t current)  {

  Adafruit_BusIO_Register gccr_reg = Adafruit_BusIO_Register(_i2c_dev, GCCR);

  gccr_reg.write(current);

  _global_current = current;
}

/**
 * @brief Get global current value from internal var
 * 
 * @return 8-bit global current value

 */
uint8_t AW21036::get_global_current()  {

  return _global_current;
}

/**
 * @brief Get global current value from IC
 * 
 * @return 8-bit global current value

 */
uint8_t AW21036::get_real_global_current()  {

  Adafruit_BusIO_Register gccr_reg = Adafruit_BusIO_Register(_i2c_dev, GCCR);
  uint8_t global_current;
  gccr_reg.read(&global_current);

  return global_current;
}

/**
 * @brief Set if PWM phase delay should be enabled
 * 
 * @param delay true = enabled, false = disabled

 */
void AW21036::set_phase_delay(bool delay) {

  Adafruit_BusIO_Register phcr_reg = Adafruit_BusIO_Register(_i2c_dev, PHCR);
    Adafruit_BusIO_RegisterBits   phase_delay_bit  = Adafruit_BusIO_RegisterBits(&phcr_reg, 1, 7);

  if(delay == true) phase_delay_bit.write(1);
  else  phase_delay_bit.write(0);
}

/**
 * @brief Get PWM phase delay status
 * 
 * @return status

 */
bool AW21036::get_phase_delay() {

  Adafruit_BusIO_Register phcr_reg = Adafruit_BusIO_Register(_i2c_dev, PHCR);
    Adafruit_BusIO_RegisterBits   phase_delay_bit  = Adafruit_BusIO_RegisterBits(&phcr_reg, 1, 7);

  return phase_delay_bit.read();
}

/**
 * @brief Groups 3 LEDs as RGB and therefore sets all 3 LEDs to the same PWM brightness
 * 
 * @param group true = RGB group, false = individual LED control

 */
void AW21036::set_led_rgb_mode(bool group)  {

  Adafruit_BusIO_Register gcr2_reg = Adafruit_BusIO_Register(_i2c_dev, GCR2);
    Adafruit_BusIO_RegisterBits   led_rgb_bit  = Adafruit_BusIO_RegisterBits(&gcr2_reg, 1, 0);

  if(group == true) {

    led_rgb_bit.write(1);
    _rgb_mode = true;
  }

  else  {
    
    led_rgb_bit.write(0);
    _rgb_mode = false;
  }
}

/**
 * @brief Information if RGB mode is enabled (true) or disabled (false)
 * 
 * @return bool

 */
bool AW21036::get_led_rgb_mode()  {

  return _rgb_mode;
}

/**
 * @brief Set slew rate for LED output rising time 
 * 
 * @param slew true = 6ns, false = 1ns

 */
void AW21036::set_rising_slew_rate(bool slew)  {

  Adafruit_BusIO_Register gcr4_reg = Adafruit_BusIO_Register(_i2c_dev, GCR4);
    Adafruit_BusIO_RegisterBits   rising_slew_bit  = Adafruit_BusIO_RegisterBits(&gcr4_reg, 1, 2);

  if(slew == true) rising_slew_bit.write(1);
  else rising_slew_bit.write(0);
}

bool AW21036::get_rising_slew_rate()  {

  Adafruit_BusIO_Register gcr4_reg = Adafruit_BusIO_Register(_i2c_dev, GCR4);
    Adafruit_BusIO_RegisterBits   rising_slew_bit  = Adafruit_BusIO_RegisterBits(&gcr4_reg, 1, 2);

  return rising_slew_bit.read();
}

/**
 * @brief Set slew rate for LED output falling time 
 * 
 * @param slew value between 0 and 3; 0 = 1ns, 1 = 3ns, 2 = 6ns, 3 = 10ns

 */
void AW21036::set_falling_slew_rate(uint8_t slew)  {

  Adafruit_BusIO_Register gcr4_reg = Adafruit_BusIO_Register(_i2c_dev, GCR4);
    Adafruit_BusIO_RegisterBits   falling_slew_bits  = Adafruit_BusIO_RegisterBits(&gcr4_reg, 2, 0);

  if (slew >= 0 && slew <= 3) falling_slew_bits.write(slew);
}

/**
 * @brief Get slew rate for LED output falling time 
 * 
 * @return 0 = 1ns, 1 = 3ns, 2 = 6ns, 3 = 10ns

 */
uint8_t AW21036::get_falling_slew_rate()  {

  Adafruit_BusIO_Register gcr4_reg = Adafruit_BusIO_Register(_i2c_dev, GCR4);
    Adafruit_BusIO_RegisterBits   falling_slew_bits  = Adafruit_BusIO_RegisterBits(&gcr4_reg, 2, 0);

  return falling_slew_bits.read();
}

/**
 * @brief Set PWM brightness for an individual LED
 * 
 * @param led number of LED (RGB pixel for RGB mode) to adjust, starting with 0
 * @param PWM_brightness 0-255 PWM level

 */
void AW21036::set_PWM(uint8_t led, uint8_t PWM_brightness)  {

  // RGB mode disabled -> all LEDs individual PWM
  if (_rgb_mode == false) {

    if (led >= 36 && led <= 71) led = led-36;
    else if (led >= 72 && led <= 107) led = led-(2*36);
    else if (led >= 108 && led <= 143)  led = led-(3*36);
    else if (led > 143) return;
  }

  // RGB mode enabled -> each 3 LEDs (RGB pixel) grouped with same PWM
  else  {

    if (led >= 12 && led <= 23) led = led-12;
    else if (led >= 24 && led <= 35) led = led-(2*12);
    else if (led >= 36 && led <= 47)  led = led-(3*12);
    else if (led > 47) return;
  }

  Adafruit_BusIO_Register pwm_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, PWM0 + led);

  if (_CIE_active_for_PWM == false) {
    pwm_brightness_reg.write(PWM_brightness);
  }
  else  {
    pwm_brightness_reg.write(cie[PWM_brightness]);
  }

  _pwm[led] = PWM_brightness;
}

/**
 * @brief Set PWM brightness for a block of LEDs
 * 
 * @param start_led number of LED (RGB pixel for RGB mode) to start block
 * @param value_array pointer to array of pwm (brightness) values
 * @param block_length how many bytes shall be written

 */
void AW21036::set_PWM_Block(uint8_t start_led, uint8_t* value_array, uint8_t block_length)  {

  bool needs_update = false;
  for (uint8_t i = 0; i < block_length; i++) {
      if (_pwm[start_led + i] != value_array[i]) {
          needs_update = true;
          break;
      }
  }
  if (!needs_update) return; // Keine Änderung, also kein I2C-Transfer nötig

  // RGB mode disabled -> all LEDs individual PWM
  if (_rgb_mode == false) {

    if (start_led >= 36 && start_led <= 71) start_led = start_led-36;
    else if (start_led >= 72 && start_led <= 107) start_led = start_led-(2*36);
    else if (start_led >= 108 && start_led <= 143)  start_led = start_led-(3*36);
    else if (start_led > 143) return;

    if (block_length > 143) return;
  }

  // RGB mode enabled -> each 3 LEDs (RGB pixel) grouped with same PWM
  else  {

    if (start_led >= 12 && start_led <= 23) start_led = start_led-12;
    else if (start_led >= 24 && start_led <= 35) start_led = start_led-(2*12);
    else if (start_led >= 36 && start_led <= 47)  start_led = start_led-(3*12);
    else if (start_led > 47) return;

    if (block_length > 47) return;
  }

  Adafruit_BusIO_Register pwm_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, PWM0 + start_led, block_length, MSBFIRST);

  if (_CIE_active_for_PWM == false) {
    
    pwm_brightness_reg.write(value_array, block_length);
  } 
  
  else {

  uint8_t cie_array[block_length];
  
  for (uint8_t i = 0; i < block_length; i++) {

    cie_array[i] = cie[value_array[i]];
  }
    
  pwm_brightness_reg.write(cie_array, block_length);
  }

  for(uint8_t i = 0; i < block_length; i++) {

    _pwm[start_led + i] = value_array[i];
  }
}

/**
 * @brief Get PWM brightness for an individual LED or RGB pixel in RGB mode from internal var
 * 
 * @param led number of LED or pixel to get value from
 * 
 * @return 8-bit PWM value

 */
uint8_t AW21036::get_PWM(uint8_t led)  {

  if (_rgb_mode == false) {

    if (led >= 36 && led <= 71) led = led-36;
    else if (led >= 72 && led <= 107) led = led-(2*36);
    else if (led >= 108 && led <= 143)  led = led-(3*36);
  }

  else  {

    if (led >= 12 && led <= 23) led = led-12;
    else if (led >= 24 && led <= 35) led = led-(2*12);
    else if (led >= 36 && led <= 47)  led = led-(3*12);
  }

  return _pwm[led];
}

/**
 * @brief Get PWM brightness for an individual LED or RGB pixel in RGB mode from IC
 * 
 * @param led number of LED or pixel to get value from
 * 
 * @return 8-bit PWM value

 */
uint8_t AW21036::get_real_PWM(uint8_t led) {

  if (_rgb_mode == false) {

    if (led >= 36 && led <= 71) led = led-36;
    else if (led >= 72 && led <= 107) led = led-(2*36);
    else if (led >= 108 && led <= 143)  led = led-(3*36);
  }

  else  {

    if (led >= 12 && led <= 23) led = led-12;
    else if (led >= 24 && led <= 35) led = led-(2*12);
    else if (led >= 36 && led <= 47)  led = led-(3*12);
  }

  Adafruit_BusIO_Register pwm_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, PWM0 + led);
  uint8_t pwm_value;
  pwm_brightness_reg.read(&pwm_value);

  return pwm_value;
}

/**
 * @brief Manually set the internal variable for pwm channel
 * @param LED Channel or RGB pixel
 * @param pwm_value 8-bit PWM value

 */
void AW21036::set_PWM_var(uint8_t LED, uint8_t pwm_value) {

  _pwm[LED] = pwm_value;
}

/**
 * @brief Sync drivers to broadcast object for changed PWM variables
 * @param drivers point to array of AW21036 objects
 * @param driver_count number of AW21036 drivers
 * @param start_LED first channel or RGB pixel to sync
 * @param length number of consecutive channels/pixels to sync

 */
void AW21036::sync_pwm_to_broadcast(AW21036* drivers, uint8_t driver_count, uint8_t start_led, uint8_t length) {

  if (_i2c_address != AW21036_BROADCAST) return;

  for (uint8_t d = 0; d < driver_count; d++)  {
    for (uint8_t i = 0; i < length; i++)  {
      drivers[d].set_PWM_var(start_led + i, _pwm[start_led + i]);
    }
  }
}

/**
 * @brief Sync drivers to broadcast object for changed Current variables
 * @param drivers point to array of AW21036 objects
 * @param driver_count number of AW21036 drivers
 * @param start_LED first RGB pixel to sync
 * @param length number of consecutive pixels to sync

 */
void AW21036::sync_color_to_broadcast(AW21036* drivers, uint8_t driver_count, uint8_t start_led, uint8_t length) {

  if (_i2c_address != AW21036_BROADCAST) return;

  for (size_t d = 0; d < driver_count; d++) {
    for (uint8_t i = 0; i < length; i++) {
        // Für jede LED drei Bytes (RGB) synchronisieren
        uint8_t base = (start_led + i) * 3;
        drivers[d].set_Current_var(base,     _current[base]);
        drivers[d].set_Current_var(base + 1, _current[base + 1]);
        drivers[d].set_Current_var(base + 2, _current[base + 2]);
    }
  }
}

/**
 * @brief Update PWM brightness for all LEDs

 */
void AW21036::update_PWM()  {

  Adafruit_BusIO_Register pwm_update_reg = Adafruit_BusIO_Register(_i2c_dev, PWM_UPDATE);

  pwm_update_reg.write(0x00);
}

/**
 * @brief Set Analog brightness via current adjustment for an individual LED
 * @param led number of LED to adjust, starting with 0
 * @param analog_brightness 8-bit current level
 * 
 */
void AW21036::set_Current(uint8_t led, uint8_t analog_brightness)  {

  if (led >= 36 && led <= 71) led = led-36;
  else if (led >= 72 && led <= 107) led = led-(2*36);
  else if (led >= 108 && led <= 143)  led = led-(3*36);
  else if (led > 143) return;

  Adafruit_BusIO_Register analog_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, ANALOG0 + led);

  if (_CIE_active_for_Current == false) {
    analog_brightness_reg.write(analog_brightness);
  }
  else  {
    analog_brightness_reg.write(cie[analog_brightness]);
  }

  _current[led] = analog_brightness;
}

/**
 * @brief Get Analog brightness for an individual LED from var
 * @param led number of LED to get value from
 * @return 8-bit current value

 */
uint8_t AW21036::get_Current(uint8_t led) {

  if (led >= 36 && led <= 71) led = led-36;
  else if (led >= 72 && led <= 107) led = led-(2*36);
  else if (led >= 108 && led <= 143)  led = led-(3*36);

  return _current[led];

}

/**
 * @brief Get Analog brightness for an individual LED from IC
 * @param led number of LED to get value from
 * @return 8-bit current value
 */
uint8_t AW21036::get_real_Current(uint8_t led)  {

  if (led >= 36 && led <= 71) led = led-36;
  else if (led >= 72 && led <= 107) led = led-(2*36);
  else if (led >= 108 && led <= 143)  led = led-(3*36);

  Adafruit_BusIO_Register analog_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, ANALOG0 + led);
  uint8_t current_value;
  analog_brightness_reg.read(&current_value);

  return current_value;
}

/**
 * @brief Manually set the internal current variable per channel
 * 
 */
void AW21036::set_Current_var(uint8_t led, uint8_t analog_brightness_value) {

  _current[led] = analog_brightness_value;
}

/**
 * @brief Set PWM und Current for all LEDs
 * 
 * @param pwm 0-255 PWM level
 * @param current 0-255 current level

 */
void AW21036::set_all_leds(uint8_t pwm, uint8_t current)  {

  if (_rgb_mode == false) {

    for(uint8_t i = 0; i <= 35; i++)  {

      set_PWM(i, pwm);
    }
  }
  
  else  {

    for (uint8_t i = 0; i <= 11; i++) {

      set_PWM(i, pwm);
    }
  }

  update_PWM();

  for (uint8_t i = 0; i <= 35; i++) {

    set_Current(i, current);
  }
}

/**
 * @brief Set PWM und Current for all red LEDs
 * 
 * @param pwm 0-255 PWM level
 * @param current 0-255 current level

 */
void AW21036::set_all_red_leds(uint8_t pwm, uint8_t current) {

  if (_rgb_mode == false) {

    for (uint8_t i = 0; i<36; i = i+3)  {

      set_PWM(i, pwm);
    }

    update_PWM();
  }

  for (uint8_t i = 0; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set Current for all red LEDs
 * 
 * @param current 0-255 current level

 */
void AW21036::set_all_red_leds(uint8_t current) {

    for (uint8_t i = 0; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set PWM und Current for all green LEDs
 * 
 * @param pwm 0-255 PWM level
 * @param current 0-255 current level

 */
void AW21036::set_all_green_leds(uint8_t pwm, uint8_t current) {

  if (_rgb_mode == false) {

    for (uint8_t i = 1; i<36; i = i+3)  {

      set_PWM(i, pwm);
    }

    update_PWM();
  }
  
  for (uint8_t i = 1; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set Current for all green LEDs
 * 
 * @param current 0-255 current level

 */
void AW21036::set_all_green_leds(uint8_t current) {

    for (uint8_t i = 1; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set PWM und Current for all blue LEDs
 * 
 * @param pwm 0-255 PWM level
 * @param current 0-255 current level

 */
void AW21036::set_all_blue_leds(uint8_t pwm, uint8_t current) {

  if (_rgb_mode == false) {

    for (uint8_t i = 2; i<36; i = i+3)  {

      set_PWM(i, pwm);
    }

    update_PWM();
  }

  for (uint8_t i = 2; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set Current for all blue LEDs
 * 
 * @param current 0-255 current level

 */
void AW21036::set_all_blue_leds(uint8_t current) {

    for (uint8_t i = 2; i<36; i = i+3)  {

    set_Current(i, current);
  }
}

/**
 * @brief Set the color for an RGB pixel using current adjustment
 * 
 * @param pixel number of RGB-LED, starting with 0
 * @param red 0-255 for red
 * @param green 0-255 for green
 * @param blue 0-255 for blue

 */
void AW21036::set_RGB_color(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue)  {

  if (pixel >= 12 && pixel <= 23) pixel = pixel-12;
  else if (pixel >= 24 && pixel <= 35) pixel = pixel-(2*12);
  else if (pixel >= 36 && pixel <= 47)  pixel = pixel-(3*12);
  else if (pixel < 0 || pixel > 47) return;

  uint8_t led_red, led_green, led_blue;
  led_red = pixel * 3;
  led_green = pixel * 3 + 1;
  led_blue = pixel * 3 + 2;

  Adafruit_BusIO_Register analog_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, ANALOG0 + led_red, 3, MSBFIRST);

  if (_CIE_active_for_Current == false) {

    uint32_t hex;
    hex |= (uint32_t)red << 16;
    hex |= (uint32_t)green << 8;
    hex |= blue;

    analog_brightness_reg.write(hex);
  }
  else  {

    uint8_t cie_value_red, cie_value_green, cie_value_blue;
    cie_value_red = cie[red];
    cie_value_green = cie[green];
    cie_value_blue = cie[blue];

    uint32_t cie_hex;
    cie_hex |= (uint32_t)cie_value_red << 16;
    cie_hex |= (uint32_t)cie_value_green << 8;
    cie_hex |= cie_value_blue;

    analog_brightness_reg.write(cie_hex);
  }

  _current[led_red] = red;
  _current[led_green] = green;
  _current[led_blue] = blue;
}

/**
 * @brief Set the color for an RGB pixel using current adjustment
 * 
 * @param pixel number of RGB-LED, starting with 0
 * @param hex 24-bit (0xFFFFFF) HEX-code for RGB color

 */
void AW21036::set_RGB_color(uint8_t pixel, uint32_t hex)  {

  if (pixel >= 12 && pixel <= 23) pixel = pixel-12;
  else if (pixel >= 24 && pixel <= 35) pixel = pixel-(2*12);
  else if (pixel >= 36 && pixel <= 47)  pixel = pixel-(3*12);
  else if (pixel < 0 || pixel > 47) return;

  uint8_t led_red, led_green, led_blue;
  led_red = pixel * 3;
  led_green = pixel * 3 + 1;
  led_blue = pixel * 3 + 2;

  uint8_t value_red, value_green, value_blue;
  value_red = hex >> 16;
  value_green = (hex >> 8) & 0x00FF;
  value_blue = hex & 0x0000FF;

  Adafruit_BusIO_Register analog_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, ANALOG0 + led_red, 3, MSBFIRST);

  if (_CIE_active_for_Current == false) {

    analog_brightness_reg.write(hex);
  }
  else  {

    uint8_t cie_value_red, cie_value_green, cie_value_blue;
    cie_value_red = cie[value_red];
    cie_value_green = cie[value_green];
    cie_value_blue = cie[value_blue];

    uint32_t cie_hex;
    cie_hex |= (uint32_t)cie_value_red << 16;
    cie_hex |= (uint32_t)cie_value_green << 8;
    cie_hex |= cie_value_blue;

    analog_brightness_reg.write(cie_hex);

    // _current[led_red] = cie_value_red;
    // _current[led_green] = cie_value_green;
    // _current[led_blue] = cie_value_blue;
  }

  _current[led_red] = value_red;
  _current[led_green] = value_green;
  _current[led_blue] = value_blue;
}

/**
 * @brief Set Color as hex for a block of RGB-LEDS
 * @param start_pixel number of RGB-LED to start block
 * @param hex_value_array pointer to array of 24-bit (0xFFFFFF) HEX-values for RGB color
 * @param block_length how many pixels shall be updated

 */
void AW21036::set_RGB_color_Block(uint8_t start_pixel, uint32_t* hex_value_array, uint8_t block_length)  {

  if (start_pixel >= 12 && start_pixel <= 23) start_pixel = start_pixel-12;
    else if (start_pixel >= 24 && start_pixel <= 35) start_pixel = start_pixel-(2*12);
    else if (start_pixel >= 36 && start_pixel <= 47)  start_pixel = start_pixel-(3*12);
    else if (start_pixel > 47) return;

  if (block_length > 12) return;

  uint8_t byte_array[block_length * 3];

  for (uint8_t i = 0; i < block_length; i++)  {

    uint32_t rgb_value = hex_value_array[i];
    uint8_t byte_index = i * 3;

    byte_array[byte_index] = (rgb_value >> 16) & 0xFF;      // Rot
    byte_array[byte_index + 1] = (rgb_value >> 8) & 0xFF;   // Grün
    byte_array[byte_index + 2] = rgb_value & 0xFF;          // Blau
  }

  bool needs_update = false;
  for (uint8_t i = 0; i < block_length * 3; i++) {
      if (_current[(start_pixel * 3) + i] != byte_array[i]) {
          needs_update = true;
          break;
      }
  }
  if (!needs_update) return; // Keine Änderung, kein Transfer

  Adafruit_BusIO_Register analog_brightness_reg = Adafruit_BusIO_Register(_i2c_dev, ANALOG0 + start_pixel, block_length * 3, MSBFIRST);

  if (_CIE_active_for_PWM == false) {
    
    analog_brightness_reg.write(byte_array, block_length * 3);
  } 

  else  {

    uint8_t cie_array[block_length * 3];

    for (uint8_t i = 0; i < block_length * 3; i++)  {

      cie_array[i] = cie[byte_array[i]];
    }

    analog_brightness_reg.write(cie_array, block_length * 3);
  }

  for (uint8_t i = 0; i < block_length * 3; i++)  {

    _current[start_pixel + i] = byte_array[i];
  }
}

/**
 * @brief Get the RGB color value (24-bit) for a pixel
 * 
 * @param pixel number of RGB-LED, starting with 0
 * 
 * @return 24-bit hex RGB-code

 */
uint32_t AW21036::get_RGB_color(uint8_t pixel)  {

  uint8_t led_red, led_green, led_blue;
  led_red = pixel * 3;
  led_green = pixel * 3 + 1;
  led_blue = pixel * 3 + 2;

  uint8_t led_red_value, led_green_value, led_blue_value;
  led_red_value = _current[led_red];
  led_green_value = _current[led_green];
  led_blue_value = _current[led_blue];

  uint32_t values_combined;
  values_combined |= (uint32_t)led_red_value << 16;
  values_combined |= (uint32_t)led_green_value << 8;
  values_combined |= led_blue_value;

  return values_combined;
}

/**
 * @brief Get the color value of a single red/green/blue LED in a RGB pixel
 * 
 * @param pixel number of RGB-LED, starting with 0
 * @param color RED, GREEN or BLUE for the corresponding LED
 * 
 * @return 8-bit color value

 */
uint8_t AW21036::get_single_color_from_RGB(uint8_t pixel, RGB_color color)  {

  uint8_t led_red, led_green, led_blue;

  switch (color)  {
    case RED:
        led_red = pixel * 3;
        return _current[led_red];
        break;

    case GREEN:
        led_green = pixel * 3 + 1;
        return _current[led_green];
        break;

    case BLUE:
        led_blue = pixel * 3 + 2;
        return _current[led_blue];
        break;
  }
}

void AW21036::set_all_RGB_color(uint8_t red, uint8_t green, uint8_t blue) {

  set_all_red_leds(red);
  set_all_green_leds(green);
  set_all_blue_leds(blue);
}

void AW21036::set_all_RGB_color(uint32_t hex) {

  set_all_red_leds(  hex >> 16  );
  set_all_green_leds(  (hex >> 8) & 0x00FF  );
  set_all_blue_leds(  hex & 0x0000FF  );
}

/**
 * @brief Adjust global RGB colors for White Balance
 * 
 * @param red 0-255 red value
 * @param green 0-255 green value
 * @param blue 0-255 blue value

 */
void AW21036::set_RGB_for_White_Balance(uint8_t red, uint8_t green, uint8_t blue)  {

  Adafruit_BusIO_Register red_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBR);
  Adafruit_BusIO_Register green_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBG);
  Adafruit_BusIO_Register blue_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBB);

  red_for_white_balance_reg.write(red);
  green_for_white_balance_reg.write(green);
  blue_for_white_balance_reg.write(blue);
}

/**
 * @brief Get global RGB colors for White Balance
 * 
 * @param *red pointer to store red value
 * @param *green pointer to store green value
 * @param *blue pointer to store blue value
 * 
 * @return 24-bit hex RGB-code

 */
uint32_t AW21036::get_RGB_for_White_Balance(uint8_t *red, uint8_t *green, uint8_t *blue)  {

  Adafruit_BusIO_Register red_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBR);
  Adafruit_BusIO_Register green_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBG);
  Adafruit_BusIO_Register blue_for_white_balance_reg = Adafruit_BusIO_Register(_i2c_dev, WBB);

  uint8_t red_value, green_value, blue_value;
  red_for_white_balance_reg.read(&red_value);
  green_for_white_balance_reg.read(&green_value);
  blue_for_white_balance_reg.read(&blue_value);

  *red = red_value;
  *green = green_value;
  *blue = blue_value;

  uint32_t values_combined;
  values_combined |= (uint32_t)red_value << 16;
  values_combined |= (uint32_t)green_value << 8;
  values_combined |= blue_value;

  return values_combined;
}

/**
 * @brief set if to use CIE adjustment for PWM
 * @param status CIE enable status
 */
void AW21036::set_CIE_for_PWM(bool status)  {

  _CIE_active_for_PWM = status;
}

/**
 * @brief set if to use CIE adjustment for PWM
 * @return CIE enable status
 */
bool AW21036::get_CIE_for_PWM() {

  return _CIE_active_for_PWM;
}

/**
 * @brief set if to use CIE adjustment for Current
 * @param status CIE enable status
 */
void AW21036::set_CIE_for_Current(bool status)  {

  _CIE_active_for_Current = status;
}

/**
 * @brief set if to use CIE adjustment for PWM
 * @return CIE enable status
 */
bool AW21036::get_CIE_for_Current() {

  return _CIE_active_for_Current;
}

void AW21036::calculate_LEDs_from_pixel(uint8_t pixel, uint8_t *led_red, uint8_t *led_green, uint8_t *led_blue)  {

  *led_red = pixel * 3;
  *led_green = pixel * 3 + 1;
  *led_blue = pixel * 3 + 2;
}