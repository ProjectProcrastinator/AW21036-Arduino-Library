/*
 AW21036 Arduino Library
 ----------------------
 This library provides an interface for the AW21036 8-bit 36-LED driver from AWINIC.
 Most features of the AW21036 are implemented, except for the pattern controller.
 Optimized for fast I2C block writes and efficient LED updates.
*/

#ifndef AW21036_h
#define AW21036_h

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

// I2C Settings
#define AW21036_I2C_FREQ 400000UL
#define AW21036_I2C_OVERCLOCK 1000000UL

// Device I2C Addresses
#define AW21036_AD_GND 0x34
#define AW21036_AD_VDD 0x35
#define AW21036_AD_SCL 0x36
#define AW21036_AD_SDA 0x37

#define AW21036_BROADCAST 0x1C

// I2C Control-Registers

// Global Control Register
#define GCR 0x00

// Individual 8-bit PWM LED-Control Registers
#define PWM0 0x01
#define PWM1 0x02
#define PWM2 0x03
#define PWM3 0x04
#define PWM4 0x05
#define PWM5 0x06
#define PWM6 0x07
#define PWM7 0x08
#define PWM8 0x09
#define PWM9 0x0A
#define PWM10 0x0B
#define PWM11 0x0C
#define PWM12 0x0D
#define PWM13 0x0E
#define PWM14 0x0F
#define PWM15 0x10
#define PWM16 0x11
#define PWM17 0x12
#define PWM18 0x13
#define PWM19 0x14
#define PWM20 0x15
#define PWM21 0x16
#define PWM22 0x17
#define PWM23 0x18
#define PWM24 0x19
#define PWM25 0x1A
#define PWM26 0x1B
#define PWM27 0x1C
#define PWM28 0x1D
#define PWM29 0x1E
#define PWM30 0x1F
#define PWM31 0x20
#define PWM32 0x21
#define PWM33 0x22
#define PWM34 0x23
#define PWM35 0x24

// Update PWM Register
#define PWM_UPDATE 0x49

// Individual 8-bit Analog (Current) Dimming LED-Control Register
#define ANALOG0 0x4A
#define ANALOG1 0x4B
#define ANALOG2 0x4C
#define ANALOG3 0x4D
#define ANALOG4 0x4E
#define ANALOG5 0x4F
#define ANALOG6 0x50
#define ANALOG7 0x51
#define ANALOG8 0x52
#define ANALOG9 0x53
#define ANALOG10 0x54
#define ANALOG11 0x55
#define ANALOG12 0x56
#define ANALOG13 0x57
#define ANALOG14 0x58
#define ANALOG15 0x59
#define ANALOG16 0x5A
#define ANALOG17 0x5B
#define ANALOG18 0x5C
#define ANALOG19 0x5D
#define ANALOG20 0x5E
#define ANALOG21 0x5F
#define ANALOG22 0x60
#define ANALOG23 0x61
#define ANALOG24 0x62
#define ANALOG25 0x63
#define ANALOG26 0x64
#define ANALOG27 0x65
#define ANALOG28 0x66
#define ANALOG29 0x67
#define ANALOG30 0x68
#define ANALOG31 0x69
#define ANALOG32 0x6A
#define ANALOG33 0x6B
#define ANALOG34 0x6C
#define ANALOG35 0x6D

// 8-bit Global Current Control Register
#define GCCR 0x6E

// Phase Control Register
#define PHCR 0x70

// Open Short Detect Control Register
#define OSDCR 0x71

// Open/Short Status Register
#define OSST0 0x72
#define OSST1 0x73
#define OSST2 0x74
#define OSST3 0x75
#define OSST4 0x76

// Over Temperature Control Register
#define OTCR 0x77

// Spread Spectrum Control Register
#define SSCR 0x78

// UVLO Control Register
#define UVCR 0x79

// Further Global Control Registers
#define GCR2 0x7A
#define GCR4 0x7C

// Version Register
#define VER 0x7E

// Software Reset Register
#define RESET 0x7F

// Red/Green/Blue Scaling for White Balance
#define WBR 0x90
#define WBG 0x91
#define WBB 0x92

// Pattern Configure Registers
#define PATCFG 0xA0
#define PATGO 0xA1

// Pattern Timer Registers
#define PATT0 0xA2
#define PATT1 0xA3

// Pattern Control Registers
#define PATT2 0xA4
#define PATT3 0xA5

// Maximum/Minimum Brightness for Auto Breath
#define FADEH 0xA6
#define FADEL 0xA7

// Red/Green/Blue Mixing for Group Color
#define GCOLR 0xA8
#define GCOLG 0xA9
#define GCOLB 0xAA

// Group Configure Registers
#define GCFG0 0xAB
#define GCFG1 0xAC

// ENUM for individual colors of RGB
enum RGB_color {
    RED,
    GREEN,
    BLUE
};

const uint8_t cie[256] = {
    0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 
    1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 
    2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 
    3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 
    5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 
    7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 
    10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 
    13, 14, 14, 15, 15, 15, 16, 16, 17, 17, 
    17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 
    22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 
    28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 
    34, 34, 35, 36, 37, 37, 38, 39, 39, 40, 
    41, 42, 43, 43, 44, 45, 46, 47, 47, 48, 
    49, 50, 51, 52, 53, 54, 54, 55, 56, 57, 
    58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 
    68, 70, 71, 72, 73, 74, 75, 76, 77, 79, 
    80, 81, 82, 83, 85, 86, 87, 88, 90, 91, 
    92, 94, 95, 96, 98, 99, 100, 102, 103, 105, 
    106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 
    121, 123, 124, 126, 128, 129, 131, 132, 134, 136, 
    138, 139, 141, 143, 145, 146, 148, 150, 152, 154, 
    155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 
    175, 177, 179, 181, 183, 185, 187, 189, 191, 193, 
    196, 198, 200, 202, 204, 207, 209, 211, 214, 216, 
    218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 
    242, 245, 247, 250, 252, 255, 
};

/**
 * @brief Class to communicate with the AW21036
 */
class AW21036 
{
    public:
        AW21036(); // Constructor

        void begin(TwoWire* i2c_bus, uint8_t address, uint8_t enablePin);
        void begin_lite(TwoWire* i2c_bus, uint8_t address);

        void soft_enable();
        void soft_disable();
        bool get_soft_en_status();

        void hard_enable();
        void hard_disable();
        bool get_hard_en_status();

        void reset();
        uint8_t get_version();

        void set_osc_freq(uint8_t freq);
        uint8_t get_osc_freq();

        void set_global_current(uint8_t current);
        uint8_t get_global_current();
        uint8_t get_real_global_current();

        void set_phase_delay(bool delay);
        bool get_phase_delay();

        void set_led_rgb_mode(bool group);
        bool get_led_rgb_mode();

        void set_rising_slew_rate(bool slew);
        bool get_rising_slew_rate();
        void set_falling_slew_rate(uint8_t slew);
        uint8_t get_falling_slew_rate();

        void set_PWM(uint8_t LED, uint8_t PWM);
        void set_PWM_Block(uint8_t start_led, uint8_t* value_array, uint8_t block_length);
        uint8_t get_PWM(uint8_t led);
        uint8_t get_real_PWM(uint8_t led);
        void set_PWM_var(uint8_t LED, uint8_t pwm_value);

        void update_PWM();

        void set_Current(uint8_t led, uint8_t analog_brightness);
        uint8_t get_Current(uint8_t led);
        uint8_t get_real_Current(uint8_t led);
        void set_Current_var(uint8_t led, uint8_t analog_brightness_value);

        void sync_pwm_to_broadcast(AW21036* drivers, uint8_t driver_count, uint8_t start_led, uint8_t length);
        void sync_color_to_broadcast(AW21036* drivers, uint8_t driver_count, uint8_t start_led, uint8_t length);

        void set_all_leds(uint8_t pwm, uint8_t current);

        void set_all_red_leds(uint8_t pwm, uint8_t current);
        void set_all_red_leds(uint8_t current);
        void set_all_green_leds(uint8_t pwm, uint8_t current);
        void set_all_green_leds(uint8_t current);
        void set_all_blue_leds(uint8_t pwm, uint8_t current);
        void set_all_blue_leds(uint8_t current);

        void set_RGB_color(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
        void set_RGB_color(uint8_t pixel, uint32_t hex);
        void set_RGB_color_Block(uint8_t start_pixel, uint32_t* hex_value_array, uint8_t block_length);
        uint32_t get_RGB_color(uint8_t pixel);
        uint8_t get_single_color_from_RGB(uint8_t pixel, RGB_color color);

        void set_all_RGB_color(uint8_t red, uint8_t green, uint8_t blue);
        void set_all_RGB_color(uint32_t hex);

        void calculate_LEDs_from_pixel(uint8_t pixel, uint8_t *led_red, uint8_t *led_green, uint8_t *led_blue);

        void set_RGB_for_White_Balance(uint8_t red, uint8_t green, uint8_t blue);
        uint32_t get_RGB_for_White_Balance(uint8_t *red, uint8_t *green, uint8_t *blue);

        void set_CIE_for_PWM(bool status);
        bool get_CIE_for_PWM();
        void set_CIE_for_Current(bool status);
        bool get_CIE_for_Current();

    protected:
        Adafruit_I2CDevice *_i2c_dev; ///< I2C bus device

    private:
        uint8_t     _i2c_address = 0x34;
        uint8_t     _enable_pin = 0xFF;

        bool        _rgb_mode = false;
        bool        _CIE_active_for_Current = true;
        bool        _CIE_active_for_PWM = false;
        bool        _MHPA0808RGBDT = false;

        uint8_t     _global_current = 0;
        uint8_t     _pwm[36];
        uint8_t     _current[36];
};


#endif
