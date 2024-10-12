// -------------------------------------------------------------------------------------------------
// Pin map for ESP32 Devboard (ESP32)
#pragma once

#if defined(ESP32)

// Serial0: RX Pin GPIO3, TX Pin GPIO1 (to USB serial adapter)

#if SERIAL_A_BAUD_DEFAULT != OFF
  #define SERIAL_A              Serial
#endif
#if SERIAL_B_BAUD_DEFAULT != OFF
  #define SERIAL_B              Serial2
#endif


// Specify the ESP32 I2C pins
#define I2C_SDA_PIN             14
#define I2C_SCL_PIN             12

#else
#error "Wrong processor for this configuration!"

#endif
