// -----------------------------------------------------------------------------------
// axis simplefoc servo motor
#pragma once
#include "../../../../Common.h"

// ODRIVE DRIVER MODEL
#ifndef SIMPLEFOC_DRIVER_FIRST
  #define SIMPLEFOC_DRIVER_FIRST       300
  #define SIMPLEFOC                    300
  #define SIMPLEFOC_DRIVER_LAST        300
#endif

#define SF_UART       1
#define SF_I2C        2

#ifdef SIMPLEFOC_MOTOR_PRESENT
  // // #include <Wire.h>
  // #include "SimpleComms.h"
  // #include "comms/i2c/I2CCommanderMaster.h"
#include "../Motor.h"
#include "../../../convert/Convert.h"


// the following would be in the pinmap normally and should trigger #error on compile here when not present
// for now, they fit your hardware as best as I can tell...
#ifndef SIMPLEFOC_SERIAL
  #define SIMPLEFOC_SERIAL      Serial3 
#endif
#ifndef SIMPLEFOC_SERIAL_BAUD
  #define SIMPLEFOC_SERIAL_BAUD 19200
#endif
#ifndef SIMPLEFOC_RST_PIN
  #define SIMPLEFOC_RST_PIN     3
#endif

// simplefoc update rate default 10Hz
#ifndef SIMPLEFOC_UPDATE_MS
  #define SIMPLEFOC_UPDATE_MS   3000
#endif

// simplefoc direct slewing ON or OFF (simplefoc handles acceleration)
#ifndef SIMPLEFOC_SLEW_DIRECT
  #define SIMPLEFOC_SLEW_DIRECT OS_OFF
#endif

// odrive if using absolute encoders set to ON
#ifndef SIMPLEFOC_ABSOLUTE
  #define SIMPLEFOC_ABSOLUTE    OS_OFF
#endif

// odrive sync limit (for absolute encoders) OFF or specify the sync limit in arc-seconds
// this should, I hope, allow you to limit just how far from the encoders you can sync. (+/-) to fine-tune the origin
// with absolute encoders this protects you from exceeding the software min/max limits by > the amount specified
#ifndef SIMPLEFOC_SYNC_LIMIT
  #define SIMPLEFOC_SYNC_LIMIT  OS_OFF
#endif

// #if SIMPLEFOC_COMM_MODE == SF_UART
//   // #include <ODriveArduino.h> // https://github.com/odriverobotics/ODrive/tree/master/Arduino/ODriveArduino 
//   // ODrive servo motor serial driver
//   // extern UartCommanderMaster *commander;
// #elif SIMPLEFOC_COMM_MODE == SF_I2C

//   extern I2CCommanderMaster* commander;
// #endif

class SimpleFOCMotor : public Motor {
  public:
    // constructor
    SimpleFOCMotor(uint8_t axisNumber, bool useFastHardwareTimers = true);

    // sets up the odrive motor
    bool init();

    // set driver reverse state
    void setReverse(int8_t state);

    // get driver type code
    inline char getParameterTypeCode() { return 'O'; }  // codes used so far are S(tep/dir), T(mc), P(id), and O(Drive)

    // set driver parameters
    void setParameters(float param1, float param2, float param3, float param4, float param5, float param6);

    // validate driver parameters
    bool validateParameters(float param1, float param2, float param3, float param4, float param5, float param6);

    // sets motor enable on/off (if possible)
    void enable(bool value);

    // set instrument coordinate, in steps
    void setInstrumentCoordinateSteps(long value);

    // get the associated driver status
    DriverStatus getDriverStatus();

    // resets motor and target angular position in steps, also zeros backlash and index 
    void resetPositionSteps(long value);

    // get tracking mode steps per slewing mode step
    inline int getStepsPerStepSlewing() { return 64; }

    // get movement frequency in steps per second
    float getFrequencySteps();

    // set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
    void setFrequencySteps(float frequency);

    // set slewing state (hint that we are about to slew or are done slewing)
    void setSlewing(bool state);

    // updates PID and sets servo motor power/direction
    void poll();

    // sets dir as required and moves coord toward target at setFrequencySteps() rate
    void move();

  private:

//  float o_position0 = 0;
//  float o_position1 = 0;

    void setPosition(int motor_number, float position);

    unsigned long lastSetPositionTime = 0;
    uint8_t oDriveMonitorHandle = 0;
    uint8_t taskHandle = 0;

    int  stepSize = 1;                  // step size
    volatile int  homeSteps = 1;        // step count for microstep sequence between home positions (driver indexer)
    volatile bool takeStep = false;     // should we take a step

    float currentFrequency = 0.0F;      // last frequency set 
    float lastFrequency = 0.0F;         // last frequency requested
    unsigned long lastPeriod = 0;       // last timer period (in sub-micros)

    volatile int absStep = 1;           // absolute step size (unsigned)

    void (*callback)() = NULL;

    bool useFastHardwareTimers = true;

    bool isSlewing = false;

    DriverStatus status = { false, {false, false}, {false, false}, false, false, false, false };
    float stepsPerMeasure = 0.0F;
};

#endif