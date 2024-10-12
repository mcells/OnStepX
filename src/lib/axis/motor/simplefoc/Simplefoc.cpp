// -----------------------------------------------------------------------------------
// axis odrive servo motor

#include "Simplefoc.h"

#ifdef SIMPLEFOC_MOTOR_PRESENT
  #include <Wire.h>
  // #include "SimpleFOCDrivers.h"
  #include "./i2ccommander/I2CCommanderMaster.h"
// #include "ODriveEnums.h"


#include "../../../tasks/OnTask.h"


SimpleFOCMotor *sfocMotorInstance[2];
IRAM_ATTR void moveSfocMotorAxis1() { sfocMotorInstance[0]->move(); }
IRAM_ATTR void moveSfocMotorAxis2() { sfocMotorInstance[1]->move(); }

// ODrive servo motor driver object pointer
#if SIMPLEFOC_COMM_MODE == SF_UART
  // UartCommanderMaster commander;
#elif SIMPLEFOC_COMM_MODE == SF_I2C

  I2CCommanderMaster* commander;
#endif

// constructor
SimpleFOCMotor::SimpleFOCMotor(uint8_t axisNumber, bool useFastHardwareTimers) {
  if (axisNumber < 1 || axisNumber > 2) return;

  driverType = SIMPLEDRIVER;
  strcpy(axisPrefix, "MSG: SFOC, ");
  axisPrefix[11] = '0' + this->axisNumber;
  #if SIMPLEFOC_SWAP_AXES == ON
    this->axisNumber = 3 - axisNumber;
  #else
    this->axisNumber = axisNumber;
  #endif


  if (axisNumber > 2) useFastHardwareTimers = false;
  this->useFastHardwareTimers = useFastHardwareTimers;

  if (axisNumber == 1) { // only do once since motor 2 object creation could
    #if SIMPLEFOC_COMM_MODE == SF_UART
      // commander = new UartCommanderMaster(SIMPLEFOC_SERIAL);
    #elif SIMPLEFOC_COMM_MODE == SF_I2C
      commander = new I2CCommanderMaster();
    #endif
  }

  // attach the function pointers to the callbacks
  sfocMotorInstance[this->axisNumber - 1] = this;
  switch (this->axisNumber) {
    case 1: callback = moveSfocMotorAxis1; break;
    case 2: callback = moveSfocMotorAxis2; break;
  }
}

bool SimpleFOCMotor::init() {
  if (axisNumber < 1 || axisNumber > 2) return false;

  if (axisNumber == 1) {
    // pinModeEx(SIMPLEFOC_RST_PIN, OUTPUT);
    // digitalWriteEx(SIMPLEFOC_RST_PIN, HIGH); // bring ODrive out of Reset
    delay(1000);  // allow time for ODrive to boot
  }
    #if SIMPLEFOC_COMM_MODE == SF_UART
      // SIMPLEFOC_SERIAL.begin(SIMPLEFOC_SERIAL_BAUD);
      // VLF("MSG: SFOC, SERIAL channel init");
      // commander->init();
    #elif SIMPLEFOC_COMM_MODE == SF_I2C
      if(!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, SIMPLEFOC_I2C_SPEED)){
        VLF("I2C initialization failed");
      }
      if (axisNumber <= SIMPLEFOC_I2C_MOTORS1)
      {
        V(axisNumber); VLF("Add motor");
        commander->addI2CMotors(SIMPLEFOC_I2C_ADDRESS1, SIMPLEFOC_I2C_MOTORS1);            // add target device, it has 1 motor

      }else if (axisNumber > SIMPLEFOC_I2C_MOTORS1 && axisNumber <= SIMPLEFOC_I2C_MOTORS1 + SIMPLEFOC_I2C_MOTORS2)
      {
        #if SIMPLEFOC_I2C_MOTORS2 > 0
          commander->addI2CMotors(SIMPLEFOC_I2C_ADDRESS2, SIMPLEFOC_I2C_MOTORS2);
        #endif
      }
    #endif
    
    commander->init();

    // uint8_t nummotors;
    // commander->readRegister(0, SimpleFOCRegister::REG_NUM_MOTORS, &nummotors, 1);
    // V(nummotors); VLF(" Motors in driver 1");
  

  enable(false);

  // start the motor timer
  V(axisPrefix);
  VF("start task to move motor... ");
  char timerName[] = "Target_";
  timerName[6] = '0' + axisNumber;
  taskHandle = tasks.add(0, 0, true, 0, callback, timerName);
  if (taskHandle) {
    V("success");
    if (useFastHardwareTimers && !tasks.requestHardwareTimer(taskHandle, 0)) { VLF(" (no hardware timer!)"); } else { VLF(""); }
  } else {
    VLF("FAILED!");
    return false;
  }

  return true;
}

// set driver reverse state
void SimpleFOCMotor::setReverse(int8_t state) {
  if (state == ON) {
    // VF("WRN: SFOC"); V(axisNumber); VF(", ");
    // VLF("axis reversal must be accomplished with hardware or SFOC setup!");
    reverse = true;
  } else
  {
    reverse = false;
  }
  
}

// set driver parameters
void SimpleFOCMotor::setParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  UNUSED(param1); // general purpose settings defined in Extended.config.h and stored in NV, they can be modified at runtime
  UNUSED(param2);
  UNUSED(param3);
  UNUSED(param4);
  UNUSED(param5);
  stepsPerMeasure = param6;
  setSlewing(isSlewing);
}

// validate driver parameters
bool SimpleFOCMotor::validateParameters(float param1, float param2, float param3, float param4, float param5, float param6) {
  UNUSED(param1);
  UNUSED(param2);
  UNUSED(param3);
  UNUSED(param4);
  UNUSED(param5);
  UNUSED(param6);
  return true;
}

// sets motor enable on/off (if possible)
void SimpleFOCMotor::enable(bool state) {
  V(axisPrefix); VF("driver powered ");
  if (state) { VLF("up"); } else { VLF("down"); } 
  VF("axisnumber"); V(axisNumber); VLF("bla");

  uint8_t requestedState = 0; //AXIS_STATE_IDLE
  if (state) requestedState = 1;//AXIS_STATE_CLOSED_LOOP_CONTROL
  
  #if SIMPLEFOC_COMM_MODE == SF_UART 
    // char command[32];
    // sprintF(command, "nE0\n");
    // command[0] = 'M' + motor_number;
    // command[2] = '0' + state ? 1 : 0;
    // SIMPLEFOC_SERIAL.print(command);
    // // float timeout = 0.5;                        
    //   // if(!_oDriveDriver->run_state(axisNumber - 1, requestedState, false, timeout)) {
    //   //   VF("WRN: SFOC"); V(axisNumber); VF(", ");
    //   //   VLF(" Power, closed loop control - command timeout!");
    //   //   return;
    //   // }
  #elif SIMPLEFOC_COMM_MODE == SF_I2C
    if (axisNumber <= SIMPLEFOC_I2C_MOTORS1)
    {
      VLF("drv1");
      uint8_t motorAddr = axisNumber - 1;
      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);
      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_ENABLE, &requestedState, 1);
    }else if (axisNumber > SIMPLEFOC_I2C_MOTORS1 && axisNumber <= SIMPLEFOC_I2C_MOTORS1 + SIMPLEFOC_I2C_MOTORS2)
    {
      VLF("drv2");
      uint8_t motorAddr = axisNumber - 1 - SIMPLEFOC_I2C_MOTORS1;
      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);

      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_ENABLE, &requestedState, 1);
    }
  #endif

  V(axisPrefix); VF("closed loop control - ");
  if (state) { VLF("Active"); } else { VLF("Idle"); }

  enabled = state;
}



void SimpleFOCMotor::setPosition(int motor_number, float position) {
  // special command to send high resolution position to odrive
  // VLF("set position");
  if (reverse)
  {
    position *= -1;
  }
  
  #if SIMPLEFOC_COMM_MODE == SF_UART
      
      char command[32];
      sprintF(command, "n%1.8f\n", position);
      command[0] = 'M' + motor_number;
      SIMPLEFOC_SERIAL.print(command);
    }
  #elif SIMPLEFOC_COMM_MODE == SF_I2C
    if (motor_number <= SIMPLEFOC_I2C_MOTORS1)
    {
      // if (motor_number == 0)
      {
        uint8_t motorAddr = motor_number;
        commander->writeRegister(motor_number, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);

        // float anangle;
        commander->writeRegister(motor_number, SimpleFOCRegister::REG_TARGET, &position, sizeof(float));
        // commander->readRegister(motor_number, SimpleFOCRegister::REG_TARGET, &anangle, sizeof(float));
        // V(anangle); VLF("set position");

      }
      
      
    }else if (motor_number > SIMPLEFOC_I2C_MOTORS1 && motor_number <= SIMPLEFOC_I2C_MOTORS1 + SIMPLEFOC_I2C_MOTORS2)
    {
      uint8_t motorAddr = motor_number - SIMPLEFOC_I2C_MOTORS1;
      commander->writeRegister(motor_number, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);

      commander->writeRegister(motor_number, SimpleFOCRegister::REG_TARGET, &position, sizeof(float));
    }
  #endif
}

void SimpleFOCMotor::setInstrumentCoordinateSteps(long value) {
  #if SIMPLEFOC_ABSOLUTE == ON && SIMPLEFOC_SYNC_LIMIT != OFF
    noInterrupts();
    long index = value - motorSteps;
    interrupts();
    float indexDeg = index/stepsPerMeasure;
    if (indexDeg >= -degToRadF(SIMPLEFOC_SYNC_LIMIT/3600.0F) && indexDeg <= degToRadF(SIMPLEFOC_SYNC_LIMIT/3600.0F))
  #endif
  Motor::setInstrumentCoordinateSteps(value);
}

// get the associated driver status
DriverStatus SimpleFOCMotor::getDriverStatus() {
  return status;
}

// resets motor and target angular position in steps, also zeros backlash and index
void SimpleFOCMotor::resetPositionSteps(long value) {
  // this is where the initial odrive position in "steps" is brought into agreement with the motor position in steps
  // not sure on this... but code below ignores (value,) gets the odrive position convert to steps and resets the motor
  // there (as the odrive encoders are absolute.)

  long oPosition;
  // if (axisNumber - 1 == 0) oPosition = o_position0;
  // if (axisNumber - 1 == 1) oPosition = o_position1;

  // get ODrive position in fractionial Turns
  #if SIMPLEFOC_COMM_MODE == SF_UART
    // oPosition = _oDriveDriver->GetPosition(axisNumber - 1)*TWO_PI*stepsPerMeasure; // axis1/2 are in steps per radian
  #elif SIMPLEFOC_COMM_MODE == SF_I2C
    float angle = 0.0f;
    if (axisNumber <= SIMPLEFOC_I2C_MOTORS1)
    {
      // if (axisNumber == 1)
      {
      uint8_t motorAddr = axisNumber - 1;
      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);

      commander->readRegister(axisNumber - 1, SimpleFOCRegister::REG_ANGLE, &angle, sizeof(float));
      }
    }else if (axisNumber > SIMPLEFOC_I2C_MOTORS1 && axisNumber <= SIMPLEFOC_I2C_MOTORS1 + SIMPLEFOC_I2C_MOTORS2)
    {
      uint8_t motorAddr = axisNumber - 1 - SIMPLEFOC_I2C_MOTORS1;
      commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_MOTOR_ADDRESS, &motorAddr, 1);

      commander->readRegister(axisNumber - 1, SimpleFOCRegister::REG_ANGLE, &angle, sizeof(float));
    }
    oPosition = angle*stepsPerMeasure;
    if (reverse)
    {
      oPosition *= -1;
    }
    
  #endif

  noInterrupts();
  motorSteps    = oPosition;
  targetSteps   = motorSteps;
  #if SIMPLEFOC_ABSOLUTE == OFF
    // but what if the odrive encoders are incremental?  how to tell the odrive what its angular position is?
    // here thinking we'll ignore it... sync OnStepX there and let the offset handle it
    indexSteps  = value - motorSteps;
  #else
    UNUSED(value);
    indexSteps = 0;
  #endif
  backlashSteps = 0;
  interrupts();
}

// set frequency (+/-) in steps per second negative frequencies move reverse in direction (0 stops motion)
void SimpleFOCMotor::setFrequencySteps(float frequency) {
  // negative frequency, convert to positive and reverse the direction
  int dir = 0;
  if (frequency > 0.0F) dir = 1; else if (frequency < 0.0F) { frequency = -frequency; dir = -1; }

  // if in backlash override the frequency
  if (inBacklash)
    frequency = backlashFrequency;

  if (frequency != currentFrequency) {
    lastFrequency = frequency;

    // if slewing has a larger step size divide the frequency to account for it
    if (lastFrequency <= backlashFrequency * 2.0F) stepSize = 1; else { if (!inBacklash) stepSize = 64; }
    frequency /= stepSize;

    // timer period in microseconds
    float period = 1000000.0F / frequency;

    // range is 0 to 134 seconds/step
    if (!isnan(period) && period <= 130000000.0F) {
      period *= 16.0F;
      lastPeriod = (unsigned long)lroundf(period);
    } else {
      lastPeriod = 0;
      frequency = 0.0F;
      dir = 0;
    }

    currentFrequency = frequency;

    // change the motor rate/direction
    noInterrupts();
    step = 0;
    interrupts();
    tasks.setPeriodSubMicros(taskHandle, lastPeriod);
  }

  noInterrupts();
  step = dir * stepSize;
  absStep = abs(step);
  interrupts();
}

float SimpleFOCMotor::getFrequencySteps() {
  if (lastPeriod == 0) return 0;
  return (16000000.0F / lastPeriod) * absStep;
}

// set slewing state (hint that we are about to slew or are done slewing)
void SimpleFOCMotor::setSlewing(bool state) {
  isSlewing = state;
}

// updates PID and sets odrive position
void SimpleFOCMotor::poll() {
  if ((long)(millis() - lastSetPositionTime) < SIMPLEFOC_UPDATE_MS) return;
  lastSetPositionTime = millis();

  noInterrupts();
  #if SIMPLEFOC_SLEW_DIRECT == ON
    long target = targetSteps + backlashSteps;
  #else
    long target = motorSteps + backlashSteps;
  #endif
  interrupts();
  #if SIMPLEFOC_COMM_MODE == SF_UART
    setPosition(axisNumber -1, target/(stepsPerMeasure));
  #elif SIMPLEFOC_COMM_MODE == SF_I2C
    setPosition(axisNumber -1, target/(stepsPerMeasure));
    // float angle = target/(TWO_PI*stepsPerMeasure);
    // if (axisNumber <= SIMPLEFOC_I2C_MOTORS1)
    // {
    //   commander->writeRegister(axisNumber - 1, SimpleFOCRegister::REG_ANGLE, &angle, sizeof(float));
    // }else if (axisNumber > SIMPLEFOC_I2C_MOTORS1 && axisNumber <= SIMPLEFOC_I2C_MOTORS1 + SIMPLEFOC_I2C_MOTORS2)
    // {
    //   commander->writeRegister(axisNumber - 1 - SIMPLEFOC_I2C_MOTORS1, SimpleFOCRegister::REG_ANGLE, &angle, sizeof(float));
    // }
  #endif
}

// sets dir as required and moves coord toward target at setFrequencySteps() rate
IRAM_ATTR void SimpleFOCMotor::move() {
  if (sync && !inBacklash) targetSteps += step;

  if (motorSteps > targetSteps) {
    if (backlashSteps > 0) {
      backlashSteps -= absStep;
      inBacklash = backlashSteps > 0;
    } else {
      motorSteps -= absStep;
      inBacklash = false;
    }
  } else

  if (motorSteps < targetSteps || inBacklash) {
    if (backlashSteps < backlashAmountSteps) {
      backlashSteps += absStep;
      inBacklash = backlashSteps < backlashAmountSteps;
    } else {
      motorSteps += absStep;
      inBacklash = false;
    }
  }
}

#endif
