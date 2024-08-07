/******************************************************************
/ Arduino nano based moonlite focuser with temperature compensation
/
/ (c) 2024 GPL V2
/
/ Designed to work with INDI but should work with ascom.
/ Developed for sparkfun 'big easy driver' board and nema17 stepper motor.
/ See https://learn.sparkfun.com/tutorials/big-easy-driver-hookup-guide/all for more details
/
*/
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Motor pin definitions:
#define STEP_PIN 2
#define DIR_PIN 3
#define MS1_PIN 4
#define MS2_PIN 5
#define MS3_PIN 7
#define ENABLE_PIN  8
// temp probe pin
#define ONE_WIRE_BUS 6

// Define the AccelStepper interface type; 1 - DRIVER. Define all pins to driver board.
#define MotorInterfaceType 1
#define HOMEPOSITION 20000
#define MAXSPEED 2000
#define MAXCOMMAND 8
#define BACKLASHSTEPS 0 // determined by experiment. 

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AccelStepper stepper(MotorInterfaceType, STEP_PIN, DIR_PIN);

// EEPROM @ mapping. This is used on a cold start. Config settings saved and pos+temp from last known move. 
// These values allow the focuser to do a temp compensation move at the beginning of a night on powerup 
#define ENABLE_COMP_ADDR    0 
#define TEMPC_ADDR          ENABLE_COMP_ADDR+sizeof(bool) //The temperature coefficient
#define CUR_POS             TEMPC_ADDR+sizeof(int)        //Saved current motor position  
#define LAST_MOVE_TEMP      CUR_POS+sizeof(long)          //Saved temp when motor last moved

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char incomingCommand[MAXCOMMAND];
// long pos;
int isRunning = 0;
int previousDirection = 0;
long backlashApplied = 0;
int speed = 32;
int eoc = 0;
int idx = 0;
long millisLastMove = 0;
long millisLastTempRead = 0;
float lastTemperatureReading = 0;
float lastMotorMoveTemperatureReading = 0;
float tempDeltaToTriggerCompensation = 0.20;
int tempCoefficient = 0; //+-63
bool tempCompEnabled = true;

void setup()
{
  Serial.begin(9600);
  configureBigEasyStepperDriver();
  memset(incomingCommand, 0, MAXCOMMAND);
  millisLastMove = millis();
  sensors.begin(); 
  readTempSensor();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  EEPROM.get(ENABLE_COMP_ADDR, tempCompEnabled);
  EEPROM.get(TEMPC_ADDR, tempCoefficient);
  
  long savedPos;
  EEPROM.get(CUR_POS, savedPos);
  if (savedPos > 1000 && savedPos < 50000) {
    stepper.setCurrentPosition(savedPos);
  }
  float savedTemp;
  EEPROM.get(LAST_MOVE_TEMP, savedTemp);
  if (savedTemp > -50.00 && savedTemp < 50.00) {
    lastMotorMoveTemperatureReading=savedTemp;
  }
  if (tempCompEnabled) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

/**
 * Main loop. This will execute while the microcontroller has power.
 */
void loop() {
  if (!Serial.available())  {
    runStepper();
  } else {
    handleSerialInput();
  }
  monitorForTemperatureChanges();
  processSerialCommand();
}

/**
 * process the command we got and send a reply serial message
 */
void processSerialCommand() {
  if (eoc) {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(incomingCommand);
    if (len == 1) {
      strncpy(cmd, incomingCommand, 1);
    }
    if (len >= 2) {
      strncpy(cmd, incomingCommand, 2);
    }

    if (len > 2) {
      strncpy(param, incomingCommand + 2, len - 2);
    }

    memset(incomingCommand, 0, MAXCOMMAND);
    eoc = 0;
    idx = 0;

    // LED backlight value, always return "00"
    if (!strcasecmp(cmd, "GB")) {
      Serial.print("00#");
    }

    // firmware value, always return "10"
    if (!strcasecmp(cmd, "GV")) {
      Serial.print("10#");
    }

    // get the current motor position
    if (!strcasecmp(cmd, "GP")) {
      long pos = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the new motor position (target)
    if (!strcasecmp(cmd, "GN")) {
      long pos = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
    }

    // get the current temperature. YYYY# Returns the current temperature where YYYY is a four-digit signed (2’s complement) hex number. Precision is only to 0.5 deg
    if (!strcasecmp(cmd, "GT")) {      
      long t_int = (long) roundf(lastTemperatureReading * 2.0);
      char tempString[6];
      sprintf(tempString, "%04lX", t_int);
      Serial.print(tempString);
      Serial.print('#');
    }

    // get the temperature coefficient
    if (!strcasecmp(cmd, "GC")) {
      char tempString[4];
	    sprintf(tempString, "%02X", tempCoefficient);
	    Serial.print(tempString);
	    Serial.print('#');
    }

    // get the temp to log. This is not a Moonlite command. Used to collect data to manually determine coefficient
    if (!strcasecmp(cmd, "TL")) {
      long savedPos;
      EEPROM.get(CUR_POS, savedPos);
      Serial.print(savedPos);
      Serial.print(",");
      Serial.print(lastTemperatureReading,3);
      Serial.print(",");
      Serial.print(tempCoefficient);
    }

    // get the current motor speed, speed change not supported
    if (!strcasecmp(cmd, "GD")) {
      char tempString[6];
      sprintf(tempString, "%02X", speed);
      Serial.print(tempString);
      Serial.print("#");
    }

    // set speed. Unsupported
    if (!strcasecmp(cmd, "SD")) {
      speed = hexstr2long(param);
      stepper.setSpeed(MAXSPEED);
      stepper.setMaxSpeed(MAXSPEED);
    }

    // whether half-step is enabled or not, always return "00"
    if (!strcasecmp(cmd, "GH")) {
      Serial.print("00#");
    }

    
    // Enable Temp Comp
    if (!strcasecmp(cmd, "+")) {
      tempCompEnabled = true;
      digitalWrite(LED_BUILTIN, HIGH);
      EEPROM.put(ENABLE_COMP_ADDR, tempCompEnabled);
      // reset last move temp to current to prevent unexpected temp compensations
      lastMotorMoveTemperatureReading = lastTemperatureReading;
      EEPROM.put(LAST_MOVE_TEMP, lastTemperatureReading); 
    }

    // Disable Temp Comp
    if (!strcasecmp(cmd, "-")) {
      tempCompEnabled = false;
      digitalWrite(LED_BUILTIN, LOW);
      EEPROM.put(ENABLE_COMP_ADDR, tempCompEnabled);
    }

    // motor is moving - 01 if moving, 00 otherwise
    if (!strcasecmp(cmd, "GI")) {
      if (abs(stepper.distanceToGo()) > 0) {
        Serial.print("01#");
      } else {
        Serial.print("00#");
      }
    }

    // set current motor position. :SPYYYY# Where YYYY is a 4-digit unsigned hex number
    if (!strcasecmp(cmd, "SP")) {
      long pos = hexstr2long(param);
      stepper.setCurrentPosition(pos);
      EEPROM.put(CUR_POS, pos);
    }

    // set tempCoefficient. :SCXX# : 2 digit signed 2's complement number (range +-63)
    if (!strcasecmp(cmd, "SC")) {
      tempCoefficient = (int) strtol(param, NULL, 16) / 2;
      if (tempCoefficient > 63) {
        tempCoefficient -= 128;
      }     
      EEPROM.put(TEMPC_ADDR, tempCoefficient);
    }

    // set new motor position and compensate for backlash/ :SNYYYY# Where YYYY is a 4-digit unsigned hex number
    if (!strcasecmp(cmd, "SN") and !isRunning) {
      long pos = hexstr2long(param);
      moveMotorToPosition(pos);
    }


    // initiate a move
    if (!strcasecmp(cmd, "FG") and !isRunning) {
      stepper.enableOutputs();
    }

    // stop a move
    if (!strcasecmp(cmd, "FQ")) {
      stepper.moveTo(stepper.currentPosition());
      stepper.run();
    }
  }
}

void moveMotorToPosition(long pos) {
  // stepper.moveTo(pos);
  // 
  long curPos = stepper.currentPosition();
  if (curPos != pos) {
    int newDirection = 1;
    if (pos < curPos) {
      newDirection = -1;
    }
    if (previousDirection != 0) {
      if (newDirection != previousDirection) {
        pos = pos + (BACKLASHSTEPS * newDirection);
        backlashApplied = BACKLASHSTEPS * newDirection;
      }
    }
    previousDirection = newDirection;
    stepper.enableOutputs();
    stepper.moveTo(pos);
    lastMotorMoveTemperatureReading = lastTemperatureReading;
  }
}

void readTempSensor() {
 sensors.requestTemperatures();
 float reading = sensors.getTempCByIndex(0);
 if (reading < 40 && reading > -20) {
    lastTemperatureReading = sensors.getTempCByIndex(0);
 }
}

void applyTemperatureCompensation() {
  double delta = lastMotorMoveTemperatureReading - lastTemperatureReading;
  if (abs(delta) >= tempDeltaToTriggerCompensation && abs(delta) < 50.00) {
    long offset = (delta * tempCoefficient * 4) + stepper.currentPosition(); 
    moveMotorToPosition(offset);
    EEPROM.put(CUR_POS, offset);
    EEPROM.put(LAST_MOVE_TEMP, lastTemperatureReading);
  }
}

long hexstr2long(char *hexstr) {
  long ret = 0;
  ret = strtol(hexstr, NULL, 16);
  return (ret);
}

void configureBigEasyStepperDriver() {
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  //Set HLL for half step mode on the big-easy-driver board. LLL=full step. HLL=1/2 step, HL=1/4 step. HHL=1/8 step. HHH=1/16 step 
  digitalWrite(MS1_PIN, HIGH);
  digitalWrite(MS2_PIN, HIGH);
  digitalWrite(MS3_PIN, LOW);
  stepper.setMaxSpeed(MAXSPEED);
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true);
  stepper.setAcceleration(300);
  stepper.disableOutputs();
  stepper.setCurrentPosition(HOMEPOSITION);
}

void runStepper() {
  if (stepper.distanceToGo() == 0) {
    stepper.disableOutputs();
    isRunning = 0;
  } else {
    isRunning = 1;
    stepper.run();
    millisLastMove = millis();
    lastMotorMoveTemperatureReading = lastTemperatureReading;
  }
}

void handleSerialInput() {
  while (Serial.available() && !eoc) {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':') {
      incomingCommand[idx++] = inChar;
      if (idx >= MAXCOMMAND) {
        idx = MAXCOMMAND - 1;
      }
    }
    else {
      if (inChar == '#') {
        eoc = 1;
      }
    }
  }
}

void monitorForTemperatureChanges() {
  if (millis() > millisLastTempRead + 20000 && !isRunning) {
    //Every 20sec, get temp reading, store, apply compensation
    millisLastTempRead = millis();
    readTempSensor();
    if (tempCompEnabled) {
      applyTemperatureCompensation();
    }
  }
}