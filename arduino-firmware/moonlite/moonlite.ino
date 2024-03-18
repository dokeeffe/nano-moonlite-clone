/******************************************************************
/ Arduino nano based moonlite focuser with temperature compensation
/
/ (c) 2024 GPL V2
/
/ Designed to work with INDI but should work with ascom.
/
*/
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

// Motor pin definitions:
#define motorPin1  2      // IN1 on the ULN2003 driver
#define motorPin2  3      // IN2 on the ULN2003 driver
#define motorPin3  4     // IN3 on the ULN2003 driver
#define motorPin4  5     // IN4 on the ULN2003 driver
#define ONE_WIRE_BUS 6
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper stepper = AccelStepper(MotorInterfaceType, motorPin1, motorPin3, motorPin2, motorPin4);

#define HOMEPOSITION 20000
#define MAXSPEED 1000
#define SPEEDMULT 3
#define MAXCOMMAND 8
#define BACKLASHSTEPS 27 // determined by experiment. 

// EEPROM @ mapping. This is used on a cold start. Config settings saved and pos+temp from last known move. 
// These values allow the focuser to do a temp compensation move at the beginning of a night on powerup 
#define ENABLE_COMP_ADDR    0 
#define TEMPC_ADDR          ENABLE_COMP_ADDR+sizeof(bool) //The temperature coefficient
#define CUR_POS             TEMPC_ADDR+sizeof(int)        //Saved current motor position  
#define LAST_MOVE_TEMP      CUR_POS+sizeof(long)          //Saved temp when motor last moved

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
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
  stepper.setMaxSpeed(MAXSPEED);
  stepper.setSpeed(MAXSPEED);
  stepper.setAcceleration(MAXSPEED * .75);
  stepper.enableOutputs();
  stepper.setCurrentPosition(HOMEPOSITION);
  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();
  sensors.begin(); 
  readTempSensor();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  EEPROM.get(ENABLE_COMP_ADDR, tempCompEnabled);
  EEPROM.get(TEMPC_ADDR, tempCoefficient);
  
  long savedPos;
  EEPROM.get(CUR_POS, savedPos);
  if (savedPos > 1000 && savedPos < 30000) {
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


void loop()
{
  // run the stepper if there's no pending command and if there are pending movements
  if (!Serial.available())  {
    if (isRunning) {
      stepper.run();
      millisLastMove = millis();
      lastMotorMoveTemperatureReading = lastTemperatureReading;
    }
    else {
      // reported on INDI forum that some steppers "stutter" if disableOutputs is done repeatedly
      // over a short interval; hence we only disable the outputs and release the motor 1/4 second
      // after movement has stopped
      if ((millis() - millisLastMove) > 250) {
        stepper.disableOutputs();
      }
    }

    if (stepper.distanceToGo() == 0) {
      stepper.run();
      isRunning = 0;
      if (backlashApplied != 0) {
        //The motor has moved more than requested to compensate. Now reset the position to actual requested so the driver won't get confused when it gets the new position
        stepper.setCurrentPosition(stepper.currentPosition() - backlashApplied);
        backlashApplied = 0;
      }
    }
  } else {
    // read the command until the terminating # character
    while (Serial.available() && !eoc) {
      inChar = Serial.read();
      if (inChar != '#' && inChar != ':') {
        line[idx++] = inChar;
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

  if (millis() > millisLastTempRead + 20000 && !isRunning) {
    //Every 20sec, get temp reading, store, apply compensation
    millisLastTempRead = millis();
    readTempSensor();
    if (tempCompEnabled) {
      applyTemperatureCompensation();
    }
  }

  // process the command we got
  if (eoc) {
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);

    int len = strlen(line);
    if (len == 1) {
      strncpy(cmd, line, 1);
    }
    if (len >= 2) {
      strncpy(cmd, line, 2);
    }

    if (len > 2) {
      strncpy(param, line + 2, len - 2);
    }

    memset(line, 0, MAXCOMMAND);
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
      isRunning = 1;
      stepper.enableOutputs();
    }

    // stop a move
    if (!strcasecmp(cmd, "FQ")) {
      isRunning = 0;
      stepper.moveTo(stepper.currentPosition());
      stepper.run();
    }
  }
}

void moveMotorToPosition(long pos) {
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
    stepper.moveTo(pos);
  }
}

void readTempSensor() {
 sensors.requestTemperatures();
 float reading = sensors.getTempCByIndex(0);
 if (reading < 100) {
    lastTemperatureReading = sensors.getTempCByIndex(0);
 }
}

void applyTemperatureCompensation() {
  double delta = lastMotorMoveTemperatureReading - lastTemperatureReading;
  if (abs(delta) >= tempDeltaToTriggerCompensation && abs(delta) < 50.00) {
    long offset = (delta * tempCoefficient * 10) + stepper.currentPosition();
    moveMotorToPosition(offset);
    isRunning = 1;
    stepper.enableOutputs();
    EEPROM.put(CUR_POS, offset);
    EEPROM.put(LAST_MOVE_TEMP, lastTemperatureReading);
  }
}

long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}
