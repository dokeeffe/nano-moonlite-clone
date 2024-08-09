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
#define TEMPERATURE_POLLING_INTERVAL 20000

// Define the AccelStepper interface type as 1-DRIVER. 
#define MotorInterfaceType 1
#define HOMEPOSITION 20000
#define MAXSPEED 2000
#define MAXCOMMAND 8
#define BACKLASHSTEPS 0 // determined by experiment. 
// EPROM slots
#define ENABLE_COMP_ADDR    0 
#define TEMPC_ADDR          ENABLE_COMP_ADDR+sizeof(bool) //The temperature coefficient
#define CUR_POS             TEMPC_ADDR+sizeof(int)        //Saved current motor position  
#define LAST_MOVE_TEMP      CUR_POS+sizeof(long)          //Saved temp when motor last moved
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AccelStepper stepper(MotorInterfaceType, STEP_PIN, DIR_PIN);



char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char incomingCommand[MAXCOMMAND];
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

/**
 * @brief Initializes the microcontroller and sets up the necessary components.
 *
 * This function sets up the serial communication, configures the Big Easy Stepper Driver,
 * initializes variables, and reads the temperature sensor. It also checks the saved
 * settings in the EEPROM and applies the necessary configurations.
 *
 * @param None.
 *
 * @return void. This function does not return any value.
 *
 * @note The function assumes that the necessary board pin definitions, constants,
 * and libraries (like DallasTemperature) are already included and initialized.
 */
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
 * @brief The main loop of the program. This function handles the execution of various tasks.
 *
 * The loop continuously checks for serial input and performs the following tasks:
 * 1. If there is no available serial input, it runs the stepper motor using the runStepper() function.
 * 2. If there is available serial input, it handles the input using the handleSerialInput() function.
 * 3. It monitors for temperature changes and applies compensation if enabled using the monitorForTemperatureChanges() function.
 * 4. It processes any received serial commands using the processSerialCommand() function.
 *
 * @return void. This function does not return any value.
 */
void loop() {
  if (!Serial.available())  {
    runStepper();
  } else {
    handleSerialInput();
  }
  monitorForTemperatureChanges();
  processMoonliteSerialCommand();
}

/**
 * @brief Function to implement the moonlite serial protocol commandset
 * 
 * @return void. This function does not return any value.
 */
void processMoonliteSerialCommand() {
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

/**
 * This function is responsible for moving the motor to the specified position,
 * taking into account the backlash compensation and temperature compensation.
 *
 * @param pos The target position to move the motor to.
 *
 * @return void. This function does not return any value.
 *
 * @note The function checks if the current position is different from the target
 * position. If they are different, it calculates the new direction based on the
 * target position and compares it with the previous direction. If the directions
 * are different, it applies backlash compensation by adding or subtracting the
 * backlash steps to the target position.
 *
 * After calculating the new position, it enables the stepper motor outputs, sets
 * the new target position, and updates the last motor move temperature reading.
 */
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
    stepper.enableOutputs();
    stepper.moveTo(pos);
    lastMotorMoveTemperatureReading = lastTemperatureReading;
  }
}

/**
 * This function reads the temperature from the connected temperature sensor.
 *
 * @return void. This function does not return any value.
 *
 * @note The function uses the DallasTemperature library to request temperature
 * readings from the sensor and then retrieves the temperature in Celsius using
 * the getTempCByIndex() function. The function checks if the reading is within
 * a valid range (between -20 and 40 degrees Celsius) and updates the
 * lastTemperatureReading variable accordingly.
 */
void readTempSensor() {
 sensors.requestTemperatures();
 float reading = sensors.getTempCByIndex(0);
 if (reading < 40 && reading > -20) {
    lastTemperatureReading = sensors.getTempCByIndex(0);
 }
}

/**
 * This function applies temperature compensation to the motor position.
 * It calculates the temperature difference between the last motor move and the current temperature reading.
 * If the temperature difference is within the specified range, it applies a compensation offset to the motor position.
 * The offset is calculated based on the temperature difference, the temperature coefficient, and the current motor position.
 * The function then updates the current motor position, the last motor move temperature reading, and the saved position in the EEPROM.
 *
 * @return void. This function does not return any value.
 *
 * @note The function checks if the temperature difference is greater than or equal to the tempDeltaToTriggerCompensation
 * and less than 50.00. If the condition is met, it calculates the offset using the formula:
 * offset = (delta * tempCoefficient * 4) + stepper.currentPosition().
 * The offset is then applied to the motor position using the moveMotorToPosition() function.
 * After applying the offset, the function updates the current motor position, the last motor move temperature reading,
 * and the saved position in the EEPROM using the EEPROM.put() function.
 */
void applyTemperatureCompensation() {
  double delta = lastMotorMoveTemperatureReading - lastTemperatureReading;
  if (abs(delta) >= tempDeltaToTriggerCompensation && abs(delta) < 50.00) {
    long offset = (delta * tempCoefficient * 4) + stepper.currentPosition(); 
    moveMotorToPosition(offset);
    EEPROM.put(CUR_POS, offset);
    EEPROM.put(LAST_MOVE_TEMP, lastTemperatureReading);
  }
}

/**
 * Converts a hexadecimal string to a long integer.
 *
 * This function takes a hexadecimal string as input and converts it to a long integer.
 * It uses the strtol() function from the C standard library to perform the conversion.
 *
 * @param hexstr A pointer to the hexadecimal string to be converted.
 *
 * @return The converted long integer.
 *
 * @note The function does not check if the input string is a valid hexadecimal number.
 * It assumes that the input string is a valid hexadecimal number.
 *
 * @example
 *
 * char hexStr[] = "1234";
 * long result = hexstr2long(hexStr);
 * // result will be 4660
 */
long hexstr2long(char *hexstr) {
  long ret = 0;
  ret = strtol(hexstr, NULL, 16);
  return (ret);
}

/**
 * @brief Configures the Big Easy Stepper Driver 1/8th step mode.
 *
 * This function sets the pin modes for MS1, MS2, and MS3 to OUTPUT, and writes
 * specific values to these pins to enable the step mode on the Big Easy Driver
 * board. It also configures the stepper motor driver with the maximum speed,
 * enable pin, inverted pins, acceleration, and initial position.
 * The MS1,MS2 and MS3 pins on the board are used to configure the step mode as follows:
 * HLL = 1/2step
 * LLL=full step
 * HLL=1/2 step
 * HL=1/4 step
 * HHL=1/8 step
 * HHH=1/16 step 
 *
 * @param None.
 *
 * @return None.
 *
 * @note The function assumes that the necessary board pin definitions (MS1_PIN,
 * MS2_PIN, MS3_PIN, ENABLE_PIN) and constants (MAXSPEED, HOMEPOSITION) are
 * already defined and initialized.
 */
void configureBigEasyStepperDriver() {
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
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

/**
 * @brief Executes the stepper motor based on the distance to go.
 *
 * This function checks the distance to go using the stepper's distanceToGo() method.
 * If the distance is zero, it disables the stepper motor outputs and sets the isRunning flag to 0.
 * Otherwise, it sets the isRunning flag to 1, runs the stepper motor, updates the millisLastMove variable,
 * and stores the current temperature reading in the lastMotorMoveTemperatureReading variable.
 *
 * @param None.
 *
 * @return None.
 *
 * @note This function assumes that the stepper, millisLastMove, lastTemperatureReading, and isRunning variables are already initialized and accessible.
 */
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

/**
 * @brief Handles the serial input and builds the incoming command.
 *
 * This function continuously reads from the serial port until an end-of-command (EOC) character is received.
 * The EOC character is determined by the '#' character.
 * The incoming command is built by appending characters to the 'incomingCommand' array.
 * If the incoming command exceeds the maximum command length (MAXCOMMAND), the index is reset to MAXCOMMAND - 1.
 *
 * @param None.
 *
 * @return None.
 *
 * @note This function assumes that the 'Serial', 'incomingCommand', 'idx', and 'eoc' variables are already initialized and accessible.
 */
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

/**
 * @brief Monitors for temperature changes and applies compensation if enabled.
 *
 * This function checks if it's time to read the temperature sensor and apply compensation.
 * It does this by comparing the current time with the last time a temperature reading was taken.
 * If the specified interval has passed and the motor is not currently running, it reads the temperature,
 * stores the reading, and applies temperature compensation if enabled.
 *
 * @param None.
 *
 * @return None.
 *
 * @note This function assumes that the 'millis', 'millisLastTempRead', 'TEMPERATURE_POLLING_INTERVAL',
 * 'isRunning', 'readTempSensor()', and 'tempCompEnabled' variables are already initialized and accessible.
 */
void monitorForTemperatureChanges() {
  if (millis() > millisLastTempRead + TEMPERATURE_POLLING_INTERVAL && !isRunning) {
    // Every 20sec, get temp reading, store, apply compensation
    millisLastTempRead = millis();
    readTempSensor();
    if (tempCompEnabled) {
      applyTemperatureCompensation();
    }
  }
}