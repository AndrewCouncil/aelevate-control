#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <limits.h>

// import track data from csv file
// TODO: test that this works even a little bit
// it builds but like
// idk
// also note: csv must be a single line of comma separated values
const float testTrack[] = {
  #include "testdata.csv"
};

#define LOOP_DURATION 100 // ms

#define HEF_PIN 2

// TODO: switch if wrong direction
// NOTE: must be PWM enabled pins
#define HOIST_MOTOR_PIN_A 5
#define HOIST_MOTOR_PIN_B 6

#define LIMIT_PIN 3

#define HOIST_WAVE_COUNT 10
#define HOIST_WAVE_PERIOD HOIST_WAVE_COUNT*LOOP_DURATION // ms

#define HOIST_KP 2
#define HOIST_KI 5
#define HOIST_KD 1

#define SERVO_PIN 9
// TODO: adjust range to actual range of possible values
#define SERVO_MIN 30
#define SERVO_MAX 155

#define POT_PIN A4
// TODO: adjust range to actual range of possible values
#define POT_MIN 130
#define POT_MAX 170
#define POT_MID ((POT_MAX + POT_MIN) / 2)

#define SERIAL_RESET_CHAR 's'
#define SERIAL_POSITION_START_CHAR 'p'
#define SERIAL_POSITION_STOP_CHAR 'P'
#define SERIAL_RESISTANCE_DATA_CHAR 'r'
#define SERIAL_ANGLE_DATA_CHAR 'a'
#define SERIAL_DIFFICULTY_DATA_CHAR 'd'
#define SERIAL_TRACK_DATA_CHAR 't'
#define SERIAL_END_CHAR '\n'

#define MAX_TRACK_LENGTH 999

Servo restistanceServo;

int hoistLoops = 0;

bool isWritingPosition = false;

unsigned long currentBikePosition = 0;

double hoistSetPoint, hoistInput, hoistOutput;
PID hoistPID(
  &hoistInput, 
  &hoistOutput,
  &hoistSetPoint,
  HOIST_KP,
  HOIST_KI,
  HOIST_KD,
  DIRECT
);

unsigned char trackData[MAX_TRACK_LENGTH];

/*
 * Function:  setResistance 
 * --------------------
 * sets the resistance of the bike by rotating the servo, with
 * 0 being the lowest and 254 being the highest resistance.
 * 
 * resistance: the resistance to set the bike to between 0 and 254
 */
void setResistance(int resistance) {
  int servoval = map(resistance, 0, 254, SERVO_MIN, SERVO_MAX);
  restistanceServo.write(servoval);
}

/*
 * Function: getBikeAnglePot
 * --------------------
 * gets the angle of the bike from the potentiometer.
 * 
 * returns: the value representing position of the range
 * the bike is at, with 0 being the lowest and 254 being the highest.
 */
long getBikeAnglePot() {
  // get the angle from the potentiometer on port 4
  int potval = analogRead(POT_PIN);
  Serial.println(potval);
  // map the value to a value between 0 and 254
  long angle = map(potval, POT_MIN, POT_MAX, 0, 254);
  return angle;
}

/*
 * Function: getBikeAngleIMU
 * --------------------
 * gets the angle of the bike from the IMU (alternative).
 * 
 * returns: the value representing position of the range
 * the bike is at, with 0 being the lowest and 254 being the highest.
 */
long getBikeAngleIMU() {
  return 0;
}

/*
 * Function: setHoistVelPWM
 * --------------------
 * sets the speed of the hoist motor using PWM switching on the
 * SSRs connected to the motor.
 * 
 * velocity: the speed to set the hoist motor to between -255 and 255
 */
void setHoistVelPWM(short velocity) {
  if(velocity >= 0){
    analogWrite(HOIST_MOTOR_PIN_A, abs(velocity));
    analogWrite(HOIST_MOTOR_PIN_B, 0);
  } else {
    analogWrite(HOIST_MOTOR_PIN_A, 0);
    analogWrite(HOIST_MOTOR_PIN_B, abs(velocity));
  }
}

/*
 * Function: setHoistVelTime
 * --------------------
 * sets the speed of the hoist motor using delay-based switching on the
 * SSRs connected to the motor.
 * 
 * velocity: the speed to set the hoist motor to between -255 and 255
 */
void setHoistVelTime(short velocity){
  // count up the number of loops
  hoistLoops++;
  // if the number of loops is equal to loop count, set back to 0 
  if (hoistLoops >= HOIST_WAVE_COUNT) hoistLoops = 0;
  // set number of loops to run on based on proportion out of 255
  int highWaveCount = ceil((HOIST_WAVE_COUNT * ((float)abs(velocity)) / 255));
  // if the loop count is less than high count, set motor on in direction given
  digitalWrite(
    HOIST_MOTOR_PIN_A,
    (hoistLoops < highWaveCount) && (velocity > 0)
  );
  digitalWrite(
    HOIST_MOTOR_PIN_B,
    (hoistLoops < highWaveCount) && (velocity < 0)
  );
}

void setHoistVel(short velocity) {
  setHoistVelTime(velocity);
}

bool halfSpeedState = false;
void setHoistVelSimple(short velocity) {
  switch (velocity) {
    case 0:
      digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
      digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
      break;
    case -2:
      digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
      digitalWrite(HOIST_MOTOR_PIN_B, LOW);
      break;
    case 2:
      digitalWrite(HOIST_MOTOR_PIN_A, LOW);
      digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
      break;
    case -1:
      digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
      digitalWrite(HOIST_MOTOR_PIN_B, halfSpeedState);
      break;
    case 1:
      digitalWrite(HOIST_MOTOR_PIN_A, halfSpeedState);
      digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
      break;
  }
  hoistLoops++;
  if(hoistLoops >= HOIST_WAVE_COUNT){
    hoistLoops = 0;
    halfSpeedState = !halfSpeedState;
  }
}

void badControl(long input) {
  // Serial.println(input);
  // Serial.print("      ");
  // Serial.println((unsigned int)hoistSetPoint);
  if(input <  hoistSetPoint - 20){
    Serial.println("too low");
    setHoistVelSimple(2);
  } else if(input > hoistSetPoint + 20){
    Serial.println("too high");
    setHoistVelSimple(-2);
  } else {
    setHoistVelSimple(0);
  }
  return;
}

/*
 * Function: updateHoistPID
 * --------------------
 * updates the PID controller for the hoist motor.
 * sets speed of hoist based on PID controller output.
 */
void updateHoistControl() {
  hoistInput = getBikeAnglePot();
  // if hoistInput is within +- 10 of the setpoint, set to 0
  hoistPID.Compute();
  if (abs(hoistInput - hoistSetPoint) <= 10) {
    setHoistVel(0);
  } else {
    hoistPID.Compute();
    setHoistVel(hoistOutput);
  }
}

/*
 * Function: resetBike
 * --------------------
 * resets the bike to the lowest position.
 * stops writing position data to serial.
 */
void resetBike() {
  // stop the bike
  setResistance(254);
  hoistSetPoint = 0;
  // reset the position
  currentBikePosition = 0;
  // stop writing position
  isWritingPosition = false;
}

/*
 * Function: checkSerial
 * --------------------
 * checks if serial read is available.
 * if so, performs appropriate serial read
 * based on prompting character
 * 
 * returns: true if serial read was performed, false otherwise
 */
bool checkSerial() {
  if(Serial.available() > 0) {
    // checks if magic number
    unsigned char is255 = Serial.read();
    if(is255 != 255) {
      // set builtin LED on for 1 second
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      return false;
    }

    // read in command character
    while(Serial.available() == 0);
    char c = Serial.read();
    switch(c) {
      case SERIAL_RESET_CHAR:
        resetBike();
        return true;
      
      case SERIAL_POSITION_START_CHAR:
        // set the bike to start writing position to serial
        isWritingPosition = true;
        return true;
      
      case SERIAL_POSITION_STOP_CHAR:
        // set the bike to stop writing position to serial
        isWritingPosition = false;
        return true;

      // RESISTANCE_DATA_CHAR, resistance value
      case SERIAL_RESISTANCE_DATA_CHAR:
        // set resistance
        while(Serial.available() == 0);
        setResistance(Serial.read());
        return true;

      // ANGLE_DATA_CHAR, angle value
      case SERIAL_ANGLE_DATA_CHAR:
        // set angle
        hoistSetPoint = (unsigned char) Serial.read();
        return true;

      // DIFFICULTY_DATA_CHAR, resistance value, angle value
      case SERIAL_DIFFICULTY_DATA_CHAR:
        // set resistance and angle
        while(Serial.available() == 0);
        setResistance(Serial.read());
        hoistSetPoint = (unsigned char) Serial.read();
        return true;
      
      case SERIAL_TRACK_DATA_CHAR:
        // set trackData to characters read from serial
        // NOTE: probably not using this, seems like a bad idea
        while(Serial.available() == 0);
        Serial.readBytesUntil(
          SERIAL_END_CHAR,
          trackData,
          MAX_TRACK_LENGTH
        );
        return true;

      default:
        Serial.println("Invalid serial command");
        return false;
    }
  } 
  return false;
}

/*
 * Function: writePositionSerial
 * --------------------
 * writes bike position to serial followed
 * by the time in millis.
 * 
 * returns: true if position updated and wrote values to serial,
 * false otherwise
 */
void writePositionSerial() {
  Serial.write(currentBikePosition);
  Serial.write(millis());
}

/*
 * Function: hallEffISR
 * --------------------
 * runs when the hall effect sensor gets a positive edge.
 * increments the current bike position and prints position
 * to serial if isWritingPosition is true.
 */
void hallEffISR() {
  currentBikePosition++;
  if(isWritingPosition) {
    writePositionSerial();
  }
  // Serial.println(currentBikePosition);
}

/*
 * Function: limitISR
 * --------------------
 * runs when the limit switch gets a positive edge.
 * prints limit reached to serial.
 */
void limitISR() {
  // print that limit has been reached
  Serial.println("Limit reached");
}

void setup() {
  Serial.begin(9600, SERIAL_8E1);
  // Serial.begin(9600);

  // set builtin LED to output
  pinMode(LED_BUILTIN, OUTPUT);

  // set hall effect sensor pin to input
  pinMode(HEF_PIN, INPUT_PULLUP);
  // increment position on rising edge interrupt
  attachInterrupt(digitalPinToInterrupt(HEF_PIN), hallEffISR, FALLING);

  // set limit switch pin to input
  pinMode(LIMIT_PIN, INPUT);
  // reset bike on rising edge interrupt
  // attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), limitISR, RISING);

  // set hoist motor pins to output
  pinMode(HOIST_MOTOR_PIN_A, OUTPUT);
  pinMode(HOIST_MOTOR_PIN_B, OUTPUT);
  // turn motor off
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);

  // attaches the servo on pin 9 to the servo object
  restistanceServo.attach(9);  

  // // initialize PID input
  // hoistInput = getBikeAnglePot();
  // hoistSetPoint = 255/2;
  // // sets PID range to same as possible analog outputs
  // hoistPID.SetOutputLimits(-255, 255);
  // hoistPID.SetMode(AUTOMATIC);
}

void controlLoop() {
  // update PID controller
  updateHoistControl();
  // check if serial read is available
  checkSerial();
  // if(isWritingPosition) {
  //   writePositionSerial();
  // }
}

void testMotorBasic() {
  // test basic functionality of the motor
  // setHoistVel(255);
  // delay(1000);
  // setHoistVel(-255);
  // delay(1000);
  setHoistVelSimple(-1);
  delay(LOOP_DURATION);
}

void testMotorAnalog() {
  // test analogWrite functionality of the motor
  for(int i = -255; i <= 255; i++) {
    setHoistVel(i);
    delay(10);
  }
  for(int i = 255; i >= -255; i--) {
    setHoistVel(i);
    delay(10);
  }
}

void testAngle() {
  // test angle functionality
  for(int i = 0; i <= 254; i++) {
    setHoistVel(i);
    delay(10);
  }
  for(int i = 254; i >= 0; i--) {
    setHoistVel(i);
    delay(10);
  }
}

void testPID() {
  // test PID functionality
  for(int i = 0; i <= 254; i++) {
    hoistSetPoint = i;
    updateHoistControl();
    delay(10);
  }
  for(int i = 254; i >= 0; i--) {
    hoistSetPoint = i;
    updateHoistControl();
    delay(10);
  }
}

void testSetAngle() {
  hoistSetPoint = 250;
  badControl(getBikeAnglePot());
  delay(LOOP_DURATION);
}

void testDistance() {
  isWritingPosition = true;
  delay(9999999);
}

void testHEF() {
  // set builtin LED to state of hall effect sensor
  while(true) {
    digitalWrite(LED_BUILTIN, digitalRead(HEF_PIN));
    delay(LOOP_DURATION);
  }
}

void testPot() {
  // print the pot value to serial
  Serial.println(analogRead(POT_PIN));
  delay(LOOP_DURATION);
}

void simpleHoist() {
  // simple hoist test
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(1000);
  // DOWN
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, LOW);
  delay(1000);
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(1000);
  // UP
  digitalWrite(HOIST_MOTOR_PIN_A, LOW);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(300);
  // flash the builtin led
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(500);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(500);
}

void stepDown() {
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(1000);
  // DOWN
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, LOW);
  delay(250);
}

void stepUp() {
  digitalWrite(HOIST_MOTOR_PIN_A, HIGH);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(1000);
  // UP
  digitalWrite(HOIST_MOTOR_PIN_A, LOW);
  digitalWrite(HOIST_MOTOR_PIN_B, HIGH);
  delay(250);
}

void loop() {
  // setResistance(0);
  // delay(2000);
  // setResistance(254);
  // delay(2000);
  // testDistance();
  // testHEF();
  // simpleHoist();
  // testMotorBasic();
  // testSetAngle();
  // stepDown();
  // stepUp();
  // testPot();
  // blink the buildin led
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}