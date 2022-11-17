#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>
#include <PID_v1.h>

// import track data from csv file
// TODO: test that this works even a little bit
// it builds but like
// idk
// also note: csv must be a single line of comma separated values
const float testTrack[] = {
  #include "testdata.csv"
};

#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// TODO: switch if wrong direction
// NOTE: must be PWM enabled pins
#define HOIST_MOTOR_PIN_A 5
#define HOIST_MOTOR_PIN_B 6

#define HOIST_KP 2
#define HOIST_KI 5
#define HOIST_KD 1

#define SERVO_PIN 9
// TODO: adjust range to actual range of possible values
#define SERVO_MIN 30
#define SERVO_MAX 155
// note: this is A4
#define POT_PIN A4
// TODO: adjust range to actual range of possible values
#define POT_MIN 0
#define POT_MAX 1023

#define SERIAL_STOP_CHAR '\n'
#define SERIAL_RESISTANCE_DATA_CHAR 'r'
#define SERIAL_ANGLE_DATA_CHAR 'a'
#define SERIAL_DIFFICULTY_DATA_CHAR 'd'
#define SERIAL_TRACK_DATA_CHAR 't'

#define MAX_TRACK_LENGTH 999

Servo restistanceservo;

Encoder biketravel(ENCODER_PIN_A, ENCODER_PIN_B);

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

char trackData[MAX_TRACK_LENGTH];

/*
 * Function:  setResistance 
 * --------------------
 * sets the resistance of the bike by rotating the servo, with
 * 0 being the lowest and 255 being the highest resistance.
 * 
 * resistance: the resistance to set the bike to between 0 and 255
 */
void setResistance(int resistance) {
  int servoval = map(resistance, 0, 255, SERVO_MIN, SERVO_MAX);
  restistanceservo.write(servoval);
}

/*
 * Function: getBikeAnglePot
 * --------------------
 * gets the angle of the bike from the potentiometer.
 * 
 * returns: the value representing position of the range
 * the bike is at, with 0 being the lowest and 255 being the highest.
 */
char getBikeAnglePot() {
  // get the angle from the potentiometer on port 4
  int potval = analogRead(POT_PIN);
  // map the value to a value between 0 and 255
  char angle = map(potval, POT_MIN, POT_MAX, 0, 255);
  return angle;
}

/*
 * Function: getBikeAngleIMU
 * --------------------
 * gets the angle of the bike from the IMU (alternative).
 * 
 * returns: the value representing position of the range
 * the bike is at, with 0 being the lowest and 255 being the highest.
 */
long getBikeAngleIMU() {
  return 0;
}

/*
 * Function: setHoistVel
 * --------------------
 * sets the speed of the hoist motor using PWM switching on the
 * SSRs connected to the motor.
 * 
 * velocity: the speed to set the hoist motor to between -255 and 255
 */
void setHoistVel(short velocity) {
  if(velocity >= 0){
    analogWrite(HOIST_MOTOR_PIN_A, abs(velocity));
    analogWrite(HOIST_MOTOR_PIN_B, 0);
  } else {
    analogWrite(HOIST_MOTOR_PIN_A, 0);
    analogWrite(HOIST_MOTOR_PIN_B, abs(velocity));
  }
}

/*
 * Function: updateHoistPID
 * --------------------
 * updates the PID controller for the hoist motor.
 * sets speed of hoist based on PID controller output.
 */
void updateHoistPID() {
  hoistInput = getBikeAnglePot();
  hoistPID.Compute();
  setHoistVel(hoistOutput);
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
  if(Serial.available()) {
    char c = Serial.read();

    // below assumes that the data is sent in the format:
    // RESISTANCE_DATA_CHAR, resistance value
    // ANGLE_DATA_CHAR, angle value
    // DIFFICULTY_DATA_CHAR, resistance value, angle value
    if (c ==  SERIAL_RESISTANCE_DATA_CHAR || c == SERIAL_DIFFICULTY_DATA_CHAR){
      // set resistance based on input byte
      setResistance((unsigned char) Serial.read());
    }
    if (c == SERIAL_ANGLE_DATA_CHAR || c == SERIAL_DIFFICULTY_DATA_CHAR){
      // set desired angle based on input byte
      hoistSetPoint = (unsigned char) Serial.read();
    }
    else if (c == SERIAL_TRACK_DATA_CHAR){
      // read in track data as char vals from serial to trackData
      Serial.readBytesUntil(SERIAL_STOP_CHAR, trackData, MAX_TRACK_LENGTH);
    }
    return true;
  } else {
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  // set encoder pins to input
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  // set hoist motor pins to output
  pinMode(HOIST_MOTOR_PIN_A, OUTPUT);
  pinMode(HOIST_MOTOR_PIN_B, OUTPUT);

  restistanceservo.attach(9);  // attaches the servo on pin 9 to the servo object

  // initialize PID input
  hoistInput = getBikeAnglePot();
  hoistSetPoint = 0;
  // sets PID range to same as possible analog outputs
  hoistPID.SetOutputLimits(-255, 255);
  hoistPID.SetMode(AUTOMATIC);
}

void loop() {
  setResistance(0);
  delay(2000);
  setResistance(255);
  delay(2000);
}