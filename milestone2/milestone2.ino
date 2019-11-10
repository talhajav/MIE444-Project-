/* NOTE:
 *    - PINS LEFT ON ARDUINO UNO: 2, 4, 6 - NEED TO SWTICH TO ARDUINO MEGA 
 */

#include <SoftwareSerial.h>
#include <Servo.h>

// bluetooth
#define RXpin 10 // bluetooth's TX pin goes here
#define TXpin 11 // bluetooth's RX pin goes here
SoftwareSerial BTSerial(RXpin, TXpin); // RX | TX

// servo motors
#define gripperPin 5
Servo gripper;

// dc motor
#define RightMotorIn1 12
#define RightMotorIn2 7
#define LeftMotorIn1 13
#define LeftMotorIn2 8
#define RightMotorPWM 3
#define LeftMotorPWM 9

// function declarations
void action();
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void loadBlock();
void unloadBlock();

// motor speed
int forward_speed1 = 100; // right motor
int forward_speed2 = 70; // left motor
int backward_speed1 = 150; // right motor
int backward_speed2 = 100; // left motor
int turn_speed = 80;

// servo position
int startPos = 180;
int finalPos = startPos - 70;

int data;
int command = 0;

void setup() {
  Serial.begin(9600); 
  BTSerial.begin(9600); 

  pinMode(RightMotorIn1, OUTPUT); // Right Motor
  pinMode(RightMotorIn2, OUTPUT);
  pinMode(LeftMotorIn1, OUTPUT); // Left motor
  pinMode(LeftMotorIn2, OUTPUT);

  brake(); // start at braking position

  gripper.attach(gripperPin); // Gripper Servo
  unloadBlock(); // start at initial position
}
void loop() {
  if(BTSerial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
      data = BTSerial.read(); // Reads the data from the bluetooth serial port
      if(data >= 65 && data <= 72)
        command = data;
  } 
  Serial.println(command);
  action(command);  
  delay(50);
}

void action(int command)
{
  if(command == 65)
  {
      moveForward(forward_speed1, forward_speed2);
  }
  else if(command == 66)
  {
      moveBackward(backward_speed1, backward_speed2);
  }
  else if(command == 67)
  {
      spinLeft(turn_speed);
  }
  else if(command == 68)
  {
      spinRight(turn_speed);
  }
  else if(command == 69) 
  {
      loadBlock();
  }
  else if (command == 70)
  {
      unloadBlock();
  }
  else
  {
      brake();
  }
}

void moveForward(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);
}
void moveBackward(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);
}
void spinLeft(int turn_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, turn_speed);
  analogWrite(LeftMotorPWM, turn_speed);
}
void spinRight(int turn_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, turn_speed);
  analogWrite(LeftMotorPWM, turn_speed);
}
void brake()
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, HIGH);
  digitalWrite(LeftMotorIn2, HIGH);

  analogWrite(RightMotorPWM, 0);
  analogWrite(LeftMotorPWM, 0);
}
void loadBlock()
{
  gripper.write(finalPos);
  delay(1000);
}
void unloadBlock()
{
  gripper.write(startPos);
  delay(1000);
}

