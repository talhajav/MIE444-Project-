#include <SoftwareSerial.h>
#include <Servo.h>

// bluetooth
#define RXpin 10 // bluetooth's TX pin goes here
#define TXpin 11 // bluetooth's RX pin goes here
SoftwareSerial BTSerial(RXpin, TXpin); // RX | TX

// servo motors
#define ServoPin 8
Servo servo;

// dc motor
#define RightMotorIn1 2
#define RightMotorIn2 3
#define LeftMotorIn1 4
#define LeftMotorIn2 5
#define RightMotorPWM 6
#define LeftMotorPWM 7

// function declarations
void action();
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void loadBox();
void unloadBox();

int motor_speed = 100;
int data;
int command = 0;

void setup() {
  Serial.begin(9600); 
  BTSerial.begin(9600); 

  pinMode(RightMotorIn1, OUTPUT); // Right Motor
  pinMode(RightMotorIn2, OUTPUT);
  pinMode(LeftMotorIn1, OUTPUT); // Left motor
  pinMode(LeftMotorIn2, OUTPUT);

  servo.attach(ServoPin); // Gripper Servo
}
void loop() {
  if(BTSerial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
      data = BTSerial.read(); // Reads the data from the bluetooth serial port
      if(data >= 65 && data <= 72)
        command = data;
  } 
  action(command);  
}

void action(int command)
{
  if(command == 65)
  {
      moveForward(motor_speed);
  }
  else if(command == 66)
  {
      moveBackward(motor_speed);
  }
  else if(command == 67)
  {
      spinLeft(motor_speed);
  }
  else if(command == 68)
  {
      spinRight(motor_speed);
  }
  else if(command == 69) 
  {
      loadBox();
  }
  else if (command == 70)
  {
      unloadBox();
  }
  else
  {
      brake();
  }
}

void moveForward(int motor_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);

  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, motor_speed);
  analogWrite(LeftMotorPWM, motor_speed);
}
void moveBackward(int motor_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(RightMotorIn2, HIGH);

  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(LeftMotorIn2, HIGH);

  analogWrite(RightMotorPWM, motor_speed);
  analogWrite(LeftMotorPWM, motor_speed);
}
void spinLeft(int motor_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);

  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(LeftMotorIn2, HIGH);

  analogWrite(RightMotorPWM, motor_speed);
  analogWrite(LeftMotorPWM, motor_speed);
}
void spinRight(int motor_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(RightMotorIn2, HIGH);

  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, motor_speed);
  analogWrite(LeftMotorPWM, motor_speed);
}
void brake()
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);

  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(LeftMotorIn2, LOW);
}
void loadBox()
{
}
void unloadBox()
{
}

