#include <NewPing.h>
#include <Servo.h>

// servo motors
#define gripperPin 6
Servo gripper;

// servo position
int startPos = 140;
int finalPos = startPos - 80;

// dc motor
#define LeftMotorIn1 12
#define LeftMotorIn2 9 // Brake-A
#define RightMotorIn1 13
#define RightMotorIn2 8 // Brake-B
#define RightMotorPWM 3
#define LeftMotorPWM 11

// motor speed
int forward_speed1 = 100; // right motor
int forward_speed2 = 150; // left motor
int backward_speed1 = 100; // right motor
int backward_speed2 = 150; // left motor
int turn_speed1 = 80;  // right motor
int turn_speed2 = 100; // left motor
//int forward_speed1 = 200; // right motor
//int forward_speed2 = 200; // left motor
//int backward_speed1 = 200; // right motor
//int backward_speed2 = 200; // left motor
//int turn_speed1 = 200;  // right motor
//int turn_speed2 = 200; // left motor

// ultrasonic sensor
#define TRIGGER_PIN    52 // green
#define ECHO_PIN_NORTH 49
#define ECHO_PIN_EAST1 46 // white
#define ECHO_PIN_EAST2 50 // yellow
#define ECHO_PIN_SOUTH 48 // orange
#define ECHO_PIN_WEST1 53 // brown/grey
#define ECHO_PIN_WEST2 51 // purple
#define MAX_DISTANCE 400

NewPing sonar_north(TRIGGER_PIN, ECHO_PIN_NORTH, MAX_DISTANCE);
NewPing sonar_east1(TRIGGER_PIN, ECHO_PIN_EAST1, MAX_DISTANCE);
NewPing sonar_east2(TRIGGER_PIN, ECHO_PIN_EAST2, MAX_DISTANCE);
NewPing sonar_south(TRIGGER_PIN, ECHO_PIN_SOUTH, MAX_DISTANCE);
NewPing sonar_west1(TRIGGER_PIN, ECHO_PIN_WEST1, MAX_DISTANCE);
NewPing sonar_west2(TRIGGER_PIN, ECHO_PIN_WEST2, MAX_DISTANCE);
                    // 0 - north , 1 - east1, 2 - east2, 3 - south, 4 - west1, 5 - west2
int sonar_arr[] = {0, 0, 0, 0, 0, 0};
int prev_sonar_arr[] = {0, 0, 0, 0, 0, 0};
int sonar_delay = 10; // ms
int sonar_avg = 10; // average x readings
int north_threshold = 15; // cm
int east_threshold = 10; // cm
int west_threshold = 10; // cm

// tracker variables
bool moving = false;
bool loaded = false;
int current_speed1 = 0;
int current_speed2 = 0;

// function declaration
void getSensorReadings();
void printSensorReadings();
void action();
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void loadBlock();
void unloadBlock();
void turnLeft(); // 90 degrees
void turnRight(); // 90 degrees
void testMoveForward();
void testMoveBackward();
void testSpinLeft();
void testSpinRight();

void setup()
{
  Serial.begin(9600);

  pinMode(RightMotorIn1, OUTPUT); // Right Motor
  pinMode(RightMotorIn2, OUTPUT);
  pinMode(LeftMotorIn1, OUTPUT); // Left motor
  pinMode(LeftMotorIn2, OUTPUT);

  gripper.attach(gripperPin); // Gripper Servo
  unloadBlock(); // start at initial position
}

void loop()
{
  getSensorReadings(true);
  moveStraightForward();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// ULTRASONIC SENSOR FUNCTIONS ///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getSensorReadings(bool average_the_readings)
{
  if(average_the_readings)
  {
    // average x readings
    int d0=0, d1=0, d2=0, d3=0, d4=0, d5=0;
    for(int i=0; i<sonar_avg; i++)
    {
      d0 += sonar_north.ping_cm();
      delay(sonar_delay);
      d1 += sonar_east1.ping_cm(); 
      delay(sonar_delay);
      d2 += sonar_east2.ping_cm();
      delay(sonar_delay);
      d3 += sonar_south.ping_cm();
      delay(sonar_delay);
      d4 += sonar_west1.ping_cm();
      delay(sonar_delay);
      d5 += sonar_west2.ping_cm();
      delay(sonar_delay);
    }
    sonar_arr[0] = d0 / sonar_avg;
    sonar_arr[1] = d1 / sonar_avg;
    sonar_arr[2] = d2 / sonar_avg;
    sonar_arr[3] = d3 / sonar_avg;
    sonar_arr[4] = d4 / sonar_avg;
    sonar_arr[5] = d5 / sonar_avg;
  }
  else
  {
    sonar_arr[0] = sonar_north.ping_cm();
    delay(sonar_delay);
    sonar_arr[1] = sonar_east1.ping_cm(); // east1 very reliable
    delay(sonar_delay);
    sonar_arr[2] = sonar_east2.ping_cm(); // east2 not as reliable
    delay(sonar_delay);
    sonar_arr[3] = sonar_south.ping_cm(); // south very reliable
    delay(sonar_delay);
    sonar_arr[4] = sonar_west1.ping_cm(); // west1 not as reliable
    delay(sonar_delay);
    sonar_arr[5] = sonar_west2.ping_cm(); // west2 not as reliable
    delay(sonar_delay);
  }
  printSensorReadings();
}

void printSensorReadings()
{
  Serial.println((String)"North: "+sonar_arr[0]+". East1: "+sonar_arr[1]+". East2: "+sonar_arr[2]+
                         ". South: "+sonar_arr[3]+". West1: "+sonar_arr[4]+". West2: "+sonar_arr[5]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// BASIC MOTION FUNCTIONS /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveForward(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);

  moving = true;
  current_speed1 = right_motor_speed;
  current_speed2 = left_motor_speed;
}
void moveBackward(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);

  moving = true;
  current_speed1 = right_motor_speed;
  current_speed2 = left_motor_speed;
}
void spinLeft(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, HIGH);
  digitalWrite(LeftMotorIn1, LOW);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);

  moving = true;
  current_speed1 = right_motor_speed;
  current_speed2 = left_motor_speed;
}
void spinRight(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(RightMotorIn1, LOW);
  digitalWrite(LeftMotorIn1, HIGH);
  digitalWrite(RightMotorIn2, LOW);
  digitalWrite(LeftMotorIn2, LOW);

  analogWrite(RightMotorPWM, right_motor_speed);
  analogWrite(LeftMotorPWM, left_motor_speed);

  moving = true;
  current_speed1 = right_motor_speed;
  current_speed2 = left_motor_speed;
}
void brake()
{ 
  if(moving)
  {
    // decelerate
    for(int i=1; i<4; i++)
    {
      digitalWrite(RightMotorIn1, HIGH);
      digitalWrite(LeftMotorIn1, HIGH);
      digitalWrite(RightMotorIn2, HIGH);
      digitalWrite(LeftMotorIn2, HIGH);
  
      analogWrite(RightMotorPWM, current_speed1 - i*current_speed1/4);
      analogWrite(LeftMotorPWM, current_speed2 - i*current_speed2/4);
  
      delay(250);
    }
  }
  else
  {
    digitalWrite(RightMotorIn1, HIGH);
    digitalWrite(LeftMotorIn1, HIGH);
    digitalWrite(RightMotorIn2, HIGH);
    digitalWrite(LeftMotorIn2, HIGH);

    analogWrite(RightMotorPWM, 0);
    analogWrite(LeftMotorPWM, 0);
  }

  moving = false;
  current_speed1 = 0;
  current_speed2 = 0;
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// ADVANCED MOTION FUNCTIONS ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveStraightForward()
{
  // TODO: make delays into while loop statements
  
  if(sonar_arr[1] < east_threshold && sonar_arr[2] >= sonar_arr[1])
  {
    Serial.println("Robot's front is veering to the right. Adjusting towards left");
    spinLeft(turn_speed1, 0); // <-- why are they switched? Need to figure out ASAP.
    delay(400);
    brake();
    delay(100);
    getSensorReadings(true);
    if(sonar_arr[0] > north_threshold)
    {
      moveForward(forward_speed1, forward_speed2);
      delay(400);
      brake();
      delay(100);
      spinRight(0, turn_speed2); // <-- why are they switched? Need to figure out ASAP.
      delay(200);
      brake();
      delay(100);
      getSensorReadings(true);
    }
  }

  if(sonar_arr[4] < west_threshold && sonar_arr[5] >= sonar_arr[4])
  {
    Serial.println("Robot's front is veering to the left. Adjusting towards right");
    spinRight(0, turn_speed2); // <-- why are they switched? Need to figure out ASAP.
    delay(400);
    brake();
    delay(100);
    getSensorReadings(true);
    if(sonar_arr[0] > north_threshold)
    {
      moveForward(forward_speed1, forward_speed2);
      delay(400);
      brake();
      delay(100);
      spinLeft(turn_speed1, 0); // <-- why are they switched? Need to figure out ASAP.
      delay(200);
      brake();
      delay(100);
      getSensorReadings(true);
    }
  }
  
  if(sonar_arr[0] < north_threshold)
  {
    Serial.println("Obstacle in front of robot detected");
  }
  else
  {
    moveForward(forward_speed1, forward_speed2);
    delay(400);
    brake();
    delay(100);
  }
}
void turnLeft(int turn_speed1, int turn_speed2)
{
  int count = 0;
  while(true)
  {
    memcpy(prev_sonar_arr, sonar_arr, sizeof(sonar_arr[0])*6); 
    getSensorReadings(true);
  
    if(sonar_arr[0] > north_threshold && count > 5 && abs(sonar_arr[1] - sonar_arr[2]) < 2)
      break;

    spinLeft(turn_speed1, turn_speed2);
    moving = true;
    current_speed1 = turn_speed1;
    current_speed2 = turn_speed2;
    delay(200);
    brake();
    moving = false;
    delay(50);
    
    count++;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// TESTING MOTION FUNCTIONS ////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void testMoveForward()
{
  moveForward(forward_speed1, forward_speed2);
  delay(1000);
  brake();
  delay(500);
}
void testMoveBackward()
{
  moveBackward(backward_speed1, backward_speed2);
  delay(1000);
  brake();
  delay(500);
}
void testSpinLeft()
{
  spinLeft(turn_speed1, turn_speed2);
  delay(600);
  brake();
  delay(500);
}
void testSpinRight()
{
  spinRight(turn_speed1, turn_speed2);
  delay(600);
  brake();
  delay(500);
}
