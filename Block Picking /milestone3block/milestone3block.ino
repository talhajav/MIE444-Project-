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
int forward_speed1 = 140; // right motor
int forward_speed2 = 140; // left motor
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
#define BLOCK_SENSOR 37 //BROWN 

NewPing sonar_north(TRIGGER_PIN, ECHO_PIN_NORTH, MAX_DISTANCE);
NewPing sonar_east1(TRIGGER_PIN, ECHO_PIN_EAST1, MAX_DISTANCE);
NewPing sonar_east2(TRIGGER_PIN, ECHO_PIN_EAST2, MAX_DISTANCE);
NewPing sonar_south(TRIGGER_PIN, ECHO_PIN_SOUTH, MAX_DISTANCE);
NewPing sonar_west1(TRIGGER_PIN, ECHO_PIN_WEST1, MAX_DISTANCE);
NewPing sonar_west2(TRIGGER_PIN, ECHO_PIN_WEST2, MAX_DISTANCE);
NewPing blockSensorUS(TRIGGER_PIN, BLOCK_SENSOR,   200);

                    // 0 - north , 1 - east1, 2 - east2, 3 - south, 4 - west1, 5 - west2
int sonar_arr[] = {0, 0, 0, 0, 0, 0};
int prev_sonar_arr[] = {0, 0, 0, 0, 0, 0};
int sonar_delay = 10; // ms
int sonar_avg = 10; // average x readings
int north_threshold = 15; // cm
int east_threshold = 10; // cm
int west_threshold = 10; // cm
int blockSensor=0;
int southSensor=0;
int scanSpeed=90;


// tracker variables
bool moving = false;
bool loaded = false;
int current_speed1 = 0;
int current_speed2 = 0;

//Block tracker variables 
bool block;
bool blockFound = false;
bool blockPicked = false;


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
   //spinRight(scanSpeed,scanSpeed);
   //delay(1000);
// loadBlock();
//  delay(1000);

 // getSensorReadings(true);
 
 getBlock();
 delay(500000);
 
   //unloadBlock();
  //delay(1000);
 //moveStraightForward();
 //moveStraightBackward();
// moveForward(50,50);
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
  if(sonar_arr[1] <= 10 && sonar_arr[4] <= 10)
  {
    Serial.println("Adjust robot toward equidistance");
    if(sonar_arr[1] > sonar_arr[4] + 1) // east > west
      adjustRight(true);
    else if(sonar_arr[4] > sonar_arr[1] + 1) // west > east
      adjustLeft(true);
  }
  else if(sonar_arr[1] <= 10)
  {
    Serial.println("Hugging east wall");
    if(sonar_arr[1] > east_threshold + 1 and sonar_arr[2] <= sonar_arr[1])
      adjustRight(true);
    else if(sonar_arr[1] < east_threshold)
      while(sonar_arr[1] <= sonar_arr[2])
        adjustLeft(true);
  }
  else if(sonar_arr[4] <= 10)
  {
    Serial.println("Hugging west wall");
    if(sonar_arr[4] > west_threshold + 1 and sonar_arr[5] <= sonar_arr[4])
      adjustLeft(true);
    else if(sonar_arr[4] < west_threshold)
      while(sonar_arr[4] <= sonar_arr[5])
        adjustRight(true);
  }
  
  if(sonar_arr[0] < north_threshold)
    Serial.println("Obstacle in front of robot detected");
  else
  {
    moveForward(forward_speed1, forward_speed2);
    delay(400);
    brake();
  }
}
void adjustLeft(bool backward)
{
  if(backward)
    spinLeft(turn_speed1, 0); // <-- why are they switched? Need to figure out ASAP.
  else
    spinLeft(0, turn_speed2);
  delay(200);
  brake();  
  getSensorReadings(true);
}

void adjustRight(bool backward)
{
  if(backward)
    spinRight(0, turn_speed2); // <-- why are they switched? Need to figure out ASAP.
  else
    spinRight(turn_speed1, 0);
  delay(200);
  brake();  
  getSensorReadings(true);
}

void moveStraightBackward()
{
  if(sonar_arr[1] <= 10 && sonar_arr[4] <= 10)
  {
    Serial.println("Adjust robot toward equidistance");
    if(sonar_arr[1] > sonar_arr[4] + 1) // east > west
      adjustRight(true);
    else if(sonar_arr[4] > sonar_arr[1] + 1) // west > east
      adjustLeft(true);
  }
  else if(sonar_arr[1] <= 10)
  {
    Serial.println("Hugging east wall");
    if(sonar_arr[1] > east_threshold + 1 and sonar_arr[2] <= sonar_arr[1])
      adjustRight(true);
    else if(sonar_arr[1] < east_threshold)
      while(sonar_arr[1] <= sonar_arr[2])
        adjustLeft(true);
  }
  else if(sonar_arr[4] <= 10)
  {
    Serial.println("Hugging west wall");
    if(sonar_arr[4] > west_threshold + 1 and sonar_arr[5] <= sonar_arr[4])
      adjustLeft(true);
    else if(sonar_arr[4] < west_threshold)
      while(sonar_arr[4] <= sonar_arr[5])
        adjustRight(true);
  }
  
  if(sonar_arr[0] < north_threshold)
    Serial.println("Obstacle in front of robot detected");
  else
  {
    moveForward(forward_speed1, forward_speed2);
    delay(400);
    brake();
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

//Function that assumes that robot is at loading zone

void getBlock()
{
 

  
//  //Detecting block while scanning
  while (!blockFound)
   {
      
      southSensor = sonar_north.ping_cm();
      delay(10);
      blockSensor = blockSensorUS.ping_cm();
      delay(10);
      
      Serial.print("Block Sensor reading: ");
      Serial.println(blockSensor);
      Serial.print("South Sensor reading: ");
      Serial.println(southSensor);
      
    //these numbers can be played around with
      if (blockSensor < 20 && southSensor > blockSensor+5 && blockSensor != 0){
        Serial.println("Block has been found. Moving fwd ");
        moveBackward(forward_speed1,forward_speed2);
       // moveStraightBackward();
        delay(100);
        brake();
        blockFound = true;
        break;
          }
     Serial.println("Block has NOT been found. Moving fwd and scanning");
     moveBackward(forward_speed1,forward_speed2);
     //moveStraightBackward();
     delay(200);
    //Serial.println("Spinning");
     //adjustRight(true);
    //delay(500);
      
    }
////printing distance for tuning of algo 
  
  do{ //change to condition where top sensor also showing a larger distance so we know that it is block and not something else 
      southSensor = sonar_north.ping_cm();
      delay(10);
      blockSensor = blockSensorUS.ping_cm();
      delay(10);
      moveBackward(forward_speed1,forward_speed2);
      delay(100);
      Serial.println("Moving towards block for pickup");
      
      if(blockSensor <= 3 && blockSensor != 0){
          Serial.println("Braking for pickup");
          brake();
          loadBlock();
          blockPicked = true;
          brake();
          delay(100);
          Serial.println("Block has been picked up");
          break;
        }
      
    }while (blockFound && !blockPicked);
  }
