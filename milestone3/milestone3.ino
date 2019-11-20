#include <NewPing.h>
#include <Servo.h>

// servo motors
#define gripperPin 6
Servo gripper;

// servo position
int startPos = 170;
int finalPos = startPos - 105;

// dc motor
#define LeftMotorIn1 12
#define LeftMotorIn2 9 // Brake-A
#define RightMotorIn1 13
#define RightMotorIn2 8 // Brake-B
#define RightMotorPWM 3
#define LeftMotorPWM 11

// motor speed
int forward_speed1 = 150; // right motor
int forward_speed2 = 150; // left motor
int backward_speed1 = 125; // right motor
int backward_speed2 = 125; // left motor
int turn_speed1 = 125;  // right motor
int turn_speed2 = 125; // left motor

// ultrasonic sensor
#define TRIGGER_PIN    52 // green
#define ECHO_PIN_SOUTH 49 // blue
#define ECHO_PIN_WEST2 46 // white
#define ECHO_PIN_WEST1 50 // yellow
#define ECHO_PIN_NORTH 48 // orange
#define ECHO_PIN_EAST2 53 // brown/grey
#define ECHO_PIN_EAST1 51 // purple
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

// obstacle variables
int north_threshold = 10; // cm
int east_threshold = 5; // cm
int west_threshold = 5; // cm
int min_threshold = 3; // cm
bool forward_path_is_clear = true;
bool north_danger = false;
bool south_danger = false;
bool east_danger = false;
bool west_danger = false;

// tracker variables
bool moving = false;
bool loaded = false;
int current_speed1 = 0;
int current_speed2 = 0;

// path planning variables
                      // w - forward, a - turn left, d - turn right, s - turn backward, e - end
char directions[] = {'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e'} // stops at every environment change (wall configuration changes)

// function declaration
void getSensorReadings();
void printSensorReadings();
void detectCollision();
void action();
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void loadBlock();
void unloadBlock();
void moveStraightForward();
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
void detectCollision()
{
  // see if sensor readings are below/equal to the minimum threshold
  if(sonar_arr[0] <= min_threshold)
  {
    Serial.println("Collision north detected");
    forward_path_is_clear = false;
    north_danger = true;
  }
  if(sonar_arr[1] <= min_threshold || sonar_arr[2] <= min_threshold)
  {
    Serial.println("Collision east detected");
    east_danger = true;
  }
  if(sonar_arr[3] <= min_threshold)
  {
    Serial.println("Collision south detected");
    south_danger = true;
  }
  if(sonar_arr[4] <= min_threshold || sonar_arr[5] <= min_threshold)
  {
    Serial.println("Collision west detected");
    west_danger = true;
  }
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
  if(sonar_arr[1] < east_threshold && sonar_arr[2] >= sonar_arr[1])
  {
    Serial.println("Robot's front is veering to the right. Adjusting towards left");
    while(sonar_arr[1] < east_threshold)
    {
      spinLeft(turn_speed1, 0); // <-- why are they switched? Need to figure out ASAP.
      delay(400);
      brake();
      getSensorReadings(true);
    }
    if(sonar_arr[0] > north_threshold)
    {
      moveForward(forward_speed1, forward_speed2);
      delay(400);
      brake();
      getSensorReadings(true);
    }
  }

  if(sonar_arr[4] < west_threshold && sonar_arr[5] >= sonar_arr[4])
  {
    Serial.println("Robot's front is veering to the left. Adjusting towards right");
    while(sonar_arr[4] < west_threshold)
    {
      spinRight(0, turn_speed2); // <-- why are they switched? Need to figure out ASAP.
      delay(400);
      brake();  
      getSensorReadings(true);
    }
    if(sonar_arr[0] > north_threshold)
    {
      moveForward(forward_speed1, forward_speed2);
      delay(400);
      brake();
      getSensorReadings(true);
    }
  }
  
  if(sonar_arr[0] < north_threshold)
  {
    Serial.println("Obstacle in front of robot detected");
    forward_path_is_clear = false;
  }
  else
  {
    moveForward(forward_speed1, forward_speed2);
    delay(400);
    brake();
  }
}
void turnLeft()
{
  int count = 0;
  while(true)
  {
    getSensorReadings(true);
    
    detectCollision();
    if(north_danger || south_danger || east_danger || west_danger)
      break;

    if(sonar_arr[0] > north_threshold && count > 5 && abs(sonar_arr[1] - sonar_arr[2]) < 2 && sonar_arr[3] < 10)
    {
      forward_path_is_clear = true;
      break;
    }

    spinLeft(turn_speed1, turn_speed2);
    delay(200);
    brake();
    
    count++;
  }
}
void turnRight()
{
  int count = 0;
  while(true)
  {
    getSensorReadings(true);
    
    detectCollision();
    if(north_danger || south_danger || east_danger || west_danger)
      break;

    if(sonar_arr[0] > north_threshold && count > 5 && abs(sonar_arr[4] - sonar_arr[5]) < 2 && sonar_arr[3] < 10)
    {
      forward_path_is_clear = true;
      break;
    }

    spinRight(turn_speed1, turn_speed2);
    delay(200);
    brake();
    
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
