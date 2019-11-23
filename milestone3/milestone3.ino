/*
 * TODO: implement equidistant readjustment function
 */

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
int forward_speed1 = 160; // right motor
int forward_speed2 = 160; // left motor
int backward_speed1 = 125; // right motor
int backward_speed2 = 125; // left motor
int turn_speed1 = 160;  // right motor
int turn_speed2 = 160; // left motor

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
int sonar_avg = 5; // average x readings

// obstacle variables
int north_threshold = 16; // cm
int south_threshold = 11; // cm
int east_threshold = 7; // cm
int west_threshold = 7; // cm
int min_threshold = 4; // cm
bool north_danger = false;
bool south_danger = false;
bool east_danger = false;
bool west_danger = false;

// tracker variables
bool moving = false;
bool turning = false;
bool loaded = false;
int current_speed1 = 0;
int current_speed2 = 0;
bool surrounding[] = {false, false, false, false}; // 0 - north, 1 - east, 2 - south, 4 - west
bool prev_surrounding[] = {false, false, false, false};
bool surrounding_changed = false;
int quadrant_type = 0; // 0 - intersection, 1 - T-junction, 2 - lane, 3 - alcove, 4 - corner

// path planning variables
                      // w - forward, a - turn left, d - turn right, s - turn backward, e - end
char directions[] = {'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e', 'e'}; // stops at every surrounding change (wall configuration changes)

// function declaration
void getSensorReadings();
void printSensorReadings();
void detectCollision();
void evalSurrounding();
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void loadBlock();
void unloadBlock();
void moveStraightForward();
void turnLeft(); // 90 degrees
void turnRight(); // 90 degrees
void adjustLeft();
void adjustRight();
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
  if(sonar_arr[0] < north_threshold or surrounding_changed)
    if(sonar_arr[4] > 12)
      turnLeft();
    else if(sonar_arr[1] > 12)
      turnRight();
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
  evalSurrounding();
}

void printSensorReadings()
{
  Serial.println((String)"North: "+sonar_arr[0]+". East1: "+sonar_arr[1]+". East2: "+sonar_arr[2]+
                         ". South: "+sonar_arr[3]+". West1: "+sonar_arr[4]+". West2: "+sonar_arr[5]);
}
void detectCollision()
{
  // reset variables
  north_danger = false;
  south_danger = false;
  east_danger = false;
  west_danger = false;
  
  // see if sensor readings are below/equal to the minimum threshold
  if(sonar_arr[0] < min_threshold)
  {
    Serial.println("Collision north detected");
    north_danger = true;
  }
  if(sonar_arr[1] < min_threshold or sonar_arr[2] < min_threshold)
  {
    Serial.println("Collision east detected");
    east_danger = true;
  }
  if(sonar_arr[3] < min_threshold + 2)
  {
    Serial.println("Collision south detected");
    south_danger = true;
  }
  if(sonar_arr[4] < min_threshold or sonar_arr[5] < min_threshold)
  {
    Serial.println("Collision west detected");
    west_danger = true;
  }
}

void evalSurrounding()
{
  // reset variable
  surrounding_changed = false; 
  
  surrounding[0] = sonar_arr[0] < north_threshold;
  surrounding[1] = sonar_arr[1] < 12 and sonar_arr[2] < 12;
  surrounding[2] = sonar_arr[3] < 20;
  surrounding[3] = sonar_arr[4] < 12 and sonar_arr[5] < 12;

  if(not (prev_surrounding[2] and surrounding[2]) and // if south was previously detected, wait till robot has sufficiently moved away from south wall
     not turning)
  {
    // check if wall configuration has changed
    for(int i=0; i<4; i++)
      if(prev_surrounding[i] != surrounding[i])
      {
        surrounding_changed = true;
        prev_surrounding[0] = surrounding[0];
        prev_surrounding[1] = surrounding[1];
        prev_surrounding[2] = surrounding[2];
        prev_surrounding[3] = surrounding[3];
        break;
      }
  }
  
  if(surrounding_changed)
  {
    // get quadrant type
    int count = 0;
    for(bool s: surrounding)
      if(s){count++;}
      
    if(count == 2 and (surrounding[0] or surrounding[2])) // corner
      quadrant_type = 4;
    else
      quadrant_type = count;
    Serial.println((String)"Surrounding changed! Now at quadrant type "+quadrant_type);
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
void turnLeft()
{
  if(sonar_arr[1] < 7) // need more clearance
  {
    Serial.println("Making adjustments before turning left");
    moveBackward(backward_speed1, backward_speed2);
    delay(200);
    adjustLeft(true);
    moveForward(forward_speed1, forward_speed2);
    delay(200);
    adjustRight(true);
    adjustRight(true);
    moveBackward(backward_speed1, backward_speed2);
    delay(200);
    adjustLeft(true);
    moveForward(forward_speed1, forward_speed2);
    delay(200);
  }

  Serial.println("Turning Left");
  turning = true;
  int count = 0;
  while(true)
  {
    getSensorReadings(true);
    
    detectCollision();
    if(south_danger)
      while(south_danger)
      {
        Serial.println("Adjusting away from south wall");
        adjustLeft(false);
        detectCollision();
      }

    if(quadrant_type == 0)
    {
      Serial.println("Intersection left turn");
      if(sonar_arr[0] > north_threshold and count > 3)
        break;
    }
    else if(quadrant_type == 1)
    {
      Serial.println("T-junction left turn");
      if(sonar_arr[0] > north_threshold and count > 3 and (abs(sonar_arr[1] - sonar_arr[2]) < 2 or sonar_arr[3] < 10))
        break;
    }
    else
    {
      if(sonar_arr[0] > north_threshold and // front path is clear
       count > 3 and // turned incrementally at least 3 times
       abs(sonar_arr[1] - sonar_arr[2]) < 2 and // the east sensors are parallel to the wall
       sonar_arr[3] < 10) // the back sensors detect a wall
        break;
    }

    spinLeft(turn_speed1, turn_speed2);
    delay(200);
    brake();
    
    count++;
  }
  Serial.println("Turning Completed");
  turning = false;
}

void turnRight()
{
  turning = true;
  int count = 0;
  while(true)
  {
    getSensorReadings(true);
    
    detectCollision();
    if(south_danger or east_danger or west_danger)
      break;

    if(sonar_arr[0] > north_threshold and // front path is clear
       count > 3 and // turned incrementally at least 3 times
       abs(sonar_arr[4] - sonar_arr[5]) < 2 and // the west sensors are parallel to the wall
       sonar_arr[3] < 10) // the back sensors detect a wall
      break;

    spinRight(turn_speed1, turn_speed2);
    delay(200);
    brake();
    
    count++;
  }
  turning = false;
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
