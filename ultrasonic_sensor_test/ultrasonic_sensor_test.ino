#include <NewPing.h>

// Declare ultrasonic sensor variables
#define TRIGGER_PIN    52
#define ECHO_PIN_NORTH 49
#define ECHO_PIN_EAST1 46
#define ECHO_PIN_EAST2 50
#define ECHO_PIN_SOUTH 48
#define ECHO_PIN_WEST1 53
#define ECHO_PIN_WEST2 51
#define MAX_DISTANCE 200

NewPing sonar_north(TRIGGER_PIN, ECHO_PIN_NORTH, MAX_DISTANCE);
NewPing sonar_east1(TRIGGER_PIN, ECHO_PIN_EAST1, MAX_DISTANCE);
NewPing sonar_east2(TRIGGER_PIN, ECHO_PIN_EAST2, MAX_DISTANCE);
NewPing sonar_south(TRIGGER_PIN, ECHO_PIN_SOUTH, MAX_DISTANCE);
NewPing sonar_west1(TRIGGER_PIN, ECHO_PIN_WEST1, MAX_DISTANCE);
NewPing sonar_west2(TRIGGER_PIN, ECHO_PIN_WEST2, MAX_DISTANCE);
                    // 0 - north , 1 - east1, 2 - east2, 3 - south, 4 - west1, 5 - west2
int sonar_arr[] = {0, 0, 0, 0, 0, 0};
int sonar_delay = 30;

// function declaration
void getSensorReadings();
void printSensorReadings();

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  getSensorReadings();
  printSensorReadings();
  delay(500);
}

void getSensorReadings()
{
  sonar_arr[0] = sonar_north.ping_cm();
  delay(sonar_delay);
  sonar_arr[1] = sonar_east1.ping_cm();
  delay(sonar_delay);
  sonar_arr[2] = sonar_east2.ping_cm();
  delay(sonar_delay);
  sonar_arr[3] = sonar_south.ping_cm();
  delay(sonar_delay);
  sonar_arr[4] = sonar_west1.ping_cm();
  delay(sonar_delay);
  sonar_arr[5] = sonar_west2.ping_cm();
  delay(sonar_delay);
}

void printSensorReadings()
{
  Serial.println((String)"North: "+sonar_arr[0]+". East1: "+sonar_arr[1]+". East2: "+sonar_arr[2]+
                         ". South: "+sonar_arr[3]+". West1: "+sonar_arr[4]+". West2: "+sonar_arr[5]);
}
