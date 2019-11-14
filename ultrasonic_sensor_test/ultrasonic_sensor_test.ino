#include <NewPing.h>

// Declare ultrasonic sensor variables
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
int sonar_delay = 10;
int sonar_avg = 10; // average x readings

// function declaration
void getSensorReadings();
void printSensorReadings();

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  getSensorReadings(true);
  printSensorReadings();
}

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
}

void printSensorReadings()
{
  Serial.println((String)"North: "+sonar_arr[0]+". East1: "+sonar_arr[1]+". East2: "+sonar_arr[2]+
                         ". South: "+sonar_arr[3]+". West1: "+sonar_arr[4]+". West2: "+sonar_arr[5]);
}
