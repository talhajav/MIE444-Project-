#include <SoftwareSerial.h>

// bluetooth
#define RXpin 10 // bluetooth's TX pin goes here
#define TXpin 11 // bluetooth's RX pin goes here
SoftwareSerial BTSerial(RXpin, TXpin); // RX | TX

int data;
int command = 0;

void setup() {
  Serial.begin(9600); 
  BTSerial.begin(9600); 
}
void loop() {
  if(BTSerial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
      data = BTSerial.read(); // Reads the data from the bluetooth serial port
      if((data >= 65 && data <= 72) || (data >= 97 && data <= 104))
        command = data;
  }
  Serial.println(command);
}
