//int data = 0;
//
//void setup() {
//  Serial.begin(9600); 
//}
//void loop() {
//  if(Serial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
//      data = Serial.read(); // Reads the data from the bluetooth serial port
//  }
//  Serial.println(data);
//  if(data == 65)
//    Serial.println("Recieved Command!");
//}

//int data = 0;
//
//void setup() {
//  Serial.begin(9600);
//  Serial1.begin(9600); 
//}
//void loop() {
//  if(Serial1.available() > 0){ // Checks whether data is comming from the bluetooth serial port
//      data = Serial1.read(); // Reads the data from the bluetooth serial port
//  }
//  Serial.println(data);
//  Serial1.println(data);
//  if(data == 65)
//  {
//    Serial.println("Recieved Command!");
//    Serial1.println("Recieved Command!");
//  }
//}

// Basic Bluetooth sketch BT_TALK
// Connect the module and communicate using the serial monitor
// Communicate with the BT module at 9600 (comms mode)

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX
 
char c = ' ';
 
void setup() 
{
    Serial.begin(9600);
    Serial.println("Arduino is ready");
    Serial.println("Remember to select Both NL & CR in the serial monitor");
 
    // HC-05 default serial speed for Command mode is 9600
    BTserial.begin(9600);  
    BTserial.write("testing");
}
 
void loop()
{   

    // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BTserial.available())
    {  
        c = BTserial.read();
        Serial.println(c);
    }

    delay(1000);
    
    BTserial.write("Ayy lmao.\n");
//    delay(1000);
}
