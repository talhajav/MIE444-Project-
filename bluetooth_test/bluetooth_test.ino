int data;
int command = 0;

void setup() {
  Serial.begin(9600); 
}
void loop() {
  if(Serial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
      data = Serial.read(); // Reads the data from the bluetooth serial port
      if((data >= 65 && data <= 72) || (data >= 97 && data <= 104))
        command = data;
  }
  Serial.println(command);
}
