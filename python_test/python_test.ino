int data = 0;

void setup() {
  Serial.begin(9600); 
}
void loop() {
  if(Serial.available() > 0){ // Checks whether data is comming from the bluetooth serial port
      data = Serial.read(); // Reads the data from the bluetooth serial port
  }
  Serial.println(data);
}
