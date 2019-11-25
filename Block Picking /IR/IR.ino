const int IR_1 = 32; // Front right
const int IR_2 = 33; // Front left
const int IR_3 = 30; // Back right
const int IR_4 = 31; // Back left


int IR_Sensors [4] = {IR_1, IR_2, IR_3, IR_4 };
static int IR_Readings [4] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  pinMode (IR_1, INPUT);
  pinMode (IR_2, INPUT);
  pinMode (IR_3, INPUT);
  pinMode (IR_4, INPUT);
  Serial.begin(9600);

}
//1 = black 0 = white 
int *IR_Read() {
  static int IR_Readings [4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++){
    IR_Readings [i] = digitalRead(IR_Sensors[i]);
  }
 delay(500);
 return IR_Readings;
}

void printReadings(int *IR_sensors){
  
  Serial.print("Front Right: "); 
  Serial.println(IR_sensors[0]);  
  Serial.print("Front Left: "); 
  Serial.println(IR_sensors[1]);  
  Serial.print("Back Right: "); 
  Serial.println(IR_sensors[2]);
  Serial.print("Back Left: "); 
  Serial.println(IR_sensors[3]);    
  Serial.println();
  
  }

void sendToPython(int IR_sensors[]){

 Serial.println((String)"SensorReading"+":"+IR_sensors[0] +":"+ IR_sensors[1] +":"+ IR_sensors[2] +":"+ IR_sensors[3]);
  
  }

void loop() {
  // put your main code here, to run repeatedly:
 int *IR_sensors=IR_Read();
 int IRArray [] = {0,0,0,0};
 
 IRArray[0]=IR_sensors[0];
 IRArray[1]=IR_sensors[1];
 IRArray[2]=IR_sensors[2];
 IRArray[3]=IR_sensors[3];
 Serial.println("Test "); 
 sendToPython(IRArray);
   
  
 
  
}
