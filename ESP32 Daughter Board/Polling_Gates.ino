
const int gate1pin = 34;  
const int gate2pin = 35;  
const int gate3pin = 32;  
const int gate4pin = 33;  
const int gate5pin = 25;  
const int gate6pin = 26;  
const int gate7pin = 27;  

int sensorValue = 0;  // value read from the pot

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  pinMode(gate1pin, INPUT);
  pinMode(gate2pin, INPUT);
  pinMode(gate3pin, INPUT);
  pinMode(gate4pin, INPUT);
  pinMode(gate5pin, INPUT);
  pinMode(gate6pin, INPUT);
  pinMode(gate7pin, INPUT);
  
  
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(gate1pin);
  if(sensorValue>2000)Serial.println("Gate 1 broken");

  sensorValue = analogRead(gate2pin);
  if(sensorValue>2000)Serial.println("Gate 2 broken");

  sensorValue = analogRead(gate3pin);
  if(sensorValue>2000)Serial.println("Gate 3 broken");

  sensorValue = analogRead(gate4pin);
  if(sensorValue>2000)Serial.println("Gate 4 broken");

  sensorValue = analogRead(gate5pin);
  if(sensorValue>2000)Serial.println("Gate 5 broken");

  sensorValue = analogRead(gate6pin);
  if(sensorValue>2000)Serial.println("Gate 6 broken");

  sensorValue = analogRead(gate7pin);
  if(sensorValue>2000)Serial.println("Gate 7 broken");

  // Serial.print("sensor = ");
  // Serial.println(sensorValue);
  delay(20);
}



