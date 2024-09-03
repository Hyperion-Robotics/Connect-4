#include <Wire.h>



const int gate1pin = 34;  
const int gate2pin = 32;  
const int gate3pin = 35;  
const int gate4pin = 33;  
const int gate5pin = 25;  
const int gate6pin = 26;  
const int gate7pin = 27;  
const int pumpPin = 19;
const int ledPin = 18;
const int stepPin=16;
const int dirPin=4;

int sensorValue=0;

bool pumpState = false;

byte receivedHex;
bool recieved_status = false;
byte hexGate;

volatile int current_gate=0;
int last_gate=0;
volatile long last_gate_timestamp=0;

volatile int gateCounters[]={0,0,0,0,0,0,0};



void setup() {
  // Initialize serial communications at 115200 bps:
  Serial.begin(9600);

  // Set the gate pins as input:
  pinMode(gate1pin, INPUT);
  pinMode(gate2pin, INPUT);
  pinMode(gate3pin, INPUT);
  pinMode(gate4pin, INPUT);
  pinMode(gate5pin, INPUT);
  pinMode(gate6pin, INPUT);
  pinMode(gate7pin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);


  // Attach interrupts to the gate pins:
  // attachInterrupt(digitalPinToInterrupt(gate1pin), gate1Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate2pin), gate2Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate3pin), gate3Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate4pin), gate4Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate5pin), gate5Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate6pin), gate6Intr, RISING);
  // attachInterrupt(digitalPinToInterrupt(gate7pin), gate7Intr, RISING);

  Serial.flush();


}

void loop() {
  readSerialPort();
    if(recieved_status==true){
      switch (receivedHex){
        case 0xB1:
          Serial.write(0xB1);
          Serial.flush();
          break;
        case 0xA1:
          // Serial.println("Got data");
          togglePumps(0);
          break;
        case 0xA2:
          // Serial.println("Got data");
          togglePumps(1);
          break;
        case 0xA3:
          // Serial.println("Got data");
          togglePumps(2);
          break;
        case 0xA4:
          stepperRotate(1,475);
          break;
        case 0xA5:
          stepperRotate(2,475);
          break;
        case 0xA6:
          stepperRotate(1,475*11);
          break;
        case 0xA7:
          stepperRotate(2,475*11);
          break;
        case 0xA8:
          for(int i=0;i<7;i++)gateCounters[i]=0;
          break;
          
      }
      Serial.flush();
      recieved_status=false;
    }

  sensorValue = analogRead(gate1pin);
  if(sensorValue>2000 && gateCounters[0]<6){
    current_gate=1;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate2pin);
  if(sensorValue>2000 && gateCounters[1]<6){
    current_gate=2;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate3pin);
  if(sensorValue>2000 && gateCounters[2]<6){
    current_gate=3;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate4pin);
  if(sensorValue>2000 && gateCounters[3]<6){
    current_gate=4;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate5pin);
  if(sensorValue>2000 && gateCounters[4]<6){
    current_gate=5;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate6pin);
  if(sensorValue>2000 && gateCounters[5]<6){
    current_gate=6;
    last_gate_timestamp=millis();
  }

  sensorValue = analogRead(gate7pin);
  if(sensorValue>2000 && gateCounters[6]<6){
    current_gate=7;
    last_gate_timestamp=millis();
  }


  if(last_gate!=current_gate){    
    last_gate=current_gate;
    if(current_gate==0){
      // Serial.println("Gate timeout occured");
    }
    else{
      // Serial.print("Gate ");
      // Serial.print(current_gate);
      // Serial.println(" triggered"); 
      gateCounters[current_gate-1]++;
      Serial.write(current_gate);
      Serial.flush();
  }

  }
  if(millis()-last_gate_timestamp>500){
    current_gate=0;
  }

  delay(20);
}

// void gate1Intr() {
//   if(gateCounters[0]<6){
  // current_gate=1;
  // last_gate_timestamp=millis();
//   }
// }

// void gate2Intr() {
//   if(gateCounters[1]<6){
//   current_gate=2;
//   last_gate_timestamp=millis();
//   }
// }

// void gate3Intr() {
//   if(gateCounters[2]<6){
//   current_gate=3;
//   last_gate_timestamp=millis();
//   }
// }

// void gate4Intr() {
//   if(gateCounters[3]<6){
//   current_gate=4;
//   last_gate_timestamp=millis();
//   }
// }

// void gate5Intr() {
//   if(gateCounters[4]<6){
//   current_gate=5;
//   last_gate_timestamp=millis();
//   }
// }

// void gate6Intr() {
//   if(gateCounters[5]<6){
//   current_gate=6;
//   last_gate_timestamp=millis();
//   }
// }

// void gate7Intr() {
//   if(gateCounters[6]<6){
//   current_gate=7;
//   last_gate_timestamp=millis();
//   }
// }

void togglePumps(int mode){
  if(mode==1){
    pumpState=true;
    digitalWrite(pumpPin,HIGH);
    digitalWrite(ledPin,HIGH);
    // Serial.println("2,Turning pumps on");
    }
  else if(mode==0){
    pumpState=false;
    digitalWrite(pumpPin,LOW);
    digitalWrite(ledPin,LOW);
    // Serial.println("2,Turning pumps off");
  }
  else if(mode==2){
    if(pumpState==false){
      pumpState=true;
      digitalWrite(pumpPin,HIGH);
      digitalWrite(ledPin,HIGH);
      // Serial.println("2,Turning pumps on");
    }
    else if(pumpState==true){
      pumpState=false;
      digitalWrite(pumpPin,LOW);
      digitalWrite(ledPin,LOW);
      // Serial.println("2,Turning pumps off");
    }
  }
}

void stepperRotate(int dir, int steps) {
  if(dir==1)digitalWrite(dirPin,HIGH);
  else if(dir==2)digitalWrite(dirPin,LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delay(1); // Adjust this value to control the speed of the motor
    digitalWrite(stepPin, LOW);
    delay(1); // Adjust this value to control the speed of the motor
  }
}

void readSerialPort(){
   if (Serial.available() > 0) {
    recieved_status=true;
    // Read the incoming byte
    receivedHex = Serial.read();
    
    // Print the received character to the serial monitor
    // Serial.print("Received: ");
    // Serial.print(receivedHex);
    
    // Optionally, do something with the received data
    // For example, toggle an LED or store the value
  }
}
