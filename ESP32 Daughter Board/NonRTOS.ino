#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


const int gate1pin = 34;  
const int gate2pin = 32;  
const int gate3pin = 35;  
const int gate4pin = 33;  
const int gate5pin = 25;  
const int gate6pin = 26;  
const int gate7pin = 27;  
const int pumpPin = 19;
const int ledPin = 18;

bool pumpState = false;

String receivedCommand;
int recieved_status = 0;
int commands[8];
char data[20];

volatile int current_gate=0;
int last_gate=0;
volatile long last_gate_timestamp=0;

int tofValue=0;  // value read from the pot

void setup() {
  // Initialize serial communications at 115200 bps:
  Serial.begin(115200);

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


  // Attach interrupts to the gate pins:
  attachInterrupt(digitalPinToInterrupt(gate1pin), gate1Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate2pin), gate2Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate3pin), gate3Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate4pin), gate4Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate5pin), gate5Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate6pin), gate6Intr, RISING);
  attachInterrupt(digitalPinToInterrupt(gate7pin), gate7Intr, RISING);

  // Wire.begin();
  // sensor.setTimeout(500);
  // if (!sensor.init())
  // {
  //   Serial.println("Failed to detect and initialize sensor!");
  //   while (1) {}
  // }

  // sensor.startContinuous();

}

void loop() {
  readSerialPort();
  Serial.flush();
    if(recieved_status==1){
      switch (commands[0]){
        case 1:
          // Serial.println("Got data");
          togglePumps(commands[1]);
          break;
          
      }
      recieved_status=0;
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
      Serial.print("1,");
      Serial.println(current_gate);
  }

  }
  if(millis()-last_gate_timestamp>500){
    current_gate=0;
  }

  // tofValue=sensor.readRangeContinuousMillimeters();
  // // Serial.print("2,");
  // // Serial.println(reading);
  // if(tofValue<170){
  //   digitalWrite(ledPin,HIGH);
  // }else{
  //   digitalWrite(ledPin,LOW);
  // }
  // if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  // Serial.println();

  delay(20);
}

void gate1Intr() {
  current_gate=1;
  last_gate_timestamp=millis();
}

void gate2Intr() {
  current_gate=2;
  last_gate_timestamp=millis();
}

void gate3Intr() {
  current_gate=3;
  last_gate_timestamp=millis();
}

void gate4Intr() {
  current_gate=4;
  last_gate_timestamp=millis();
}

void gate5Intr() {
  current_gate=5;
  last_gate_timestamp=millis();
}

void gate6Intr() {
  current_gate=6;
  last_gate_timestamp=millis();
}

void gate7Intr() {
  current_gate=7;
  last_gate_timestamp=millis();
}

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

void readSerialPort(){
    int i=0;
    int counter=0;
    int k=0;
    int command=0;
 
    
    if (Serial.available()) {
        for(int j=0; j<5; j++){
            commands[j]=-1;
        }
        delay(10);
        receivedCommand = "";
        while (Serial.available() > 0) {
            data[i] = (char)Serial.read();
            // mporei na einai megalo to mnm mexri to \n
            i++;   
            if (i>=30){
                break;
            } 
            delay(1);   
        }   
        recieved_status = 1;
        Serial.flush();
        delay(20);
        for(i=0; i<20; i++){
            if(data[i]=='/' || data[i]==','){
                switch(counter){
                    case 1:
                        command=data[i-counter]-'0';
                        commands[k]=command;
                        break;
                    case 2:
                        command=(data[i-counter]-'0')*10 + (data[i-counter+1]-'0');
                        commands[k]=command;
                        break;
                    case 3:
                        command=(data[i-counter]-'0')*100 + (data[i-counter+1]-'0')*10 + (data[i-counter+2]-'0');
                        commands[k]=command;
                        break;
                    case 4:
                        command=(data[i-counter]-'0')*1000 + (data[i-counter+1]-'0')*100 + (data[i-counter+2]-'0')*10 + (data[i-counter+3]-'0');
                        commands[k]=command;
                        break;    
                    case 5:
                        command=(data[i-counter]-'0')*10000+(data[i-counter+1]-'0')*1000 + (data[i-counter+2]-'0')*100 + (data[i-counter+3]-'0')*10 + (data[i-counter+4]-'0');
                        commands[k]=command;
                        break;  
                    }
                counter=0;
                command=0;
                k++;
            }
            else{
                counter++;      
            }
        }

    } 
}
