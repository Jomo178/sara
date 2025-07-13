#include <Servo.h>
#include <NewPing.h>


// max distance for sonic sensors
#define MAX_DISTANCE 300 

// number of sonar sensors (currently 3, UPDATE TO 5)
#define NUM_SONAR 3

// TriggerPins uS
#define trig_left 12
#define trig_right 8
#define trig_front 10
#define trig_middle_left 6
#define trig_middle_right 4

// EchoPins uS
#define echo_left 13
#define echo_right 9
#define echo_front 11
#define echo_middle_left 7
#define echo_middle_right 5

// JOY-Stick Y-Axis
#define Y_pin A0
#define X_pin A1

// Servo Object
Servo servo1;
Servo ESC; 

// initialize array (distance in cm)
int distanceArray [NUM_SONAR]; // {left, right, front, middle-left, middle-right}

NewPing sonar[NUM_SONAR] = {
  NewPing(trig_left, echo_left, MAX_DISTANCE),
  NewPing(trig_right, echo_right, MAX_DISTANCE),
  NewPing(trig_front, echo_front, MAX_DISTANCE)
};

// Global data array for communication
int data[] = {90, 135};



void setup() {
  setupSonicSensors();

  servo1.attach(3);

  Serial.begin(9600);

  //Motor Setup
  ESC.attach(4,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
 
}

void loop() {
  int* dataFinal = readSerial();
  lenken(dataFinal);
  fahren(dataFinal);

  //updateSonar(sonar);
  //EntfernungenVergleich(distanceArray);

  delay(10);
}

void setupSonicSensors() {
  // Initialisiere die Pins für die Sender und Empfänger
  //Sensor Left
  pinMode(trig_left, OUTPUT);
  pinMode(echo_left, INPUT);

  //Sensor Right
  pinMode(trig_right, OUTPUT);
  pinMode(echo_right, INPUT);

  //Sensor Front
  pinMode(trig_front, OUTPUT);
  pinMode(echo_front, INPUT);

  //Sensor Middle Left
  pinMode(trig_middle_left, OUTPUT);
  pinMode(echo_middle_left, INPUT);

  //Sensor Middle Right
  pinMode(trig_middle_right, OUTPUT);
  pinMode(echo_middle_right, INPUT);
}

void lenken(int* dataFinal){

  servo1.write(dataFinal[1]);
}

// Fahren Funktion
void fahren(int* dataFinal) {
  //JoyValX = analogRead(X_pin);
  //JoyValX = map(data[0], 0, 1023, 0, 180); 
  ESC.write(dataFinal[0]);

}

// updates DistanceArray (declared at the top)
void updateSonar(NewPing sonicSensor[]) {
  for (int i = 0; i < NUM_SONAR; i++) {
    distanceArray[i] = sonar[i].ping_cm(); 

    // if distance = 0cm, its not in max range
    if (distanceArray[i] == 0)
      distanceArray[i] = MAX_DISTANCE;
  }
}

int EntfernungenVergleich(int distanceArray[]){
  int lowestDistance;
  int lowestIndex = 0;
  lowestDistance = distanceArray[0];

  for(int i = 0; i < NUM_SONAR; i++) {
    if(distanceArray[i] < lowestDistance) {
      lowestDistance = distanceArray[i];
      lowestIndex = i;
    }
  }

  if(lowestDistance < 50) {
    switch(lowestIndex) {
    case 0:
      Serial.println("LEFT IS CLOSEST, DODGE RIGHT");
      break;
    case 1:
      Serial.println("RIGHT IS CLOSEST, DODGE LEFT");
      break;
    case 2:
      Serial.println("FRONT IS CLOSEST, STOP AND REVERSE");
      break;
    case 3:
      Serial.println("MIDDLE-LEFT IS CLOSEST, DODGE RIGHT");
      break;
    case 4:
      Serial.println("MIDDLE-RIGHT IS CLOSEST, DODGE LEFT");
      break;
    }
  } else {
    Serial.println("Nothing closer than 50 cm");
  }//

  return lowestIndex;
}

int* readSerial() {
  if (Serial.available() > 0) {
    data[0] = Serial.read();
    delay(1);
    data[1] = Serial.read() + 45;
    delay(1);

  }
  return data;

}