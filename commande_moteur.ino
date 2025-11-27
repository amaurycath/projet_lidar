#include <AccelStepper.h>

const int stepPin = 7;
const int dirPin  = 8;
int nbPas = 8; //nb de 1/32 step pour faire 1/2 step

AccelStepper motor(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  Serial.begin(9600);        // Initialisation du port série
  motor.setMaxSpeed(200);
  motor.setAcceleration(200);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');  // lire une ligne

    if (cmd == "start") {
      motor.moveTo(motor.currentPosition() + nbPas );  
    } 
    else if (cmd == "stop") {
      motor.stop();
    }
  }

  motor.run();  // AccelStepper doit être appelé tout le temps
}
