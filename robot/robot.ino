/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** DEV *********************/
#define SPEAK 1
#define MOVE 1
#define SENSE 1
#define GRIP 1

/****************** BIBLIOTHÈQUES *********************/
// Project
#include "CANconfig.h"
#include "sensors.h"
#include "motors.h"

// Gripper servo
#include <Servo.h>

// For devices communication using the SPI bus
#include <SPI.h>

// For math functions
#include <math.h>

/****************** CONSTANTES *********************/
// Math (pourra servir)
#define MY_PI 3.14159265359

// Loop properties
#define PERIOD_IN_MICROS 5000 // 5 ms

/****************** DECLARATION DES VARIABLES GLOBALES *********************/
// Robot
const float robotWheelRadius = 0.0225; // Radius of the wheels in meters
const float robotWheelDistance = 0.15; // Distance between the wheels in meters
int counterForMoving;
int step;  

// Gripper 
Servo gripServo;
int counterForGrab_checking;
const int Grab_checkingPeriodicity = 200;
bool totem_grabbed;

// Général
int counterForPrinting;
const int printingPeriodicity = 25; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * PERIODS_IN_MICROS
unsigned long current_time, old_time, initial_time;

// Temporairement placées ici
int robotWallOffsetSetpoint = 10; // cm
int robotWallOffsetMeasure = 0;
const float correctorGain = 0.1;
int robotWallOffsetError = 0;
float robotAngularVelocityCommand = 0.0;
float robotWallOffsetErrorIntegrated = 0.0;
const float correctorIntegralGain = 0.01; 

/****************** DECLARATION DES FONCTIONS *********************/
// Déplacement
void setRobotVelocity(float linearVelocity, float angularVelocity); // Set the linear and angular velocity of the robot //TODO : Vérifier qu'elle fonctionne
void balayer();

// Saisie
void saisir();

// Affichage
void printData(double elapsedTime);

void setup() {
  /*************** MONITEUR SERIE ***************/
  // Initialization of the serial link
  Serial.begin(115200);
  counterForPrinting = 0;
  counterForGrab_checking = 0;
  /*************** BUS CAN ***************/ 
  // Démarrage de la communication avec le bus CAN
  while (CAN.begin(CAN_500KBPS) != CAN_OK) { // Tant que le bus CAN ne reçoit rien 
    Serial.println("CAN initialization failed, trying again...");
    delay(500);
  } // On sort de la boucle si au moins un moteur à envoyé des données sur le bus CAN
  delay(2000); // On attend suffisament de temps pour que le PC se connecte au moniteur série (évite les caractères corrompus)
  Serial.println("CAN initialization succeeded !");

  /*************** MOTEURS ***************/ 
  if (MOVE) {
    Serial.println("Starting initialization routine...");
    motorsInitialization(); // Initialisation des moteurs
    Serial.println("Initialization routine suceeded !");
    counterForMoving = 0; // Reset the counter for moving
    step = 0;
  }

  /*************** CAPTEURS ***************/
  if (SENSE) {
    // Broche trigger en sortie 
    pinMode(SENSOR_TRIGGER_PIN_RIGHT, OUTPUT); 
    pinMode(SENSOR_TRIGGER_PIN_FRONT, OUTPUT);
     
    // Broche echo en entree 
    pinMode(SENSOR_ECHO_PIN_RIGHT, INPUT); 
    pinMode(SENSOR_ECHO_PIN_FRONT, INPUT); 
  }

  if (GRIP) {
    // Création servo de la pince
    gripServo.attach(SERVO_TRIGGER_PIN);  
    pinMode(SERVO_FEEDBACK_PIN, INPUT);
    totem_grabbed = false;
    gripServo.write(0);
  }

  /*************** MESURES ***************/ 
  current_time = micros(); 
  initial_time = current_time;

  // Attendre que l'utilisateur envoie 'S' (À remplacer par un signal du bouton poussoir)
  Serial.println("Everything is ready, send 'S' to start the main loop.");
  while (Serial.read() != 'S') ;
}

void loop() {
  /*
  AJOUTER COGNITION
  */

  // Gestion de la boucle
  unsigned int sleep_time;
  // Clock
  double elapsed_time_in_s;
  old_time = current_time;
  current_time = micros();
  elapsed_time_in_s = (double)(current_time - initial_time);
  elapsed_time_in_s *= 0.000001;
 
  /*************** MOTEURS ***************/
  if (SENSE && MOVE) { 
    // ------------- Correcteur -------------- // 
    // robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
    // robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
    // if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
    // if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
    // robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError; + correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
    // if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
    // if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
    // setRobotVelocity(0.1, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
    if (step == 0) {
      if (counterForMoving < 100) { // On avance pendant 1 seconde (200 * 5 ms)
        setRobotVelocity(0.01, MY_PI/8.0); // Avance à 1 cm/s en tournant de 22.5° par seconde
        counterForMoving++;
      } else {
        counterForMoving = 0; // Reset the counter for moving
        step = 1;
      }
    } else if (step == 1) {
      if (counterForMoving < 200) {
        setRobotVelocity(0.01, -MY_PI/8.0); // Avance à 1 cm/s en tournant de 22.5° par seconde
        counterForMoving++;
      } else {
        counterForMoving = 0;
        // if (measuredLenght[0] <= 8) step = -1;
        // else step = 2;
        step = 2; 
      }
    } else if (step == 2) {
      if (counterForMoving < 200) {
        setRobotVelocity(0.01, MY_PI/8.0); // Avance à 1 cm/s en tournant de 22.5° par seconde
        counterForMoving++;
      } else {
        counterForMoving = 0;
        // if (measuredLenght[0] <= 8) step = -1;
        // else step = 1;
        step = 1;
      }
    } else {
      setRobotVelocity(0.0, 0.0);
    }
  }

  /*************** CAPTEURS ***************/ 
  if (SENSE) {
    // TODO : Fonctionnaliser dans sensors.cpp et ne faire que deux appels ppur màj le tableau
    int Duree;
    for (int i = 0; i < NB_OF_SENSORS; i++) {
      // Emission d'un signal ultrasonnore de 10 µS par TRIG 
      digitalWrite(triggerPins[i], LOW); // On efface l'etat logique de TRIG 
      delayMicroseconds(2);
      digitalWrite(triggerPins[i], HIGH); // On met la broche TRIG a "1" pendant 10µS 
      delayMicroseconds(10);
      digitalWrite(triggerPins[i], LOW); // On remet la broche TRIG a "0" 
      
      // Reception du signal refléchit sur l'objet
      Duree = pulseIn(echoPins[i], HIGH, 10000); // On mesure combien de temps le niveau logique haut est resté actif sur ECHO (TRIGGER envoie et ECHO réçois le rebond), timeout de 100000µs
      measuredLenght[i] = Duree * 0.034 / 2; // Calcul de la distance grace au temps mesure (à partir de la vitesse du son)
      delayMicroseconds(1000);
    }
    robotWallOffsetMeasure = measuredLenght[1]; // On récupère la mesure du capteur latéral pour l'asservissement de distance
  }
  /*************** GESTION PINCE ***************/
  if (SENSE && GRIP) { 
    if (measuredLenght[0] >= 8) {
      gripServo.write(0);
    } 
    else {
      gripServo.write(180); //REVERIFIER CES COMMANDES
    }

    int feedbackValue = analogRead(SERVO_FEEDBACK_PIN);
    float angle = map(feedbackValue, 94, 440, 0, 180); // conversion en angle 0 < > 180
    // Serial.print("Retour pince (angle estimé) : ");
    // Serial.println(angle);
    if(angle < 176 && angle > 40) {
      counterForGrab_checking++; 
    // SI IL DETECTE UN ANGLE ENTRE 175 et 120 
    // pendant 1 seconde alors il a choppé le totem !
      if (counterForGrab_checking > Grab_checkingPeriodicity) {
        counterForGrab_checking = 0;
        totem_grabbed = !totem_grabbed;
        Serial.println("Totem attrapé !");
        //INSTRUCTIONS SUITE
      }
    }
    else counterForGrab_checking = 0;
  }
  /*************** AFFICHAGE ***************/
  if (SPEAK) {
    counterForPrinting++;
    if (counterForPrinting > printingPeriodicity) {  // Reset the counter and print
      printData(elapsed_time_in_s);
      counterForPrinting = 0; // Reset counter
    }
  }
  
  // Patienter pour respecter le cadencement de la boucle
  sleep_time = PERIOD_IN_MICROS - (micros() - current_time);
  if ( (sleep_time > 0) && (sleep_time < PERIOD_IN_MICROS) ) delayMicroseconds(sleep_time); // On patiente le temps restant pour respecter la fréquence d'itération (SUPPOSE QUE LES INSTRUCTIONS SONT RÉALISABLES DURANT LA PERIODE)
} // FIN DE LA BOUCLE PRINCIPALE

// TODO : À bouger dans un fichier à part
void setRobotVelocity(float linearVelocity, float angularVelocity) {
  /*
  Calcul la cinématique inverse pour définir la vitesse des moteurs gauche et droit du robot en fonction de la vitesse linéaire et angulaire souhaitée.

  Précondition : La vitesse linéaire doit être exprimée en m/s et la vitesse angulaire en rad/s.
  */

  long int leftMotorVel, rightMotorVel; // Dans nos équations, q1 = vitesse du moteur droit, q2 = vitesse du moteur gauche 
  leftMotorVel = (long int) ( ((1.0 / robotWheelRadius) * linearVelocity + (robotWheelDistance / ( 2.0 * robotWheelRadius)) * angularVelocity) * (100.0 * 180.0 / MY_PI)); // Convertit la vitesse en centième de degrés par seconde
  rightMotorVel = (long int) ( ((-1.0 / robotWheelRadius) * linearVelocity + (robotWheelDistance / ( 2.0 * robotWheelRadius)) * angularVelocity) * (100.0 * 180.0 / MY_PI)); // Convertit la vitesse en centième de degrés par seconde

  // Envoie les commandes de vitesse aux moteurs
  sendVelocityCommand(MOTOR_ID_LEFT, leftMotorVel); 
  readMotorState(MOTOR_ID_LEFT);
  delayMicroseconds(1000);
  sendVelocityCommand(MOTOR_ID_RIGHT, rightMotorVel); 
  readMotorState(MOTOR_ID_RIGHT);
  delayMicroseconds(1000);

}

void balayer() {
  /*
  Balaye du regard en avançant pour trouver le totem.
  */
  setRobotVelocity(0.01, MY_PI/8.0); // Avance à 1 cm/s en tournant de 22.5° par seconde
  delay(2000); // Balaye pendant 1 seconde
  setRobotVelocity(0.01, -MY_PI/8.0); // Avance à 1 cm/s en tournant de 22.5° par seconde
  delay(2000); // Balaye pendant 1 seconde
}

void printData(double elapsedTime) {
  /*************** MOTEURS ***************/
  if (MOVE) {      
    for (int i = 0; i < NB_OF_MOTORS; i++) {            
      Serial.println("--- Motor " + String(i+1) + " ---");
      Serial.print("t:");
      Serial.println(elapsedTime);
      Serial.print("currentNumOfMotorRevol[" + String(i) + "]:");
      Serial.println(currentNumOfMotorRevol[i]);
      Serial.print("currentMotorPosDeg[" + String(i) + "]:");
      Serial.println(currentMotorPosDeg[i]);
      Serial.print("currentMotorVel[" + String(i) + "]:");
      Serial.println(currentMotorVel[i]);
      Serial.print("relativeMotorPosEncoder[" + String(i) + "]:");
      Serial.println(relativeMotorPosEncoder[i]);
      Serial.print("offsetMotorPosEncoder[" + String(i) + "]:");
      Serial.println(offsetMotorPosEncoder[i]);
    }
    Serial.println("Step : " + String(step));
    Serial.println("counterForMoving : " + String(counterForMoving));
  }

  /*************** CAPTEURS ***************/
  if (SENSE) {
    int Distance = 0;
    for (int i = 0; i < NB_OF_SENSORS; i++) {
      Serial.println("--- Sensor " + String(i) + " ---");
      Distance = measuredLenght[i];
      if (Distance <= MesureMaxi && Distance >= MesureMini) {
        Serial.println("Distance : " + String(Distance) + "cm");
        
      } else {
        // Si la distance est hors plage, on affiche un message d'erreur
        Serial.println("Hors plage");
      }
    }
  }

  /*************** PINCE ***************/
  if (GRIP) {
    Serial.println("--- Gripper ---");
    Serial.print("Totem grabbed: ");
    Serial.println(totem_grabbed ? "Yes" : "No");
    Serial.print("Servo angle: ");
    Serial.println(gripServo.read());
    Serial.println("Servo mesure: "+String(analogRead(SERVO_FEEDBACK_PIN)));
  }

  /*************** Asservissement ***************/
  if (MOVE) {  
      Serial.println("--- Asservissement ---");    
      Serial.println ("Erreur : " + String(robotWallOffsetError));   
      Serial.println("Intégrale de l'erreur : " + String(robotWallOffsetErrorIntegrated));       
      Serial.println("Commande : " + String(robotAngularVelocityCommand));
  }
}