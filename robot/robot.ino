/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** DEV *********************/
#define SPEAK 1
#define MOVE 0
#define SENSE 1


/****************** BIBLIOTHÈQUES *********************/
// Project
#include "CANconfig.h"
#include "sensors.h"
#include "motors.h"

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

// Général
int counterForPrinting;
const int printingPeriodicity = 100; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * PERIODS_IN_MICROS
unsigned long current_time, old_time, initial_time;

/****************** DECLARATION DES FONCTIONS *********************/
// Capteurs
bool obstacleEnFace();
bool obstacleADroite(); 

// Déplacement
void deplacementAngulaire(float deg);
void deplacementLineaire(float dist);

void setRobotVelocity(float linearVelocity, float angularVelocity); // Set the linear and angular velocity of the robot //TODO : Vérifier qu'elle fonctionne

// Saisie
void saisir();

// Affichage
void printData(double elapsedTime);


void setup() {
  /*************** MONITEUR SERIE ***************/
  // Initialization of the serial link
  Serial.begin(115200);
  counterForPrinting = 0;
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
  if (MOVE) {
    // Pour le moment c'est pour tester
    // Activation des moteurs
    sendVelocityCommand(MOTOR_ID_LEFT, 5000);
    readMotorState(MOTOR_ID_LEFT);
    delayMicroseconds(1000);
    sendVelocityCommand(MOTOR_ID_RIGHT, -2500);
    readMotorState(MOTOR_ID_RIGHT);
    delayMicroseconds(1000);
    sendVelocityCommand(MOTOR_ID_ARM, 1250);
    readMotorState(MOTOR_ID_ARM);
    delayMicroseconds(1000);
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
  leftMotorVel = -1 / robotWheelRadius * linearVelocity - (robotWheelDistance / (2.0 * robotWheelRadius)) * angularVelocity;
  rightMotorVel = 1 / robotWheelRadius * linearVelocity - (robotWheelDistance / (2.0 * robotWheelRadius)) * angularVelocity;

  // Envoie les commandes de vitesse aux moteurs
  sendVelocityCommand(MOTOR_ID_LEFT, leftMotorVel * (100.0 / MY_PI)); // Convertit la vitesse en centième de degrés par seconde
  readMotorState(MOTOR_ID_LEFT);
  delayMicroseconds(1000);
  sendVelocityCommand(MOTOR_ID_RIGHT, rightMotorVel * (100.0 / MY_PI)); // Convertit la vitesse en centième de degrés par seconde
  readMotorState(MOTOR_ID_RIGHT);
  delayMicroseconds(1000);

}

void printData(double elapsedTime) {
  /*************** MOTEURS ***************/
  if (!MOVE) {      
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
  }

  /*************** CAPTEURS ***************/
  if (SENSE) {
    int Distance = 0;
    for (int i = 0; i < NB_OF_SENSORS; i++) {
      Serial.println("--- Sensor " + String(i) + " ---");
      Distance = measuredLenght[i];
      if (Distance <= MesureMaxi && Distance >= MesureMini) {
        // Affichage dans le moniteur serie de la distance mesuree //
        Serial.println("Distance : " + String(i) + ": " + String(Distance) + "cm");
      } else {
        // Si la distance est hors plage, on affiche un message d'erreur
        Serial.println("Hors plage");
      }
    }
  }
}