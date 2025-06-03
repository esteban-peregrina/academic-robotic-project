/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** DEV *********************/
#define SPEAK 1
#define MOVE 1
#define SENSE 1
#define PINCE 1

/****************** BIBLIOTHÈQUES *********************/
// Project
#include "CANconfig.h"
#include "sensors.h"
#include "motors.h"

#include <Servo.h>
Servo myservo;
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

// Temporairement placées ici
int robotWallOffsetSetpoint = 10; // cm
int robotWallOffsetMeasure = 0;
const float correctorGain = 0.1;
int robotWallOffsetError = 0;
float robotAngularVelocityCommand = 0.0;
float robotWallOffsetErrorIntegrated = 0.0;
const float correctorIntegralGain = 0.01; 

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

  if(PINCE){
    // Création servo de la pince
    myservo.attach(SERVO_TRIGGER_PIN);  
    pinMode(SERVO_FEEDBACK_PIN, INPUT);
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
    robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
    robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
    if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
    if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
    robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError; + correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
    if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
    if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
    setRobotVelocity(0.1, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
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
  
  if (PINCE) { 
    int feedbackValue = analogRead(SERVO_FEEDBACK_PIN);
    float angle = map(feedbackValue, 94, 383, 0, 180);
    Serial.print("Retour pince (angle estimé) : ");
    Serial.println(angle);
    // SI IL DETECTE UN ANGLE ENTRE 175 et 120 
    // pendant 1 seconde il a choppé le totem !
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
        if(i==0){ //si c'est le capteur 0
          if(Distance >= MesureMaxi/2){
            myservo.write(60);
            Serial.println("Commande = 60 servo");
          }
          else{
            myservo.write(150);
            Serial.println("Commande = 150 servo");
          }
        }
      } else {
        // Si la distance est hors plage, on affiche un message d'erreur
        if(i==0){
          Serial.println("Hors plage");
          myservo.write(0);
        }
      }
    }
  }

   /*************** Asservissement ***************/
  if (MOVE) {  
      Serial.println("--- Asservissement ---");    
      Serial.println ("Erreur : " + String(robotWallOffsetError));   
      Serial.println("Intégrale de l'erreur : " + String(robotWallOffsetErrorIntegrated));       
      Serial.println("Commande : " + String(robotAngularVelocityCommand));
  }

   /*************** Asservissement ***************/
  if (MOVE) {  
      Serial.println("--- Asservissement ---");    
      Serial.println ("Erreur : " + String(robotWallOffsetError));   
      Serial.println("Intégrale de l'erreur : " + String(robotWallOffsetErrorIntegrated));       
      Serial.println("Commande : " + String(robotAngularVelocityCommand));
  }
}