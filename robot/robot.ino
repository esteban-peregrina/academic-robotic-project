/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** DEV *********************/
#define SPEAK true
#define MOVE 1
#define SENSE 1
#define GRIP 0

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
int consigne; // Consigne du bras (degrès)
int checkSensor;

// Gripper 
Servo gripServo;
int counterForGrab_checking;
const int Grab_checkingPeriodicity = 200;
bool totem_grabbed;

// poussoir
const int boutonPin = 8;
bool robotActif = false;

// Général
int counterForPrinting;
const int printingPeriodicity = 25; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * PERIODS_IN_MICROS
unsigned long current_time, old_time, initial_time;

// Temporairement placées ici
int robotWallOffsetSetpoint = 10; // cm
int robotWallOffsetMeasure = 0;
const float correctorGain = 0.08;
int robotWallOffsetError = 0;
float robotAngularVelocityCommand = 0.0;
float robotWallOffsetErrorIntegrated = 0.0;
const float correctorIntegralGain = 0.01; 

/****************** DECLARATION DES FONCTIONS *********************/
// Déplacement
void setRobotVelocity(float linearVelocity, float angularVelocity); // Set the linear and angular velocity of the robot //TODO : Vérifier qu'elle fonctionne
void setArmPosition(float erreur);
void balayer();
bool isTotemviewed();
// Saisie
void saisir();

// Affichage
void printData(double elapsedTime);

void setup() {
  /*************** MONITEUR SERIE ***************/
  pinMode(boutonPin, INPUT_PULLUP);
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
    checkSensor = 0; // Reset the sensor check variable
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


  consigne = 70;
  /*************** MESURES ***************/ 
  current_time = micros(); 
  initial_time = current_time;
  capteur();
  delay(100); 
  capteur();
  robotWallOffsetSetpoint = currentMeasuredLenght[1];
}

void loop() {


  if (digitalRead(boutonPin) == LOW) {
    delay(100); // anti-rebond simple
    if (!robotActif){
    robotActif = true;
    step = 3;
    }else{
    robotActif = false;
    }
    delay(100);
  }

  unsigned int sleep_time;
  double elapsed_time_in_s;
  old_time = current_time;
  current_time = micros();
  elapsed_time_in_s = (double)(current_time - initial_time);
  elapsed_time_in_s *= 0.000001;



  if (robotActif) {
      if (step == 0) {
        capteur();
        delay(100);
        robotWallOffsetSetpoint = currentMeasuredLenght[1];
        delay(100);
        step = 1;
      }
      if (step == 1) {
      if (isTotemviewed()) { // Si le totem est vu, on passe à l'étape suivante
      // faire 90°
      setRobotVelocity(0,0);
      delay(500);
      setRobotVelocity(0.05,0);
      delay(3600); 
      setRobotVelocity(0, -MY_PI/8.0); // On tourne de 90° vers la droite
      delay(3150); // On attend 1 seconde pour que le robot tourne
      capteur(); // On lit les capteurs pour mettre à jour la mesure du mur
      delay(100);
      robotWallOffsetSetpoint = currentMeasuredLenght[1]; // On met à jour la position du mur
        step = 2;
      } else {
      robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
      robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
      if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
      if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
      robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError;// + correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
      if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
      if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
      setRobotVelocity(0.025, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
      }
    }
    else if (step == 2) {

      setRobotVelocity(0, 0);
    }
    else if (step == 3) {
      consigne = 100; // mettre la bonne valeur pour qu'il rase le sol (seule zone où il pourra voir à coup sur le totem en balayant)
      int nbfoisapercuconfirmed = 0;
      int nbfoisvu=0;
      int delayv = 100;
      bool trouvedx= false;
      int multiplicator = -1 ;
      setRobotVelocity(0.01, +MY_PI/8.0);
      delay(1200); // 0 -> 40°
      while(1){
        for(int b=0;b<24;b++){
          setRobotVelocity(0.01, multiplicator*MY_PI/8.0);
          delay(delayv); //echantillonnage 50ms
          int diff=previousMeasuredLenght[0]-currentMeasuredLenght[0];
          //peut etre calculer la diff de dérivée plutot
          if(diff>20){ // a modifier selon la vitesse à laquelle on avance
            nbfoisvu++;
            Serial.println("Detecté ?");
            if(nbfoisvu>3){
              // reculer de 2 ?
              Serial.println("Trouvé !");
              trouvedx=true;
              break;
            }
          }
          else{
            if(nbfoisvu!=0){nbfoisvu=0;Serial.println("Fausse alerte");}}
          //si mesure > lastmesure+10 -> peut etre trouvé ?
        }
        multiplicator=multiplicator*(-1); // change le sens de rotation
        if(trouvedx!=true){
          if(currentMeasuredLenght[0]>240){
            Serial.println("XXXXXXX Pas trouvé en largeur");
            //manip de marche arrière et demi tour
          }
          delayv=delayv+5;continue;
        } //retour au scan plus élargi
        
        // CHECKPOINT - Totem Vu

        while(1){
          setRobotVelocity(0.03, 0);
          delay(10);
          if(currentMeasuredLenght[0]<30){
            Serial.println("Trouvé et approché!");
            break;
          }
        }
      } 
      //quand la distance est < 20cm
      bool trouvedy= false;
      while(trouvedy==false){
        consigne= consigne+0.2;
        if(currentMeasuredLenght[0]-previousMeasuredLenght[0]>20){
          trouvedy==true;
          Serial.println("Trouvé en hauteur !");
        }
        delayMicroseconds(50);
        if(consigne>170){Serial.println("XXXXXXX Pas trouvé en hauteur");}
      }
      setRobotVelocity(0, 0); // On arrête le robot
    }
  } else {
    // Si le robot n'est pas actif, on arrête les moteurs
    setRobotVelocity(0, 0);
  }

  /*************** ASSERVISSEMENT BRAS ***************/
  float erreur = 150*(consigne - currentMotorPosDeg[2]);
  setArmPosition(erreur);

  capteur();
  /*************** GESTION PINCE ***************/
  if (SENSE && GRIP) { 
    if (currentMeasuredLenght[0] >= 8) {
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
    // SI IL DETECTE UN ANGLE ENTRE 175 et 120 _
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
  delayMicroseconds(10);
  sendVelocityCommand(MOTOR_ID_RIGHT, rightMotorVel); 
  readMotorState(MOTOR_ID_RIGHT);
  delayMicroseconds(10);

}

void setArmPosition(float erreur){
  // 3 - 105.2 111 118.15 129.30 133.7 - 7 HAUTEUR PINCE
  // 9 - 107.42 116.38 122.5 134.5 - 12 CAPTEUR
  // consigne = map(feedbackValue, 107.42,134.5, 105.2, 133.7);
  sendVelocityCommand(MOTOR_ID_ARM, erreur); 
  readMotorState(MOTOR_ID_ARM);
  delayMicroseconds(10);
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
      Distance = currentMeasuredLenght[i];
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

bool isTotemviewed() {

  if (abs(currentMeasuredLenght[1] - previousMeasuredLenght[1]) > 2) { // Si la distance mesurée par le capteur frontal augmente de plus de 5 cm, on considère que le totem est vu
    return true;
  }
  return false;
}

void capteur(){
  int Duree;
    for (int i = 0; i < NB_OF_SENSORS; i++) {
      // Emission d'un signal ultrasonnore de 10 µS par TRIG 
      digitalWrite(triggerPins[i], LOW); // On efface l'etat logique de TRIG 
      delayMicroseconds(2);
      digitalWrite(triggerPins[i], HIGH); // On met la broche TRIG a "1" pendant 10µS 
      delayMicroseconds(10);
      digitalWrite(triggerPins[i], LOW); // On remet la broche TRIG a "0" 
      
      // Reception du signal refléchit sur l'objet
      Duree = pulseIn(echoPins[i], HIGH, 20000); // On mesure combien de temps le niveau logique haut est resté actif sur ECHO (TRIGGER envoie et ECHO réçois le rebond), timeout de 100000µs
      previousMeasuredLenght[i] = currentMeasuredLenght[i]; // On sauvegarde la mesure précédente
      currentMeasuredLenght[i] = Duree * 0.034 / 2; // Calcul de la distance grace au temps mesure (à partir de la vitesse du son)
      delayMicroseconds(1000);
    }
    robotWallOffsetMeasure = currentMeasuredLenght[1]; // On récupère la mesure du capteur latéral pour l'asservissement de distance
}