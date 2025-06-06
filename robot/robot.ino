/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** DEV *********************/
#define SPEAK 0
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
int counterForMoving = 0;  
int checkSensor;
enum RobotState {
  SETPOINT_INIT,
  GO_TO_BEACON,
  ALIGN_WITH_BEACON,
  TURN_TO_TOTEM,
  ALIGN_WITH_WALL,
  APPROACH_AFTER,
  SWEEP,
  APPROACH,
  DETECT,
  GRAB,
  TURN_AFTER_GRAB,
  GO_STRAIGHT,
  TURN_AFTER_STRAIGHT,
  FINAL_STRAIGHT,
  DROP,
  END
};
RobotState step;
int counterForBeacons = 0;
double savedMeasuredLenght; 

// Gripper 
Servo gripServo;
int grabStabilityCounter;
const int Grab_checkingPeriodicity = 200;
bool totem_grabbed;

// poussoir
const int boutonPin = 8;
bool robotActif = false;

// Général
int tempbeacon = 0;
int counterForPrinting;
const int printingPeriodicity = 25; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * PERIODS_IN_MICROS
unsigned long current_time, old_time, initial_time;
double consigne = 180; // Consigne de position du bras
float erreur; // Erreur de position du bras
float angledrop;
// Temporairement placées ici
int robotWallOffsetSetpoint; // cm
int robotWallOffsetMeasure = 0;
const float correctorGain = 0.12;
int robotWallOffsetError = 0;
float robotAngularVelocityCommand = 0.0;
float robotWallOffsetErrorIntegrated = 0.0;
const float correctorIntegralGain = 0.01; 

/****************** DECLARATION DES FONCTIONS *********************/
// Déplacement
void setRobotVelocity(float linearVelocity, float angularVelocity); // Set the linear and angular velocity of the robot //TODO : Vérifier qu'elle fonctionne
void setArmPosition(float erreur);

// Capteurs
void capteur();
bool beaconInRange();

// Saisie
void saisir();

// Affichage
void printData(double elapsedTime);


/****************** SETUP *********************/
void setup() {
  /*************** MONITEUR SERIE ***************/
  pinMode(boutonPin, INPUT_PULLUP);
  // Initialization of the serial link
  Serial.begin(115200);
  counterForPrinting = 0;
  grabStabilityCounter = 0;
  /*************** BUS CAN ***************/ 
  // Démarrage de la communication avec le bus CAN
  while (CAN.begin(CAN_500KBPS) != CAN_OK) { // Tant que le bus CAN ne reçoit rien 
    Serial.println("CAN initialization failed, trying again...");
    delay(500);
  } // On sort de la boucle si au moins un moteur à envoyé des données sur le bus CAN
  delay(1000); // On attend suffisament de temps pour que le PC se connecte au moniteur série (évite les caractères corrompus)
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
    gripServo.write(145);
  }

  /*************** MESURES ***************/ 
  current_time = micros(); 
  initial_time = current_time;

  robotWallOffsetSetpoint = 0;
}

void loop() {
   /*************** CADENCEMENT ***************/
   unsigned int sleep_time;
   double elapsed_time_in_s;
   old_time = current_time;
   current_time = micros();
   elapsed_time_in_s = (double)(current_time - initial_time);
   elapsed_time_in_s *= 0.000001;

  /*************** BOUTON ***************/
  if (digitalRead(boutonPin) == LOW) {
    delay(100); // Anti-rebond
    if (!robotActif) {
      robotActif = true;
      counterForMoving = 0; 
      counterForBeacons = 0;
      step = SETPOINT_INIT;
      counterForMoving = 0;
      savedMeasuredLenght = 0.0;
      Serial.println("-->SETPOINT_INIT");
    } else {
      robotActif = false;
    }
    delay(100); // Anti-rebond
  }

  /*************** PARCOURS DE L'ARÈNE ***************/
  if (robotActif) {
    // Acquisition des mesures des capteurs
    capteur(); 
    // Serial.print("currentMeasuredLenght[2]:");
    // Serial.println(currentMotorPosDeg[2]);
    // erreur = 150*(consigne - currentMotorPosDeg[2]);   MARCHE PAS
    // setArmPosition(erreur);


    // Saisie du totem
    // if (currentMeasuredLenght[0] < 8) {
    //   gripServo.write(180); 

    //   int feedbackValue = analogRead(SERVO_FEEDBACK_PIN);
    //   float angle = map(feedbackValue, 94, 440, 0, 180); // conversion en angle 0 < > 180
    //   if (angle < 176 && angle > 40) {
    //     grabStabilityCounter++; 
    //     if (grabStabilityCounter > 200) {
    //       totem_grabbed = !totem_grabbed;
    //       grabStabilityCounter = 0;
    //     }
    //   } else grabStabilityCounter = 0;
    // } else gripServo.write(0);
    
    switch (step) {
      case SETPOINT_INIT: // On fixe la consigne
        counterForMoving++;
        if (counterForMoving > 100) {
          robotWallOffsetSetpoint = currentMeasuredLenght[1];
          savedMeasuredLenght = robotWallOffsetSetpoint;
          step = SWEEP; // On commence à avancer
          Serial.println("-->GO_TO_BEACON");  
        }
        break; 

      //   case APPROACH:
      //   // On avance un peu pour se rapprocher du totem
      //   counterForMoving++;
      //   if (counterForMoving < 200) {
      //     setRobotVelocity(0.05, 0.2); // Avance à 5 cm/s
      //     consigne = 113; // On fixe la consigne de position du bras
      //   } else {
      //     Serial.println("PASSAGE A 0");
      //     setRobotVelocity(0, 0);
          
      //     delay(100); // On attend un peu pour stabiliser le robot
      //     consigne = 150; // On fixe la consigne de position du bras
      //   }
      // break; 

      case GO_TO_BEACON: // On avance jusqu'à la première balise
        //static int stabilityCounterForBeacon = 0;
        // Asservissement de la distance au mur

        if ((abs(previousMeasuredLenght[1] - currentMeasuredLenght[1]) > 3)) { // Si la distance au setpoint est trop importante
          Serial.println("Beacon ?");
          step = ALIGN_WITH_BEACON;
          //stabilityCounterForBeacon++;
        }

        robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
        robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
        if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
        if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
        robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError;//+ correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
        if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
        if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
        setRobotVelocity(0.05, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
      
          //stabilityCounterForBeacon = 0; // On réinitialise le compteur de stabilité

        
        // if (stabilityCounterForBeacon >= 2) {
        //   counterForBeacons++;
        //   setRobotVelocity(0, 0);
        //   counterForMoving = 0;
        //   //step = (counterForBeacons > 3) ? DROP : ALIGN_WITH_BEACON; // Si on a vu 3 balises, on pose le totem, sinon on continue la procédure
        //   (step == DROP) ? Serial.println("-->DROP") : Serial.println("-->ALIGN_WITH_BEACON"); 
        // }

        break;
      case ALIGN_WITH_BEACON: // On s'aligne à peu près avec la balise
        counterForMoving++;
        if (counterForMoving < 260) setRobotVelocity(0.1, 0); 
        else {
          setRobotVelocity(0, 0);
          counterForMoving = 0;
          step = TURN_TO_TOTEM;
          Serial.println("-->TURN_TO_TOTEM");
        }
        break;

      case TURN_TO_TOTEM: // On tourne vers le totem
        counterForMoving++;
        if (counterForMoving < 207) setRobotVelocity(0, -MY_PI/8.0); 
        else {
          counterForMoving = 0;
          setRobotVelocity(0, 0);
          delay(100);
          setArmPosition(-2000);
          delay(3900);
          setArmPosition(0);
          step = APPROACH_AFTER;
          Serial.println("-->APPROACH_AFTER");
        }
        break;
        
      // case ALIGN_WITH_WALL:
      //   static int oscillationCounter = 0;
      //   static int currentDirection = 1; // 1 ou -1
      //   static float lastDistance = 0;
      //   static int stabilityCounter = 0;
      //   static int angularIncrementCounter = 0;
    
      //   float speed = currentDirection * 0.05;
      //   float angularSpeed = MY_PI / 6.0;  // Rotation sur place
    
      //   // Avancer un peu dans une direction
      //   angularIncrementCounter++;
      //   if (angularIncrementCounter > 100 && angularIncrementCounter < 125) { // Incrément angulaire (25 tours de boucle)
      //     setRobotVelocity(speed, angularSpeed);
      //   } else if (angularIncrementCounter > 200) { 
      //     angularIncrementCounter = 0;
      //   } else {
      //     // Aller tout droit
      //     setRobotVelocity(speed, 0);
      //   }
    
      //   // Mesure la variation du capteur latéral (ex: capteur à droite)
      //   float delta = currentMeasuredLenght[1] - lastDistance;
      //   lastDistance = currentMeasuredLenght[1];
    
      //   // Si très peu de variation -> on commence à être parallèle
      //   if (fabs(delta) < 0.1) {
      //       stabilityCounter++;
      //       angularSpeed = 0;
      //   } else {
      //       if (delta > 0) {
      //         angularSpeed = MY_PI / 6.0;
      //       } else {
      //         angularSpeed = -MY_PI / 6.0;
      //       }
      //       stabilityCounter = 0;
      //   }
    
      //   // Si plusieurs itérations sont stables, on considère qu’on est parallèle
      //   if (stabilityCounter >= 10) {
      //       setRobotVelocity(0, 0);
      //       robotWallOffsetSetpoint = currentMeasuredLenght[1];
      //       stabilityCounter = 0;
      //       oscillationCounter = 0;
      //       step = (totem_grabbed) ? GO_TO_BEACON : APPROACH_AFTER; // Si on a déjà le totem, on le prend, sinon on balaye
      //       if (step == APPROACH_AFTER) {
      //         counterForMoving = 0; // Reset the counter for moving
      //       }
      //       (step == GO_TO_BEACON) ? Serial.println("-->GO_TO_BEACON") : Serial.println("-->APPROACH_AFTER");  
      //   }
    
      //   // On change de direction toutes les 50 itérations
      //   if (++oscillationCounter > 100) {
      //       currentDirection *= -1;
      //       oscillationCounter = 0;
      //   }
      //   break;
      
      case APPROACH_AFTER:
        // On avance un peu pour se rapprocher du totem
        counterForMoving++;
        if (counterForMoving < 650) {
          setRobotVelocity(0.05, 0); // Avance à 5 cm/s
        } else {
          setRobotVelocity(0, 0);
          delay(100); // On attend un peu pour stabiliser le robot

          counterForMoving = 0;
          step = SWEEP; // On balaye pour trouver le totem
          Serial.println("-->SWEEP");
        }
        break;

        case SWEEP:
      // On balaye du regard pour trouver le totem
        counterForMoving++;
        if (counterForMoving < 80) {
          setRobotVelocity(0, MY_PI/16.0); // On tourne à 11.25°/s
          consigne = 113; // On fixe la consigne de position du bras
        } else if ( counterForMoving < 300) {
          setRobotVelocity(0, -MY_PI/64);
          Serial.println("current");
          Serial.println(currentMeasuredLenght[0]);
          Serial.println("erreur");
          Serial.println(currentMeasuredLenght[0]-previousMeasuredLenght[0]);
          delay(100);
          if (abs(currentMeasuredLenght[0]-previousMeasuredLenght[0]) > 10) { // Si on voit le totem
            Serial.println("Totem vu !");
            delay(500*currentMeasuredLenght[0]/40); // On attend un peu pour stabiliser le robot
            setRobotVelocity(0, 0); // On arrête le robot
            counterForMoving = 0; // On réinitialise le compteur de mouvement
            step = APPROACH; // On passe à l'étape de prise du totem
            Serial.println("-->APPROACH");
          }
        }
        break;

      case APPROACH: // On avance pour se rapprocher du totem
        setRobotVelocity(0.02, 0);
        if (currentMeasuredLenght[0] < 7) { 
          setRobotVelocity(0, 0);
          counterForMoving = 0;
          step = DETECT; // On passe à l'étape de prise du totem
          gripServo.write(0);
          Serial.println("-->DETECT");
          setArmPosition(2000);
        }
        break; 

      case DETECT: // On saisit le totem
        counterForMoving++;
        delay(100); // On attend un peu pour stabiliser le robot
        if (abs(currentMeasuredLenght[0] - previousMeasuredLenght[0]) > 30) {
          setArmPosition(0); 
          counterForMoving = 0; // On réinitialise le compteur de mouvement
          delay(100);
          setArmPosition(-2000);
          delay(250);
          setArmPosition(0); // On remet le bras à la position initiale
          step = GRAB;
          Serial.println("-->GRAB");
        }
        if (counterForMoving < 100) {
          setRobotVelocity(0, 0);
          Serial.println("Totem pas detecté !");
        }
        break;
      
      case GRAB: // On saisit le totem
      {
        setRobotVelocity(0.02, 0);
        if (currentMeasuredLenght[0] < 5) { // Si on est assez proche du totem
          delay(1000); 
          setRobotVelocity(0, 0); // On arrête le robot
          delay(500); 
          gripServo.write(140); // On ferme la pince
          delay(100);
          setArmPosition(2000); // On lève le bras
          delay(1000); // On attend un peu pour stabiliser le robot
          setArmPosition(0); // On remet le bras à la position initiale
          totem_grabbed = true; // On a le totem
          counterForMoving = 0; // On réinitialise le compteur de mouvement
          step = TURN_AFTER_GRAB; // On passe à l'étape de rotation après la prise du totem
        }
      }
      // case SWEEP: // On balaye du regard
      //   int TOLDX = 10;
      //   int TOLSTABLE = 4;
      //   int COUNTFORSTABLE = 1;
      //   int nbfoisapercuconfirmed = 0;
      //   int nbfoisvu=0;
      //   int delayv = 50; //valeur d'échantillonnage de délai
      //   bool trouvedx= false;
      //   int loopamount = 24;
      //   consigne = 113; // mettre la bonne valeur pour qu'il rase le sol (seule zone où il pourra voir à coup sur le totem en balayant)
      //   int multiplicator = -1 ;
      //   float linspeed = 0.01;
      //   float angspeed = MY_PI/16.0;
        
      //   setArmPosition(0);
      //   setRobotVelocity(linspeed, angspeed);
      //   delay((delayv*loopamount)/2); // 0 -> 40°
      //   while(trouvedx==false){
      //     for(int b=0;b<loopamount;b++){
      //       float erreur = 300*(consigne - currentMotorPosDeg[2]);
      //       setArmPosition(erreur);
      //       setRobotVelocity(linspeed, multiplicator*angspeed);
      //       delay(delayv); //echantillonnage 50ms
      //       capteur();
      //       if(currentMeasuredLenght[0]>180){continue;}
      //       // Serial.print(String(currentMeasuredLenght[0])+"\t");
      //       if(currentMeasuredLenght[0]<180){
      //         Serial.print("d:");
      //         Serial.println(currentMeasuredLenght[0]);
      //       }

      //       int diff=previousMeasuredLenght[0]-currentMeasuredLenght[0];
      //       //peut etre calculer la diff de dérivée plutot
      //       if(abs(diff)>TOLDX){ // a modifier selon la vitesse à laquelle on avance
      //         Serial.println("Detecté ?");
      //         // CONTINUER SANS REGARDER LES 2 PROCHAINES VALEURS
      //         setArmPosition(erreur);
      //         setRobotVelocity(linspeed, multiplicator*angspeed);
      //         delay(delayv*2); // skip 2 valeurs
      //         //
      //         for(int c=0;c<=COUNTFORSTABLE;c++){
      //           float erreur = 300*(consigne - currentMotorPosDeg[2]);
      //           setArmPosition(erreur);
      //           setRobotVelocity(linspeed, multiplicator*angspeed);
      //           delay(delayv);
      //           capteur();
      //           Serial.print("d:");
      //           Serial.println(currentMeasuredLenght[0]);
      //           diff=previousMeasuredLenght[0]-currentMeasuredLenght[0];
      //           if(abs(diff)<TOLSTABLE){
      //             Serial.print("Stable");
      //             nbfoisvu++;}
      //           else{
      //             Serial.println("Fausse alerte");
      //             nbfoisvu=0;
      //             break;
      //           }
      //         }
      //         // Serial.println("nbfoisvu = "+nbfoisvu);
      //         if(nbfoisvu==COUNTFORSTABLE+1){
      //           setRobotVelocity(linspeed, (-1)*multiplicator*angspeed);
      //           delay(delayv*COUNTFORSTABLE/2.0);
      //           setRobotVelocity(0,0);
      //           Serial.println("Trouvé !");
      //           trouvedx=true;
      //           break;
      //         }
      //       }
      //     }
      //     multiplicator=multiplicator*(-1); // change le sens de rotation

      //     if(trouvedx==false){
      //       continue;  // on retourne au début
      //       }
      //     // CHECKPOINT - Totem Vu
      //     delay(2000);

      //     while(1){
      //       float erreur = 300*(consigne - currentMotorPosDeg[2]);
      //       setArmPosition(erreur);
      //       setRobotVelocity(0.05, 0);
      //       delay(10);
      //       capteur();
      //       Serial.print("d:");
      //       Serial.println(currentMeasuredLenght[0]);
      //       if(abs(currentMeasuredLenght[0]-previousMeasuredLenght[0])>6){
      //         Serial.println("totem perdu");
      //         trouvedx=false; // on rebalaye
      //         //sécurité
      //       }
      //       if(currentMeasuredLenght[0]<5){
      //         Serial.println("Trouvé et approché!");
      //         break;
      //       }
      //     }
      //   } 
      //   //quand la distance est < 5cm
      //   bool trouvedy= false;
      //   consigne=180;
      //   float erreur = 300*(consigne - currentMotorPosDeg[2]);
      //   setArmPosition(erreur);
      //   setRobotVelocity(0, 0);
      //   delay(30);
      //   while(trouvedy==false){
      //     consigne= consigne-0.02;
      //     capteur();
      //     float erreur = 300*(consigne - currentMotorPosDeg[2]);
      //     setArmPosition(erreur);
      //     Serial.println("DIFF Y:"+String(currentMeasuredLenght[0]-previousMeasuredLenght[0]));
      //     if(abs(currentMeasuredLenght[0]-previousMeasuredLenght[0])>4){
      //       trouvedy=true;
      //       Serial.println("Trouvé en hauteur !");
      //       angledrop = consigne; // On sauvegarde l'angle de drop
      //     }
      //     delay(10);
      //   }
      //   setRobotVelocity(0, 0); // On arrête le robot
      //   setRobotVelocity(10, 0);
      //   gripServo.write(0);
      //   delayMicroseconds(50);
      //   totem_grabbed = true; // On a le totem
      //   counterForMoving = 0; // On réinitialise le compteur de mouvement
      //   step = TURN_AFTER_GRAB;
      //   break;
      case TURN_AFTER_GRAB: // On tourne pour se diriger vers la zone de dépose 
        counterForMoving++;
        if (counterForMoving < 207) setRobotVelocity(0, -MY_PI/8.0); 
        else {
          counterForMoving = 0;
          setRobotVelocity(0, 0);
          step = GO_STRAIGHT;
          robotWallOffsetSetpoint = currentMeasuredLenght[1];
          delay(500); 
          Serial.println("-->GO_STRAIGHT");
        }
        break;

      case GO_STRAIGHT: // On se positionne à la bonne distance
        //stabilityCounterForBeacon = 0;
        // Asservissement de la distance au mur
        robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
        robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
        if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
        if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
        robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError;// + correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
        if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
        if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
        setRobotVelocity(0.05, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
        
        if ((abs(previousMeasuredLenght[1] - currentMeasuredLenght[1]) > 3)) { // Si la distance au setpoint est trop importante
          Serial.println("Beacon ?");
          step = TURN_AFTER_STRAIGHT;
        }
        break;
      
      case TURN_AFTER_STRAIGHT: // On tourne pour se diriger vers la zone de dépose
      
      counterForMoving++;
        if (counterForMoving < 207) setRobotVelocity(0, MY_PI/8.0); 
        else {
          consigne = 150;
          counterForMoving = 0;
          setRobotVelocity(0, 0);
          step = FINAL_STRAIGHT;
          robotWallOffsetSetpoint = currentMeasuredLenght[1]; // On fixe la consigne de distance au mur
          Serial.println("-->FINAL_STRAIGHT");
        }
        break;
      
      case FINAL_STRAIGHT: // On avance
        //stabilityCounterForBeacon = 0;
        // Asservissement de la distance au mur
        robotWallOffsetError = robotWallOffsetSetpoint - robotWallOffsetMeasure; // Erreur de position du robot par rapport au mur
        robotWallOffsetErrorIntegrated += (double)robotWallOffsetError * elapsed_time_in_s;
        if (robotWallOffsetErrorIntegrated > 100.0) robotWallOffsetErrorIntegrated = 100.0; // Saturation de l'erreur intégrée
        if (robotWallOffsetErrorIntegrated < -100.0) robotWallOffsetErrorIntegrated = -100.0; // Saturation de l'erreur intégrée
        robotAngularVelocityCommand = correctorGain * (float)robotWallOffsetError;// + correctorIntegralGain * (float)robotWallOffsetErrorIntegrated; // Commande de vitesse angulaire du robot, proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
        if (robotAngularVelocityCommand > 5.0) robotAngularVelocityCommand = 5.0; // Saturation de la vitesse angulaire
        if (robotAngularVelocityCommand < -5.0) robotAngularVelocityCommand = -5.0; // Saturation de la vitesse angulaire
        setRobotVelocity(0.05, -1 * robotAngularVelocityCommand); // On assigne une vitesse linéaire de 20 cm/s et une vitesse angulaire proportionnelle à l'erreur de position du robot par rapport au mur (0.01 rad/cm)
        
        if ((abs(previousMeasuredLenght[1] - currentMeasuredLenght[1]) > 3)) { // Si la distance au setpoint est trop importante
          Serial.println("Beacon ?");
          step = DROP;
          counterForMoving = 0; // On réinitialise le compteur de mouvement
          //stabilityCounterForBeacon++;
        }
        break;

        case DROP:
         counterForMoving++;
        if (counterForMoving < 200){ 
          setRobotVelocity(-0.1, 0);
        }
        else {
          setRobotVelocity(0, 0);
          counterForMoving = 0;
          setArmPosition(-2000); 
          delay(1000);
          gripServo.write(0); 
          Serial.println("Totem déposé");
          totem_grabbed = false; // On a déposé le totem
          step = END; // On passe à l'étape de fin
        }
        break;

        case END: // On a fini le parcours
          if (counterForMoving < 200){ 
          setRobotVelocity(-0.1, 0);
        }  
        else {
          setRobotVelocity(0, 0);
          counterForMoving = 0;
          Serial.println("FIN DU PARCOURS");
          robotActif = false; // On arrête le robot
        }
        break;

    } 
  
   } // FIN DE LA BOUCLE PRINCIPALE
   else {
    // Si le robot n'est pas actif, on arrête les moteurs
    setRobotVelocity(0, 0);
    setArmPosition(0);
  }
}


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
if (leftMotorVel == 0) {
    sendVelocityCommand(MOTOR_ID_LEFT, 0.0000001);
  }
  else {
  sendVelocityCommand(MOTOR_ID_LEFT, leftMotorVel); 
  }  
  readMotorState(MOTOR_ID_LEFT);
  delayMicroseconds(10);
  if (rightMotorVel == 0) {
    sendVelocityCommand(MOTOR_ID_RIGHT, 0.0000001);
  }
  else {
  sendVelocityCommand(MOTOR_ID_RIGHT, rightMotorVel); 
  }
  readMotorState(MOTOR_ID_RIGHT);
  delayMicroseconds(10);

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
    Serial.println("Servo mesure: " + String(analogRead(SERVO_FEEDBACK_PIN)));
  }

  /*************** Asservissement ***************/
  if (MOVE) {  
      Serial.println("--- Asservissement ---");    
      Serial.println ("Erreur : " + String(robotWallOffsetError));   
      Serial.println("Intégrale de l'erreur : " + String(robotWallOffsetErrorIntegrated));       
      Serial.println("Commande : " + String(robotAngularVelocityCommand));
  }
}

void capteur() {
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
    previousMeasuredLenght[i] = currentMeasuredLenght[i]; // On sauvegarde la mesure précédente
    currentMeasuredLenght[i] = Duree * 0.034 / 2; // Calcul de la distance grace au temps mesure (à partir de la vitesse du son)
    delayMicroseconds(1000);
  }
  robotWallOffsetMeasure = currentMeasuredLenght[1]; // On récupère la mesure du capteur latéral pour l'asservissement de distance
}

void setArmPosition(float erreur){
  // 3 - 105.2 111 118.15 129.30 133.7 - 7 HAUTEUR PINCE
  // 9 - 107.42 116.38 122.5 134.5 - 12 CAPTEUR
  // consigne = map(feedbackValue, 107.42,134.5, 105.2, 133.7);
  sendVelocityCommand(MOTOR_ID_ARM, erreur); 
  readMotorState(MOTOR_ID_ARM);
  delayMicroseconds(10);
}