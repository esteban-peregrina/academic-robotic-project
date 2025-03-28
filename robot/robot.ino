/*****************************************************************************/
/********** PROJET DE ROBOTIQUE, 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTEURS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** BIBLIOTHÈQUES *********************/
// For CAN communications
#include <can-serial.h>
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>
// Bus CAN
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
  const int SPI_CS_PIN = BCM8;
  const int CAN_INT_PIN = BCM25;
#else
  const int SPI_CS_PIN = 10; //9 ou 10
  const int CAN_INT_PIN = 2;
#endif
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#define MAX_DATA_SIZE 8

// For devices communication using the SPI bus
#include <SPI.h>

// For math functions
#include <math.h>

/****************** CONSTANTES *********************/
// Capteurs
#define Broche_Echo 7 // Broche Echo du HC-SR04 sur D7
#define Broche_Trigger 8 // Broche Trigger du HC-SR04 sur D8 


 
// Math (pourra servir)
#define MY_PI 3.14159265359

// Loop properties
#define PERIOD_IN_MICROS 5000 // 5 ms

// Moteurs
#define NB_OF_MOTORS 3 // Global motor variables array lenght, make sure ID matchs array boundaries as they will be used as index
#define MOTOR_ID_LEFT 0x01
#define MOTOR_ID_RIGHT 0x02
#define MOTOR_ID_ARM 0x03

#define MOTOR_MAX_VEL_CMD 300000 
#define MOTOR_MAX_VOLTAGE_CMD 200 
#define MOTOR_ARM_MAX_POS_DEG 120.0 // À changer, butée logicielle pour le bras
#define MOTOR_ARM_MIN_POS_DEG -120.0

/****************** DECLARATION DES VARIABLES GLOBALES *********************/
// Capteurs
int MesureMaxi; // Distance maxi a mesurer 
int MesureMini; // Distance mini a mesurer 
long Duree;
long Distance;

// Moteurs
int absoluteMotorPosEncoder[NB_OF_MOTORS]; // In raw encoder units
int offsetMotorPosEncoder[NB_OF_MOTORS]; // In raw encoder units
int currentNumOfMotorRevol[NB_OF_MOTORS]; // Number

double currentMotorPosDeg[NB_OF_MOTORS]; // In degrees
double previousMotorPosDeg[NB_OF_MOTORS]; // In degrees

double currentMotorVel[NB_OF_MOTORS]; // In degrees per seconds

// Ports
// uint8_t analogPinP0 = A0; 
// uint8_t analogPinP1 = A2; 
// int currentP0Rawvalue;
// int currentP1Rawvalue;

// Général
int counterForPrinting;
int printingPeriodicity;
unsigned long current_time, old_time, initial_time;

// Debugg


/****************** DECLARATION DES FONCTIONS *********************/
// Capteurs
bool obstacleEnFace();
bool obstacleADroite(); 

// Déplacement
void deplacementAngulaire(float deg);
void deplacementLineaire(float dist);

// Saisie
void saisir();

// Moteurs
void motorON(int motorID);
void motorOFF(int motorID);
void sendVelocityCommand(long int vel); // This function sends a velocity command. Unit = hundredth of degree per second 
void readMotorState(int motorID);
void resetMotor(int motorID);


void setup() {
  /****************** INITIALISATION DES VARIABLES GLOBALES *********************/
  // Capteurs
  MesureMaxi = 300;
  MesureMini = 3;

  /*************** MONITEUR SERIE ***************/
  // Initialization of the serial link
  Serial.begin(115200);
  counterForPrinting = 0;
  printingPeriodicity = 50; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. Every printingPeriodicity * PERIODS_IN_MICROS

  /*************** BROCHES ***************/
  // Capteurs
  // pinMode(Broche_Trigger, OUTPUT); // Broche Trigger en sortie 
  // pinMode(Broche_Echo, INPUT); // Broche Echo en entree 

  /*************** BUS CAN ***************/ 
  // Démarrage de la communication avec le bus CAN
  while (CAN.begin(CAN_500KBPS) != CAN_OK) { // Tant que le bus CAN ne reçoit rien 
    Serial.println("CAN initialization failed, trying again...");
    delay(500);
  } // On sort de la boucle si au moins un moteur à envoyé des données sur le bus CAN

  /*************** MOTEURS ***************/ 
  // Réinitialisation du moteur
  resetMotor(MOTOR_ID_LEFT);
  resetMotor(MOTOR_ID_RIGHT);
  resetMotor(MOTOR_ID_ARM);
  /*************** MESURES ***************/ 
  current_time = micros(); 
  initial_time = current_time;
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
  // Pour le moment c'est pour tester
  // Activation des moteurs
  sendVelocityCommand(MOTOR_ID_LEFT, 1000);
  readMotorState(MOTOR_ID_LEFT);
  delayMicroseconds(10000);
  sendVelocityCommand(MOTOR_ID_RIGHT, 1000);
  readMotorState(MOTOR_ID_RIGHT);
  delayMicroseconds(10000);
  sendVelocityCommand(MOTOR_ID_ARM, 1000);
  readMotorState(MOTOR_ID_ARM);
  delayMicroseconds(10000);

  /*************** CAPTEURS ***************/ 
  // Pour le moment c'est pour tester
  // Debut de la mesure avec un signal de 10 µS applique sur TRIG 
  // digitalWrite(Broche_Trigger, LOW); // On efface l'etat logique de TRIG 
  // delayMicroseconds(2);
  // digitalWrite(Broche_Trigger, HIGH); // On met la broche TRIG a "1" pendant 10µS 
  // delayMicroseconds(10);
  // digitalWrite(Broche_Trigger, LOW); // On remet la broche TRIG a "0" 
  
  // // On mesure combien de temps le niveau logique haut est actif sur ECHO 
  // Duree = pulseIn(Broche_Echo, HIGH);
  // // Calcul de la distance grace au temps mesure 
  // Distance = Duree * 0.034 / 2; // Calcul avec la vitesse du son


  /*************** AFFICHAGE ***************/
  counterForPrinting++;
  if (counterForPrinting > printingPeriodicity) {  // Reset the counter and print
    // Verification si valeur mesuree dans la plage //
    // if (Distance >= MesureMaxi || Distance <= MesureMini) {
    //   // Si la distance est hors plage, on affiche un message d'erreur //
    //   Serial.println("Distance de mesure en dehors de la plage (3 cm à 3 m)");
    // } else {
    //   // Affichage dans le moniteur serie de la distance mesuree //
    //   Serial.print("Distance mesuree :");
    //   Serial.print(Distance);
    //   Serial.println("cm");
    // }
    counterForPrinting = 0;
                    
    Serial.println("--- Motor 1 ---");
    Serial.print("t:");
    Serial.println(elapsed_time_in_s);
    Serial.print("currentNumOfMotorRevol[0]:");
    Serial.println(currentNumOfMotorRevol[0]);
    Serial.print("currentMotorPosDeg[0]:");
    Serial.println(currentMotorPosDeg[0]);
    Serial.print("currentMotorVel[0]:");
    Serial.println(currentMotorVel[0]);
    Serial.print("offsetMotorPosEncoder[0]:");
    Serial.println(offsetMotorPosEncoder[0]);

    Serial.println("--- Motor 2 ---");
    Serial.print("t:");
    Serial.println(elapsed_time_in_s);
    Serial.print("currentNumOfMotorRevol[1]:");
    Serial.println(currentNumOfMotorRevol[1]);
    Serial.print("currentMotorPosDeg[1]:");
    Serial.println(currentMotorPosDeg[1]);
    Serial.print("currentMotorVel[1]:");
    Serial.println(currentMotorVel[1]);
    Serial.print("offsetMotorPosEncoder[1]:");
    Serial.println(offsetMotorPosEncoder[1]);
  
    Serial.println("--- Motor 3 ---");
    Serial.print("t:");
    Serial.println(elapsed_time_in_s);
    Serial.print("currentNumOfMotorRevol[2]:");
    Serial.println(currentNumOfMotorRevol[2]);
    Serial.print("currentMotorPosDeg[2]:");
    Serial.println(currentMotorPosDeg[2]);
    Serial.print("currentMotorVel[2]:");
    Serial.println(currentMotorVel[2]);
    Serial.print("offsetMotorPosEncoder[2]:");
    Serial.println(offsetMotorPosEncoder[2]);
  }
  


  // Patienter pour respecter la fréquence d'itération de la boucle
  sleep_time = PERIOD_IN_MICROS - (micros() - current_time);
  if ( (sleep_time > 0) && (sleep_time < PERIOD_IN_MICROS) ) delayMicroseconds(sleep_time); // On patiente le temps restant pour respecter la fréquence d'itération (SUPPOSE QUE LES INSTRUCTIONS SONT RÉALISABLES DURANT LA PERIODE)
} // FIN DE LA BOUCLE PRINCIPALE

/****************** IMPLEMENTATION DES FONCTIONS *********************/

void motorON(int motorID) {
  /*
  Allume le moteur identifié.

  Précondition : Le moteur doit être correctement branché.
  */
  unsigned char msg[MAX_DATA_SIZE] = {
    0x88,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };

  CAN.sendMsgBuf(0x140 + motorID, 0, 8, msg); // Transmets le message au buffer du bus CAN, retourne CAN_OK ou CAN_FAIL
}

void motorOFF(int motorID) {
  /*
  Éteint le moteur identifié.
  */
  unsigned char msg[MAX_DATA_SIZE] = {
    0x80,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };

  CAN.sendMsgBuf(0x140 + motorID, 0, 8, msg); // Transmets le message au buffer du bus CAN, retourne CAN_OK ou CAN_FAIL
}


void sendVelocityCommand(int motorID, long int velocity) {
  /* 
  Envoie la commande de vitesse spécifiée au moteur identifié. 

  Précondition : La vitesse doit être exprimée en centième de degrés par seconde.
  */

  long int local_velocity; 
  local_velocity = velocity;

  unsigned char *adresse_low = (unsigned char *)(&local_velocity); // Convertit l'adresse de la vitesse en une chaine de caractère pour concorder avec la syntaxe du message à transmettre

  unsigned char msg[MAX_DATA_SIZE] = {
    0xA2, 
    0x00,
    0x00,
    0x00,
    *(adresse_low), // Déréférence l'adresse (le premier caractère de la chaîne)
    *(adresse_low + 1), // 2ème caractère
    *(adresse_low + 2), // 3ème caractère
    *(adresse_low + 3) // 4ème caractère

  };

  CAN.sendMsgBuf(0x140 + motorID, 0, 8, msg); // Transmets le message au buffer du bus CAN, retourne CAN_OK ou CAN_FAIL
}


void readMotorState(int motorID) {
  /*
  Récupère l'état du moteur identifié via les mesures de l'encodeur, et met à jour la valeur des variables globales suivantes :
    currentNumOfMotorRevol : Nombre de révolutions du moteur (valeur précédente +/- 1 selon si une révolution à été observé par l'encodeur)
    currentArmMotorPosDeg : Position du moteur en degrés, calculée à partir des valeurs brutes de l'encodeur
    current[Left/Right/Arm]MotorVel : Vitesse du moteur, en degrés par secondes
    previousArmMotorPosDeg : Position actuelle, en degrés (car la position actuelle deviendra la précédente au prochain appel)
  */

  uint32_t id;
  uint8_t type;
  uint8_t len;
  byte cdata[MAX_DATA_SIZE] = {0}; // Déclare et remplit le tableau de 0, buffer utilisé pour receptionner les données
  int data2_3, data4_5, data6_7;
  int rawMotorVel;
  int rawMotorPosEncoder;

  // Attend de réceptionner des données
  while (CAN_MSGAVAIL != CAN.checkReceive());
  // Lis les données, len: data length, buf: data buf
  CAN.readMsgBuf(&len, cdata); // Écrit les valeurs du message transmis par le bus (données) CAN dans le buffer cdata
  id = CAN.getCanId(); // Récupère la valeur de l'ID du bus CAN depuis lequel les données sont reçues
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
  
  if ((id - 0x140) == motorID) { // Si l'ID reçu correspond à celui du moteur
    data4_5 = cdata[4] + pow(2, 8) * cdata[5];
    rawMotorVel = (int)data4_5; // Calcul la vitesse brute
    data6_7 = cdata[6] + pow(2, 8) * cdata[7];
    rawMotorPosEncoder = (int)data6_7;
  }

  // Convertit la vitesse brute en degrés par secondes
  currentMotorVel[motorID - 1] = (double)rawMotorVel;
  
  absoluteMotorPosEncoder[motorID - 1] = (double)rawMotorPosEncoder;

  // Déduction de la position en degré à partir de l'offset, du nombre de révolutions, et de la valeur brute en unité encodeur
  absoluteMotorPosEncoder[motorID - 1] -= offsetMotorPosEncoder[motorID - 1]; // On adapte la position en fonction du décalage introduit initialement (position de départ)
  currentMotorPosDeg[motorID - 1] = ((double)(currentNumOfMotorRevol[motorID - 1])) * 360.0 + ((double)absoluteMotorPosEncoder[motorID - 1]) * (180.0 / 32768.0);  // Met à jour la variable globale

  if ((currentMotorPosDeg[motorID - 1] - previousMotorPosDeg[motorID - 1]) < -20.0) {
    currentNumOfMotorRevol[motorID - 1]++;
    currentMotorPosDeg[motorID - 1] = ((double)(currentNumOfMotorRevol[motorID - 1])) * 360.0 + ((double)absoluteMotorPosEncoder[motorID - 1]) * (180.0 / 32768.0); // Met à jour la variable globale
  }
  if ((currentMotorPosDeg[motorID - 1] - previousMotorPosDeg[motorID - 1]) > 20.0) {
    currentNumOfMotorRevol[motorID - 1]--;
    currentMotorPosDeg[motorID - 1] = ((double)(currentNumOfMotorRevol[motorID - 1])) * 360.0 + ((double)absoluteMotorPosEncoder[motorID - 1]) * (180.0 / 32768.0); // Met à jour la variable globale
  }
  // Affecte à la position précédente la valeur de la position courante pour le prochain appel
  previousMotorPosDeg[motorID - 1] = currentMotorPosDeg[motorID - 1]; // writing in the global variable for next call

}

void resetMotor(int motorID) {
  /*
  Commande l'arrêt du moteur puis son démarrage.
  */

  // Initialisation des variables moteurs
  offsetMotorPosEncoder[motorID - 1] = 0; // In raw encoder units
  currentNumOfMotorRevol[motorID - 1] = 0; // Number

  currentMotorPosDeg[motorID - 1] = 0.0; // In degrees

  // Send motor OFF then motor ON command to reset
  motorOFF(motorID);
  delay(500);
  readMotorState(motorID);
  // Attendre que l'utilisateur envoie 'S'
  //while (Serial.read() != 'S');

  motorON(motorID);
  readMotorState(motorID);
  delay(500);
  
  sendVelocityCommand(motorID, (long int)(0)); // Send 0
  delay(500);
  readMotorState(motorID); 

  offsetMotorPosEncoder[motorID - 1] = absoluteMotorPosEncoder[motorID - 1]; // L'offset correspond à la valeur initiale 
  currentNumOfMotorRevol[motorID - 1] = 0; // Number
  previousMotorPosDeg[motorID - 1] = 0.0;
  sendVelocityCommand(motorID, (long int)(0));
  delay(500);
  readMotorState(motorID); 

  Serial.println("End of Initialization routine.");
}





