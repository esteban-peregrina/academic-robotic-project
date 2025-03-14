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
#include "mcp2515_can.h"

// For devices communication using the SPI bus
#include <SPI.h>

// For math functions
#include <math.h>

/****************** CONSTANTES *********************/
// Capteurs
#define Broche_Echo 7 // Broche Echo du HC-SR04 sur D7
#define Broche_Trigger 8 // Broche Trigger du HC-SR04 sur D8 

// Bus CAN
#define MAX_DATA_SIZE 8
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
  const int SPI_CS_PIN = BCM8;
  const int CAN_INT_PIN = BCM25;
#else
  const int SPI_CS_PIN = 10; 
  const int CAN_INT_PIN = 2;
#endif
 
// Math (pourra servir)
#define MY_PI 3.14159265359

// Loop properties
#define PERIOD_IN_MICROS 5000 // 5 ms

// Moteurs
#define MOTOR_ID_LEFT 11
#define MOTOR_ID_RIGHT 12
#define MOTOR_ID_ARM 41

#define MOTOR_ARM_MAX_POS_DEG 120.0 // À changer, butée logicielle pour le bras
#define MOTOR_ARM_MIN_POS_DEG -120.0

/****************** DECLARATION DES VARIABLES GLOBALES *********************/
// Capteurs
int MesureMaxi; // Distance maxi a mesurer 
int MesureMini; // Distance mini a mesurer 
long Duree;
long Distance;

// Moteurs
int currentArmMotorPosEncoder; // In raw encoder units
int offsetArmMotorPosEncoder; // In raw encoder units
int currentNumOfArmMotorRevol; // Number

double currentArmMotorPosDeg; // In degrees
double previousArmMotorPosDeg; // In degrees

double currentArmMotorVel; // In degrees per seconds
double currentLeftMotorVel; // In degrees per seconds
double currentRightMotorVel; // In degrees per seconds

// Ports
uint8_t analogPinP0 = A0; 
uint8_t analogPinP1 = A2; 
int currentP0Rawvalue;
int currentP1Rawvalue;

// Général
int counterForPrinting;
int printingPeriodicity;
unsigned long current_time, old_time, initial_time;

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
void motorON();
void motorOFF();
void sendVelocityCommand(long int vel); // This function sends a velocity command. Unit = hundredth of degree per second 
void readMotorState();


void setup() {
  /****************** INITIALISATION DES VARIABLES GLOBALES *********************/
  // Capteurs
  MesureMaxi = 300;
  MesureMini = 3;
  
  //Bus CAN
  mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin

  /*************** MONITEUR SERIE ***************/
  // Initialization of the serial link
  Serial.begin(115200);
  
  counterForPrinting = 0;
  printingPeriodicity = 10; // The variables will be sent to the serial link one out of printingPeriodicity loop runs. 

  /*************** BROCHES ***************/
  // Capteurs
  pinMode(Broche_Trigger, OUTPUT); // Broche Trigger en sortie 
  pinMode(Broche_Echo, INPUT); // Broche Echo en entree 

  // Initialisation des broches analogiques d'entrée
  pinMode(analogPinP0, INPUT);
  pinMode(analogPinP1, INPUT);

  // Lecture initiale
  currentP0Rawvalue = analogRead(analogPinP0);
  currentP1Rawvalue = analogRead(analogPinP1);

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
  initial_time=current_time;
}
void loop() {
  /*
  AJOUTER COGNITION
  */

  // Gestion de la boucle
  unsigned int sleep_time;
  double elapsed_time_in_s;
  old_time = current_time;
  current_time = micros();
  elapsed_time_in_s = (double)(current_time-initial_time);
  elapsed_time_in_s *= 0.000001;
 
  /*************** MOTEURS ***************/
  // Pour le moment c'est pour tester
  // Activation des moteurs
  sendVelocityCommand(MOTOR_ID_LEFT, 100);
  readMotorState(MOTOR_ID_LEFT);
  sendVelocityCommand(MOTOR_ID_RIGHT, 100);
  readMotorState(MOTOR_ID_RIGHT);
  sendVelocityCommand(MOTOR_ID_ARM, 100);
  readMotorState(MOTOR_ID_ARM);

  /*************** CAPTEURS ***************/ 
  // Pour le moment c'est pour tester
  // Debut de la mesure avec un signal de 10 µS applique sur TRIG 
  digitalWrite(Broche_Trigger, LOW); // On efface l'etat logique de TRIG 
  delayMicroseconds(2);
  digitalWrite(Broche_Trigger, HIGH); // On met la broche TRIG a "1" pendant 10µS 
  delayMicroseconds(10);
  digitalWrite(Broche_Trigger, LOW); // On remet la broche TRIG a "0" 
  
  // On mesure combien de temps le niveau logique haut est actif sur ECHO 
  Duree = pulseIn(Broche_Echo, HIGH);
  // Calcul de la distance grace au temps mesure 
  Distance = Duree * 0.034 / 2; // Calcul avec la vitesse du son


  /*************** AFFICHAGE ***************/
  counterForPrinting++;
  if (counterForPrinting > printingPeriodicity) {  // Reset the counter and print
    // Verification si valeur mesuree dans la plage //
    if (Distance >= MesureMaxi || Distance <= MesureMini) {
      // Si la distance est hors plage, on affiche un message d'erreur //
      Serial.println("Distance de mesure en dehors de la plage (3 cm à 3 m)");
    } else {
      // Affichage dans le moniteur serie de la distance mesuree //
      Serial.print("Distance mesuree :");
      Serial.print(Distance);
      Serial.println("cm");
    }

    counterForPrinting = 0;
    Serial.print("t:");
    Serial.println(elapsed_time_in_s);
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

  long int local_velocity; // A SUPPRIMER ?
  local_velocity = velocity; // A SUPPRIMER ?

  unsigned char *adresse_low = (unsigned char *)(&local_velocity); // Convertit l'adresse de la vitesse en une chaine de caractère pour concorder avec la syntaxe du message à transmettre

  unsigned char msg[MAX_DATA_SIZE] = {
    0xA2, // Valeur du potentiomètre ?
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
  uint8_t len;
  byte cdata[MAX_DATA_SIZE] = {0}; // Déclare et remplit le tableau de 0, buffer utilisé pour receptionner les données
  int data2_3, data4_5, data6_7;
  int currentMotorVelRaw;

  // Attend de réceptionner des données
  while (CAN_MSGAVAIL != CAN.checkReceive());

  // Lis les données, len: data length, buf: data buf
  CAN.readMsgBuf(&len, cdata); // Écrit les valeurs du message transmis par le bus (données) CAN dans le buffer cdata
  id = CAN.getCanId(); // Récupère la valeur de l'ID du bus CAN depuis lequel les données sont reçues

  
  if ((id - 0x140) == motorID) { // Si l'ID reçu correspond à celui du moteur
    data4_5 = cdata[4] + pow(2, 8) * cdata[5];
    currentMotorVelRaw = (int)data4_5; // Calcul la vitesse brute
    data6_7 = cdata[6] + pow(2, 8) * cdata[7];
    if (motorID == MOTOR_ID_ARM) currentArmMotorPosEncoder = (int)data6_7;
  }

  // Convertit la vitesse brute en degrés par secondes
  if (motorID == MOTOR_ID_LEFT) currentLeftMotorVelDegPerSec = (double)(currentMotorVelRaw); 
  else if (motorID == MOTOR_ID_RIGHT) currentRightMotorVelDegPerSec = (double)(currentMotorVelRaw); 
  else currentArmMotorVelDegPerSec = (double)(currentMotorVelRaw); 

  if (motorID == MOTOR_ID_ARM) {
    // Déduction de la position en degré à partir de l'offset, du nombre de révolutions, et de la valeur brute en unité encodeur
    currentArmMotorPosEncoder -= offsetArmMotorPosEnconder; // On adapte la position en fonction du décalage introduit initialement (position de départ)
    if ((currentArmMotorPosDeg - previousArmMotorPosDeg) < -20.0) currentNumOfArmMotorRevol++;
    else if ((currentArmMotorPosDeg - previousArmMotorPosDeg) > 20.0) currentNumOfArmMotorRevol--;
    currentArmMotorPosDeg = double)(currentNumOfArmMotorRevol * 360.0 + ((double)currentArmMotorPosEncoder) * 180.0 / 32768.0; // Met à jour la variable globale

    previousArmMotorPosDeg = currentArmMotorPosDeg; // Affecte à la position précédente la valeur de la position courante pour le prochain appel
  }
}

void resetMotor(int motorID) {
  /*
  Commande l'arrêt du moteur puis son démarrage.
  */

  // Initialisation des variables moteurs
  offsetArmMotorPosEncoder = 0; // In raw encoder units
  currentNumOfArmMotorRevol = 0; // Number

  currentArmMotorPosDeg = 0.0; // In degrees

  // Send motor OFF then motor ON command to reset
  motorOFF(motorID);
  delay(500);
  readMotorState(motorID);

  // Attendre que l'utilisateur envoie 'S'
  while (Serial.read() != 'S');

  motorON(motorID);
  readMotorState(motorID);
  delay(500);
  
  sendVelocityCommand((long int)(0)); // Send 0
  delay(500);
  readMotorState(motorID); 

  offsetMotorPosEncoder = currentMotorPosEncoder; // L'offset correspond à la valeur initiale 
  currentNumOfMotorRevol = 0;
  previousMotorPosDeg = 0.0;
  sendVelocityCommand(motorID, (long int)(0));
  delay(500);
  readMotorState(motorID); 

  Serial.println("End of Initialization routine.");
}





