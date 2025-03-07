/*****************************************************************************/
/*********** PROJET DE ROBOTIQUE 3A - POLYTECH SORBONE - MARS 2025 ***********/
/************* AUTHORS : E. PEREGRINA, E. BARNABE & P-L. PASUTTO *************/
/*****************************************************************************/

/****************** BIBLIOTHÈQUES *********************/
// Libraries for CAN communications
#include <can-serial.h>
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>
#include "mcp2515_can.h"


#include <SPI.h>

// Math
#include <math.h>

/****************** CONSTANTES *********************/
// Capteurs
#define Broche_Echo 7 // Broche Echo du HC-SR04 sur D7 //
#define Broche_Trigger 8 // Broche Trigger du HC-SR04 sur D8 //

// Bus CAN
#define MAX_DATA_SIZE 8
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
  const int SPI_CS_PIN = BCM8;
  const int CAN_INT_PIN = BCM25;
#else
  const int SPI_CS_PIN = 10;
  const int CAN_INT_PIN = 2;
#endif
 
// Math
#define MY_PI 3.14159265359

// Loop
#define PERIOD_IN_MICROS 5000 // 5 ms

// Booléens
#if !defined(FALSE)
#define FALSE 0
#define TRUE 1
#endif

// Moteurs
#define MOTOR_ID 1  // Here change with the motor ID if it has been changed with the firmware.
#define MOTOR_MAX_VEL_CMD 300000 
#define MOTOR_MAX_VOLTAGE_CMD 200 
#define MOTOR_MAX_POS_DEG 120.0
#define MOTOR_MIN_POS_DEG -120.0


/****************** DECLARATION DES VARIABLES GLOBALES *********************/
// Capteurs
int MesureMaxi; // Distance maxi a mesurer //
int MesureMini; // Distance mini a mesurer //
long Duree;
long Distance;

// Moteurs
double currentMotorPosDeg;
double currentMotorVelDegPerSec;
double previousMotorPosDeg;
int currentMotorPosEncoder;
int offsetMotorPosEnconder;
int currentNumOfMotorRevol;

// Ports
uint8_t analogPinP0=A0; 
uint8_t analogPinP1=A2; 
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

  /*************** MOTEUR ***************/
  int i;
  char serialReceivedChar;
  int nothingReceived;

  // Initialization of the serial link
  Serial.begin(115200);

  // Initialization of the analog input pins
  pinMode(analogPinP0, INPUT);
  pinMode(analogPinP1, INPUT);
  currentP0Rawvalue = analogRead(analogPinP0);
  currentP1Rawvalue = analogRead(analogPinP1);

  // Initialization of the CAN communication. THis will wait until the motor is powered on
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN init fail, retry ...");
    delay(500);
  }

  // Initialization of the motor command and the position measurement variables
  previousMotorPosDeg = 0.0;
  currentNumOfMotorRevol = 0;
  offsetMotorPosEnconder = 0;

  // Send motot off then motor on command to reset
  motorOFF();
  delay(500);
  readMotorState();

  nothingReceived = TRUE;
  while (nothingReceived==TRUE){
    serialReceivedChar = Serial.read();
    if(serialReceivedChar == 'S') {
      nothingReceived = FALSE;
    }
  }

  motorON();
  readMotorState();
  delay(500);
  
  sendVelocityCommand((long int)(0));
  delay(500);
  readMotorState(); 

  offsetMotorPosEnconder = currentMotorPosEncoder;
  currentNumOfMotorRevol = 0;
  previousMotorPosDeg = 0.0;
  sendVelocityCommand((long int)(0));
  delay(500);
  readMotorState(); 

  Serial.println("End of Initialization routine.");
  counterForPrinting = 0;
  printingPeriodicity = 10; // The variables will be sent to the serial link one out of printingPeriodicity loop runs.  
  current_time = micros(); 
  initial_time=current_time;

  // pinMode(Broche_Trigger, OUTPUT); // Broche Trigger en sortie //
  // pinMode(Broche_Echo, INPUT); // Broche Echo en entree //
  // Serial.begin (115200);
}
void loop() {
  /*
  AJOUTER COGNITION
  */

  int i;
  unsigned int sleep_time;
  double elapsed_time_in_s;
  double motorVelocityCommandInDegPerSec;
  double gainPotValToVel;

  // STEP 1 : dealing with clock
  old_time=current_time;
  current_time=micros();
  elapsed_time_in_s = (double)(current_time-initial_time);
  elapsed_time_in_s *= 0.000001;
 
  // STEP 2 : Reading the potentiometers
  currentP0Rawvalue = analogRead(analogPinP0);
  //currentP1Rawvalue = analogRead(analogPinP1);
  currentP1Rawvalue = 512;

  gainPotValToVel = 200.0;
  motorVelocityCommandInDegPerSec =  gainPotValToVel * ((double)(currentP1Rawvalue-512)) ; 
  sendVelocityCommand((long int)(motorVelocityCommandInDegPerSec));
  readMotorState();

  counterForPrinting++;
  if (counterForPrinting > printingPeriodicity) {  // Reset the counter and print
    counterForPrinting = 0;
    Serial.print("t:");
    Serial.print(elapsed_time_in_s);
    Serial.print(",P0:");
    Serial.print(currentP0Rawvalue);
    Serial.print(",P1:");
    Serial.println(currentP1Rawvalue);
  }

  sleep_time = PERIOD_IN_MICROS-((micros()-current_time));
  if ( (sleep_time >0) && (sleep_time < PERIOD_IN_MICROS) ) {
    delayMicroseconds(sleep_time);
  }


  // // Debut de la mesure avec un signal de 10 µS applique sur TRIG //
  // digitalWrite(Broche_Trigger, LOW); // On efface l'etat logique de TRIG //
  // delayMicroseconds(2);
  // digitalWrite(Broche_Trigger, HIGH); // On met la broche TRIG a "1" pendant 10µS //
  // delayMicroseconds(10);
  // digitalWrite(Broche_Trigger, LOW); // On remet la broche TRIG a "0" //
  
  // // On mesure combien de temps le niveau logique haut est actif sur ECHO //
  // Duree = pulseIn(Broche_Echo, HIGH);
  // // Calcul de la distance grace au temps mesure //
  // Distance = Duree*0.034/2; // *** voir explications apres l'exemple de code *** //
  
  // // Verification si valeur mesuree dans la plage //
  // if (Distance >= MesureMaxi || Distance <= MesureMini) {
  //   // Si la distance est hors plage, on affiche un message d'erreur //
  //   Serial.println("Distance de mesure en dehors de la plage (3 cm à 3 m)");
  // } else {
  //   // Affichage dans le moniteur serie de la distance mesuree //
  //   Serial.print("Distance mesuree :");
  //   Serial.print(Distance);
  //   Serial.println("cm");
  // }
  // delay(1000); // On ajoute 1 seconde de delais entre chaque mesure //
}

/****************** IMPLEMENTATION DES FONCTIONS *********************/

void motorON() {
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

  CAN.sendMsgBuf(0x140+MOTOR_ID, 0, 8, msg); 
}

void motorOFF() {
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

  CAN.sendMsgBuf(0x140+MOTOR_ID, 0, 8, msg); 
}


void sendVelocityCommand(long int vel) {
  /* 
  This function sends a velocity command. Unit = hundredth of degree per second 
  */

  long int local_velocity;
  local_velocity = vel;

  unsigned char *adresse_low = (unsigned char *)(&local_velocity);

  unsigned char msg[MAX_DATA_SIZE] = {
    0xA2,
    0x00,
    0x00,
    0x00,
    *(adresse_low),
    *(adresse_low + 1),
    *(adresse_low + 2),
    *(adresse_low + 3)

  };

  CAN.sendMsgBuf(0x140+MOTOR_ID, 0, 8, msg); 
}


void readMotorState() {
  /*
  This function reads the motor states and affects the following global variables
  previousMotorPosDeg : previous position in Deg (set to the new value, will be "previous" for next call);
  currentNumOfMotorRevol = number of  revoutions (old value +/- 1 if one jump has been observed ont the encoder read)
  currentEncodPos = current Encoder Pos value (read)
  currentMotorPosDeg = Motor position in degrees
  currentMotorVelDegPerSec = Motor position velocity in degrees per s
  */

  uint32_t id;
  uint8_t type;
  uint8_t len;
  byte cdata[MAX_DATA_SIZE] = { 0 };
  int data2_3, data4_5, data6_7;
  int currentMotorVelRaw;

  // wait for data
  while (CAN_MSGAVAIL != CAN.checkReceive())
    ;

  // read data, len: data length, buf: data buf
  CAN.readMsgBuf(&len, cdata);

  id = CAN.getCanId();
  type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);

  // Check if the received ID matches the motor ID
  if ((id - 0x140) == MOTOR_ID) { 
    data4_5 = cdata[4] + pow(2, 8) * cdata[5];
    currentMotorVelRaw = (int)data4_5;
    data6_7 = cdata[6] + pow(2, 8) * cdata[7];
    currentMotorPosEncoder = (int)data6_7;
  }

  // Conversion of the velocity and writing in the global cariable
  currentMotorVelDegPerSec = ((double)(currentMotorVelRaw)); 

  // Conversion of the position (with motor revolution counting) and wirting in the global variable
  currentMotorPosEncoder -= offsetMotorPosEnconder;
  currentMotorPosDeg = ((double)(currentNumOfMotorRevol)*360.0) + (((double)currentMotorPosEncoder) * 180.0 / 32768.0);  // On convertit en degré

  if ((currentMotorPosDeg - previousMotorPosDeg) < -20.0) {
    currentNumOfMotorRevol++;
    currentMotorPosDeg = ((double)(currentNumOfMotorRevol)) * 360.0 + ((double)currentMotorPosEncoder) * 180.0 / 32768.0;
  }
  if ((currentMotorPosDeg - previousMotorPosDeg) > 20.0) {
    currentNumOfMotorRevol--;
    currentMotorPosDeg = ((double)(currentNumOfMotorRevol)) * 360.0 + ((double)currentMotorPosEncoder) * 180.0 / 32768.0;
  }

  previousMotorPosDeg = currentMotorPosDeg; // writing in the global variable for next call
}





