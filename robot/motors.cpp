#include "CANconfig.h"
#include "motors.h"

/****************** DECLARATION DES VARIABLES GLOBALES *********************/
int relativeMotorPosEncoder[NB_OF_MOTORS] = {0};
int offsetMotorPosEncoder[NB_OF_MOTORS] = {0};
int currentNumOfMotorRevol[NB_OF_MOTORS] = {0};

double currentMotorPosDeg[NB_OF_MOTORS] = {0.0};
double previousMotorPosDeg[NB_OF_MOTORS] = {0.0};

double currentMotorVel[NB_OF_MOTORS] = {0.0};

/****************** DECLARATION DES FONCTIONS *********************/
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
  
    Précondition : Le moteur doit être correctement branché.
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
    delay(250);
    if (readMotorState(motorID) == -1) {
      Serial.println("Motor " + String(motorID) + " did not answered.");
      return;
    };
    motorON(motorID);
    readMotorState(motorID);
    delay(250);

    // Send null velocity
    sendVelocityCommand(motorID, (long int)(0)); // Send 0
    delay(250);
    readMotorState(motorID); 
  
    offsetMotorPosEncoder[motorID - 1] = relativeMotorPosEncoder[motorID - 1]; // L'offset correspond à la valeur initiale 
    currentNumOfMotorRevol[motorID - 1] = 0; // Number
    previousMotorPosDeg[motorID - 1] = 0.0;
    sendVelocityCommand(motorID, (long int)(0));
    delay(250);
    readMotorState(motorID); 
  
    Serial.println("Motor " + String(motorID) + " was successfully reset.");
  }
  
int readMotorState(int motorID) {
    /*
    Récupère l'état du moteur identifié via les mesures de l'encodeur, et met à jour la valeur des variables globales suivantes :
    - currentNumOfMotorRevol : Nombre de révolutions du moteur (valeur précédente +/- 1 selon si une révolution à été observé par l'encodeur)
    - currentArmMotorPosDeg : Position du moteur en degrés, calculée à partir des valeurs brutes de l'encodeur
    - current(Left/Right/Arm)MotorVel : Vitesse du moteur, en degrés par secondes
    - previousArmMotorPosDeg : Position actuelle, en degrés (car la position actuelle deviendra la précédente au prochain appel)
    */
  
    uint32_t id;
    uint8_t type;
    uint8_t len;
    byte cdata[MAX_DATA_SIZE] = {0}; // Déclare et remplit le tableau de 0, buffer utilisé pour receptionner les données
    int data2_3, data4_5, data6_7;
    int rawMotorVel;
    int absoluteMotorPosEncoder;

    int timeout;

    // Attend de réceptionner des données
    while (CAN_MSGAVAIL != CAN.checkReceive() && timeout < 5000) {
      delayMicroseconds(1000);
      timeout++;
    }
    if (timeout > 5000) return -1;
    // Lis les données, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata); // Écrit les valeurs du message transmis par le bus (données) CAN dans le buffer cdata
    id = CAN.getCanId(); // Récupère la valeur de l'ID du bus CAN depuis lequel les données sont reçues
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);
    
    if ((id - 0x140) == motorID) { // Si l'ID reçu correspond à celui du moteur
      data4_5 = cdata[4] + 256 * cdata[5];
      rawMotorVel = (int)data4_5; // Calcul la vitesse brute
      data6_7 = cdata[6] + 256 * cdata[7];
      absoluteMotorPosEncoder = (int)data6_7;
    }
  
    // Convertit la vitesse brute en degrés par secondes
    currentMotorVel[motorID - 1] = (double)rawMotorVel;
    
    relativeMotorPosEncoder[motorID - 1] = (double)absoluteMotorPosEncoder;
  
    // Déduction de la position en degré à partir de l'offset, du nombre de révolutions, et de la valeur brute en unité encodeur
    relativeMotorPosEncoder[motorID - 1] -= offsetMotorPosEncoder[motorID - 1]; // On adapte la position en fonction du décalage introduit initialement (position de départ)
    currentMotorPosDeg[motorID - 1] = 180.0 + ((double)relativeMotorPosEncoder[motorID - 1]) * (180.0 / 32768.0);  // Met à jour la variable globale
  
    double delta = currentMotorPosDeg[motorID - 1] - previousMotorPosDeg[motorID - 1];
    if (delta > 180.0) {
        currentNumOfMotorRevol[motorID - 1]--;
    } else if (delta < -180.0) {
        currentNumOfMotorRevol[motorID - 1]++;
    }
  
    // Affecte à la position précédente la valeur de la position courante pour le prochain appel
    previousMotorPosDeg[motorID - 1] = currentMotorPosDeg[motorID - 1]; // writing in the global variable for next call

    return 0;
  }
  
void sendVelocityCommand(int motorID, long int velocity) {
    /* 
    Envoie la commande de vitesse spécifiée au moteur identifié. 
  
    Précondition : La vitesse doit être exprimée en centième de degrés par seconde.
    */
  
    if (abs(velocity) > MOTOR_MAX_VEL_CMD) {
      velocity = (velocity > 0 ? MOTOR_MAX_VEL_CMD : -MOTOR_MAX_VEL_CMD);
    }
  
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

void motorsInitialization() {
    // Initialisation des moteurs
    resetMotor(MOTOR_ID_LEFT);
    resetMotor(MOTOR_ID_RIGHT);
    resetMotor(MOTOR_ID_ARM);
}