#ifndef MOTEURS_H
#define MOTEURS_H

#define NB_OF_MOTORS 3 // Global motor variables array lenght, make sure ID matchs array boundaries as they will be used as index
#define MOTOR_ID_LEFT 0x01
#define MOTOR_ID_RIGHT 0x02
#define MOTOR_ID_ARM 0x03

// Commandes
#define MOTOR_MAX_VEL_CMD 300000 
#define MOTOR_MAX_VOLTAGE_CMD 200 
#define MOTOR_ARM_MAX_POS_DEG 120.0 // À changer, butée logicielle pour le bras
#define MOTOR_ARM_MIN_POS_DEG -120.0

// Variables globales pour les moteurs
extern int relativeMotorPosEncoder[NB_OF_MOTORS]; // In raw encoder units
extern int offsetMotorPosEncoder[NB_OF_MOTORS]; // In raw encoder units
extern int currentNumOfMotorRevol[NB_OF_MOTORS]; // Number

extern double currentMotorPosDeg[NB_OF_MOTORS]; // In degrees
extern double previousMotorPosDeg[NB_OF_MOTORS]; // In degrees

extern double currentMotorVel[NB_OF_MOTORS]; // In degrees per seconds

// Prototypes et signatures des fonctions 
void motorON(int motorID);
void motorOFF(int motorID);
void resetMotor(int motorID);
void readMotorState(int motorID);
void sendVelocityCommand(int motorID, long int velocity); // Sends a velocity command to motor. Unit = hundredth of degree per second
void motorsInitialization(); 

#endif