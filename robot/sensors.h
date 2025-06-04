#ifndef CAPTEURS_H
#define CAPTEURS_H

// HC-SR04
#define NB_OF_SENSORS 2
#define SENSOR_ECHO_PIN_RIGHT 6
#define SENSOR_TRIGGER_PIN_RIGHT 7
#define SENSOR_ECHO_PIN_FRONT 3
#define SENSOR_TRIGGER_PIN_FRONT 4
#define SERVO_TRIGGER_PIN 5
#define SERVO_FEEDBACK_PIN A0

const int triggerPins[NB_OF_SENSORS] = {SENSOR_TRIGGER_PIN_FRONT, SENSOR_TRIGGER_PIN_RIGHT};
const int echoPins[NB_OF_SENSORS] = {SENSOR_ECHO_PIN_FRONT, SENSOR_ECHO_PIN_RIGHT};

extern const int MesureMaxi; // Distance maxi a mesurer 
extern const int MesureMini; // Distance mini a mesurer 
extern double currentMeasuredLenght[NB_OF_SENSORS];
extern double previousMeasuredLenght[NB_OF_SENSORS];

#endif