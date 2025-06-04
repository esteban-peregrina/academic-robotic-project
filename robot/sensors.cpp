#include "sensors.h"

/****************** DECLARATION DES VARIABLES GLOBALES *********************/
// Capteurs
const int MesureMaxi = 50; // Distance maxi a mesurer 
const int MesureMini = 5; // Distance mini a mesurer 
long currentMeasuredLenght[NB_OF_SENSORS] = {0, 0};
long previousMeasuredLenght[NB_OF_SENSORS] = {0, 0};