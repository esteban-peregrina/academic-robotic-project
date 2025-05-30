#ifndef CONFIG_H
#define CONFIG_H

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
extern mcp2515_can CAN;  // Set CS pin
#define MAX_DATA_SIZE 8

#endif