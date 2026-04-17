/****************************************************************
*
*
* A CANOpen project file example
* Created by Ign555
* Version : v1.0
* File Creation : 17/04/2026
*
*
****************************************************************/

#include <mcp_can.h>
#include <SPI.h>
#include "canny_open.hpp"

#define CAN0_INT 2 // Set INT to pin 2

CANOpen canopen(CAN_1000KBPS, CAN0_INT);

void setup()
{

  Serial.begin(115200); //Start serial communication with 115200 b/s 
  canopen.begin(); // Start Canopen communication

}

void loop() {

  if(!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    canopen.poll_message(); //Read CANMessage
  }
  
  canopen.tick(); //Get the tick for the time management

}

  
