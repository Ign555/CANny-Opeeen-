/****************************************************************
*
*
* CAN Open library for Arduino
* Created by Ign555 for the CEA ( Commissariat à l'énergie atomique )
* Version : v1.0
* File Creation : 02/2026
*
*
****************************************************************/

#include "canny_open.hpp"

//#define __DEBUG__

extern "C" const char heartbeat_state_values_JSON[];

/****************************************************************
*
* CANOpen methods
*
****************************************************************/

/******************************CANOpen constructor & destroyer******************************/

CANOpen::CANOpen(int baudrate, uint8_t int_pin){

  this->__baudrate = baudrate;
  this->__can_int_pin = int_pin;

  this->__CAN0 = new MCP_CAN(9);

}

CANOpen::~CANOpen(){
}

/******************************CANOpen private methods******************************/

void CANOpen::__send_CAN_message(INT32U id, INT8U ext, INT8U len, INT8U *buf){

  byte sndStat = this->__CAN0->sendMsgBuf(id, ext, len, buf);

  #ifdef __DEBUG__

    if(sndStat == CAN_OK){
      Serial.println("Message Sent Successfully!");
    } else {
      Serial.println("Error Sending Message...");
    }

  #else
    if(sndStat != CAN_OK){
      Serial.println("[Error] Check your CAN module");
    }
  #endif
  
}

uint8_t CANOpen::__get_node_id(){

  uint8_t node_id = 2; //Device node ID

  return node_id;

}

/******************************CANOpen begin method******************************/

void CANOpen::begin(){
  
  //Begin CAN module ( initialise )
  if(this->__CAN0->begin(MCP_ANY, this->__baudrate, MCP_16MHZ) == CAN_OK)
      Serial.println("MCP2515 Initialized Successfully!");
  else
      Serial.println("Error Initializing MCP2515...");
  
  this->__CAN0->setMode(MCP_NORMAL); //Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(this->__can_int_pin, INPUT); //Configuring pin for /INT input

  this->enable_heartbeat_timer(DEFAULT_HEARTBEAT_DELAY); //Default value for heartbeat delay
  
  //Serial.println(this->state_dictionary->json());

}

/******************************CANOpen timer methods******************************/

//Get the tick for each timer
void CANOpen::tick(){
  if(this->is_heartbeat_timer_enable)this->heartbeat_timer.tick();
  if(this->is_sync_timer_enable)this->sync_timer.tick();
}

//Synch

void CANOpen::enable_sync_timer(uint32_t delay){
  this->sync_period = delay;
  this->is_sync_timer_enable = true;
  this->sync_timer.every(delay, sync, this);
}

void CANOpen::disable_sync_timer(){
  this->sync_period = 0;
  this->is_sync_timer_enable = false;
  this->sync_timer.cancel();
}

bool CANOpen::sync(void *can_open){

  CANOpen *co = (CANOpen *)can_open;
  uint8_t data[SIZE_OF_CANOPEN_FRAME] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  co->__CAN0->sendMsgBuf(SYNCH_COBID, 0, 8, data);
  Serial.println("Sync sent !");

  return true;

}

//Heartbeat

void CANOpen::enable_heartbeat_timer(uint32_t delay){
  this->is_heartbeat_timer_enable = true;
  this->heartbeat_timer.every(delay, heartbeat, this);
}

void CANOpen::disable_heartbeat_timer(){
  this->is_heartbeat_timer_enable = false;
  this->heartbeat_timer.cancel();
}

bool CANOpen::heartbeat(void *can_open){
  
  CANOpen *co = (CANOpen *)can_open;
  uint8_t data[SIZE_OF_CANOPEN_FRAME] = {0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //Heartbeat frame
  co->__CAN0->sendMsgBuf(HEARTBEAT_COBID | co->__get_node_id(), 0, 8, data);

  #ifdef __DEBUG__
    Serial.println("Heartbeat sent !");
  #endif

  return true;

}

/******************************CANOpen NMT methods******************************/

void CANOpen::NMT_set_mode(uint8_t node_id, uint8_t mode){

  if(mode != CANOPEN_NMT_PREOPERATIONAL && 
     mode != CANOPEN_NMT_OPERATIONAL && 
     mode != CANOPEN_NMT_STOP && 
     mode != CANOPEN_NMT_RESET && 
     mode != CANOPEN_NMT_RESETCOMMUNICATION){
    
    Serial.println("Invalid nmt mode");
    return;

  }

  uint8_t Data [SIZE_OF_CANOPEN_FRAME] = {  mode,                          //Data 0 - SDO Read Request
                                            node_id,                       //Data 1 - Object Index(LSB)
                                            0x00,                          //Data 2 - Object Index(MSB)
                                            0x00,                          //Data 3 - SubIndex
                                            0x00,                          //Data 4 - LSB First Data 
                                            0x00,                          //Data 5
                                            0x00,                          //Data 6
                                            0x00                           //Data 7
                                          };
  Serial.println("message sent");
  this->__send_CAN_message(0x000, 0, 8, Data);

}

/******************************CANOpen SDO methods******************************/

void CANOpen::SDO_send_read_request(uint8_t address , uint16_t object, uint8_t subindex){

  uint8_t Data [SIZE_OF_CANOPEN_FRAME] = {  SDO_READ_REQUEST ,            //Data 0 - SDO Read Request
                                            (uint8_t)(object) ,           //Data 1 - Object Index(LSB)
                                            (uint8_t)(object >> 8) ,      //Data 2 - Object Index(MSB)
                                            subindex,                     //Data 3 - SubIndex
                                            0x00,                         //Data 4 - LSB First Data 
                                            0x00,                         //Data 5
                                            0x00,                         //Data 6
                                            0x00                          //Data 7
                                          };
                       
  this->__send_CAN_message((SDO_ASK_COBID + address), 0, 8, Data);

}

void CANOpen::SDO_write_request(uint8_t address , uint16_t object, uint8_t subindex, uint8_t data[4]){

  uint8_t Data [SIZE_OF_CANOPEN_FRAME] = {  SDO_WRITE_REQUEST ,              //Data 0 - SDO Read Request
                                            (uint8_t)(object) ,              //Data 1 - Object Index(LSB)
                                            (uint8_t)(object >> 8) ,         //Data 2 - Object Index(MSB)
                                            subindex,                        //Data 3 - SubIndex
                                            data[3],                         //Data 4 - LSB First Data 
                                            data[2],                         //Data 5
                                            data[1],                         //Data 6
                                            data[0]                          //Data 7
                                          };
                       
  this->__send_CAN_message((SDO_ASK_COBID + address), 0, 8, Data);

}

void CANOpen::SDO_write_request(uint8_t address , uint16_t object, uint8_t subindex, uint32_t data){

  uint8_t Data [SIZE_OF_CANOPEN_FRAME] = {  SDO_WRITE_REQUEST ,               //Data 0 - SDO Read Request
                                            (uint8_t)(object) ,               //Data 1 - Object Index(LSB)
                                            (uint8_t)(object >> 8) ,          //Data 2 - Object Index(MSB)
                                            subindex,                         //Data 3 - SubIndex
                                            (uint8_t)(data),                  //Data 4 - LSB First Data 
                                            (uint8_t)(data >> 8),             //Data 5
                                            (uint8_t)(data >> 16),            //Data 6
                                            (uint8_t)(data >> 24)             //Data 7
                                          };
                       
  this->__send_CAN_message((SDO_ASK_COBID + address), 0, 8, Data);

}

void CANOpen::SDO_answer_read_request(uint8_t address , uint8_t index_lsb, uint8_t index_msb, uint8_t subindex){

  uint8_t node_id = this->__get_node_id();
  uint32_t value = 0x5555555;
    
  uint8_t Data [8] = { SDO_READ_REQUEST_ANSWER , //Data 0 - SDO type of request
                        index_lsb,           //Data 1 - Object Index(LSB)
                        index_msb,      //Data 2 - Object Index(MSB)
                        subindex,                     //Data 3 - SubIndex
                        (uint8_t)(value),              //Data 4 - LSB First Data 
                        (uint8_t)(value >> 8),         //Data 5
                        (uint8_t)(value >> 16),        //Data 6
                        (uint8_t)(value >> 24)         //Data 7
                      };
  #ifdef __DEBUG__
    Serial.print("Message:");
    Serial.println(value);
  #endif    
  this->__send_CAN_message(SDO_ANSWER_COBID | node_id, 0, 8, Data);

}

int CANOpen::SDO_wait_answer(uint8_t node_id, INT32U *id, INT8U *len, INT8U buf[], uint8_t attemps){

  uint8_t j = 0;
  uint16_t wanted_rx_id = SDO_ANSWER_COBID + node_id;

  for(j = 0; 
    (this->readMsgBuf(id, len, buf) == CAN_NOMSG || *id != wanted_rx_id) 
    && j < attemps; j++){
      delay(2);
  }

  if(j >= attemps){
      return CAN_NOMSG;
  }

  return CAN_OK;

}

/******************************CANOpen PDO methods******************************/

void CANOpen::PDO_transmit(int pdo_address, uint8_t data[8]){
  this->__send_CAN_message(pdo_address, 0, 8, data);
}

//temp func
/******************************CANOpen read methods******************************/

int CANOpen::readMsgBuf(INT32U *id, INT8U *len, INT8U buf[]){
  return this->__CAN0->readMsgBuf(id, len, buf);;
}

/******************************CANOpen pool methods******************************/

void CANOpen::poll_message(){

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[SIZE_OF_CANOPEN_FRAME];

  this->readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
  #ifdef __DEBUG__

    char msgString[128];                        // Array to store serial string
    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
    Serial.println(msgString);

    for(byte i = 0; i<len; i++){
       sprintf(msgString, " 0x%.2X", rxBuf[i]);
      Serial.print(msgString);
     }
    
    Serial.println();

  #endif
    
  const uint8_t __node_id = this->__get_node_id(); //Get the node ID in order to deduce the messages ID that we can receive

  #ifdef __DEBUG__
    Serial.print("Node ID : ");
    Serial.println(this->__get_node_id());
  #endif
  
  //Show PDO message from a node
  if(rxId > 0x180 && rxId < 0x500){
    Serial.print("rinputpdo ");
    Serial.print(rxId);
    Serial.print(" ");
    for(uint8_t i = 0; i < 8; i++){
      Serial.print(rxBuf[i]);
      Serial.print(" ");
    }
    Serial.println("");
  }

  if(rxId == (SDO_ASK_COBID | __node_id)){

    switch(rxBuf[0]){ //Read the first byte of the request in order to know what kind of message it is

      case SDO_READ_REQUEST:
        SDO_answer_read_request(__node_id, rxBuf[1], rxBuf[2], rxBuf[3]); //rxBuf[1] => index lsb | rxBuf[2] => index msb | rxBuf[3] => subindex
      break;
        
      case SDO_WRITE_REQUEST:
        Serial.println("[Error] Write request aborted");
      break;
      
      default:
        Serial.println("[Error] Unknow SDO request");
      break;

    }
  }

  if(rxId == (SYNCH_COBID | __node_id))
  {

    Serial.println("[Error] EMCY signal");
  }

  

  //Display heartbeat
  if(this->display_heartbeats){

    if(rxId > 0x700 && rxId < 0x800){

      Serial.print("heartbeat ");
      Serial.print(rxId ^ 0x700);
      Serial.print(" is ");
      
      switch(rxBuf[0]){

        case 0:
          Serial.println("Boot-Up");
        break;
        
        case 4:
          Serial.println("Stopped");
        break;

        case 5:
          Serial.println("Operational");
        break;

        case 127:
          Serial.println("Pre-Operational");
        break;

        default:
          Serial.println("can't tell");
        break;

      }
      
    }

  }

}

void CANOpen::scan(){

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[SIZE_OF_CANOPEN_FRAME];
  String msg = "";                      // store serial string
  uint32_t identity_object_buf[4];

  for(uint8_t i = 1; i< 128; i++){ //Maybe a non blocking solution is better

    if(i == this->__get_node_id())continue;
    this->SDO_send_read_request(i, DEVICE_TYPE, 0x00);

    if(SDO_wait_answer(i, &rxId, &len, rxBuf, 5)) continue;

    //Print result
    get_identity_object(i, identity_object_buf);
    msg = "found " + this->get_manufacturer_device_name(i) + " id " + (rxId ^ SDO_ANSWER_COBID) + " identity ";
    Serial.print(msg);

    for(uint8_t j = 0; j < 4; j++){
      Serial.print(identity_object_buf[j]); //Print object Identity
      Serial.print((j != 3) ? ";" : "\r\n"); 
    }
    msg = "";

  }

}

/******************************CANOpen specialized methods******************************/


String CANOpen::get_manufacturer_device_name(uint8_t node_id){

  String device_name = "";
  uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  bool altern = true;
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[SIZE_OF_CANOPEN_FRAME];

  this->SDO_send_read_request(node_id, MANUFACTURER_DEVICE_NAME, 0x00);

  if(SDO_wait_answer(node_id, &rxId, &len, rxBuf, 5))return ""; //Wait for the first frame, if it failed return empty string
  
  if(rxBuf[0] == SDO_READ_REQUEST_ANSWER_SEGMENTED){

    uint8_t size_of_text = rxBuf[4], collected_char = 0;
    altern = false;
      
    do{
      data[0] = SDO_ACK + (altern) ? SDO_ACK : SDO_ACK_ALTERNATE;
      this->__send_CAN_message(SDO_ASK_COBID + node_id, 0, 8, data); //Send ack
      if(SDO_wait_answer(node_id, &rxId, &len, rxBuf, 5))return "";
      

      for(byte i = 1; i<len && collected_char < size_of_text; i++){
        device_name += (char)rxBuf[i];
        collected_char++;
      }

      altern = !altern;

    }while(collected_char < size_of_text);

  }

  return device_name;
}

void CANOpen::get_identity_object(uint8_t node_id, uint32_t identity_object[4]){

  //Clear the identity_object array
  memset(identity_object, 0, sizeof(uint32_t) * 4);

  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[SIZE_OF_CANOPEN_FRAME];

  for(uint8_t i = 1; i < 5; i++){ //from 1 to 4
    this->SDO_send_read_request(node_id, IDENTITY_OBJECT, i);
    if(SDO_wait_answer(node_id, &rxId, &len, rxBuf, 5))return; //Wait for the first frame, if it failed  => end of the function
    for(uint8_t j = 4; j < 8; j++)identity_object[i-1] |= (uint32_t)rxBuf[j] << ((j - 4)*8); //Store the identity object 
  }

}