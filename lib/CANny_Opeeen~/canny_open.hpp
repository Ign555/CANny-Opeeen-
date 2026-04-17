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

#include <arduino-timer.h>
#include <mcp_can.h>
#include <SPI.h>

#include "OD_addr.hpp"

#ifndef __CANNY_OPEN_HPP__
#define __CANNY_OPEN_HPP__

//Debug mode
//#define __DEBUG__

/****************************************************************
*
* CANOpen macros
*
****************************************************************/

#define EMPTY_8_BYTES_FRAME 0x0000000000000000
#define EMPTY_4_BYTES_FRAME (uint32_t)0x00000000

//Sync COB-ID
#define SYNCH_COBID 0x080

//SDO COB-ID
#define SDO_ASK_COBID 0x600
#define SDO_ANSWER_COBID 0x580

//SDO Request type 
#define SDO_READ_REQUEST 0x40
#define SDO_READ_REQUEST_ANSWER_SEGMENTED 0x41
#define SDO_READ_REQUEST_ANSWER 0x43
#define SDO_WRITE_REQUEST 0x22

//ACK
#define SDO_ACK 0x60
#define SDO_ACK_ALTERNATE 0x70

//HeartBeat 
#define HEARTBEAT_COBID 0x700
#define DEFAULT_HEARTBEAT_DELAY 10000 //Delay in ms

//NMT modes
#define CANOPEN_NMT_PREOPERATIONAL 0x80
#define CANOPEN_NMT_OPERATIONAL 0x01
#define CANOPEN_NMT_STOP 0x02
#define CANOPEN_NMT_RESET 0x81
#define CANOPEN_NMT_RESETCOMMUNICATION 0x82

//Size of
#define SIZE_OF_CANOPEN_FRAME 8

/****************************************************************
*
* CANOpen class
*
****************************************************************/

class CANOpen{

    public:

        /******************************CANOpen Variable******************************/

        //Configuration flag
        bool display_heartbeats = false;

        /******************************CANOpen constructor & destroyer******************************/

        CANOpen(int baudrate, uint8_t int_pin);
        ~CANOpen(); 

        /******************************CANOpen begin method******************************/
        
        void begin();
        void add_OD(String OD_JSON);

        /******************************CANOpen timer methods******************************/
        
        void tick(); //timer update

        //Sync
        void enable_sync_timer(uint32_t delay);
        void disable_sync_timer();
        static bool sync(void *can_open);

        //Heartbeat
        void enable_heartbeat_timer(uint32_t delay);
        void disable_heartbeat_timer();
        static bool heartbeat(void *can_open);

        /******************************CANOpen NMT methods******************************/

        void NMT_set_mode(uint8_t node_id, uint8_t mode);
        void NMT_node_preop(uint8_t node_id);
        void NMT_node_op(uint8_t node_id);
        void NMT_node_reset(uint8_t node_id);
        void NMT_node_reset_communication(uint8_t node_id);
        void NMT_node_stop(uint8_t node_id);

        /******************************CANOpen SDO methods******************************/

        void SDO_send_read_request(uint8_t address , uint16_t object, uint8_t subindex);
        void SDO_write_request(uint8_t address , uint16_t object, uint8_t subindex, uint8_t data[4]);
        void SDO_write_request(uint8_t address , uint16_t object, uint8_t subindex, uint32_t data);
        void SDO_answer_read_request(uint8_t address , uint8_t index_lsb, uint8_t index_msb, uint8_t subindex);
        
        int SDO_wait_answer(uint8_t node_id, INT32U *id, INT8U *len, INT8U buf[], uint8_t attemps);

        void SDO_transmit(int address, uint8_t data[8]); //Doesn't exist

        /******************************CANOpen PDO methods******************************/

        void PDO_transmit(int address, uint8_t data[8]);

        /******************************CANOpen read methods******************************/

        int readMsgBuf(INT32U *id, INT8U *len, INT8U buf[]);

        /******************************CANOpen pool methods******************************/
        
        void poll_message();
        void scan();
        String get_manufacturer_device_name(uint8_t node_id);
        void get_identity_object(uint8_t node_id, uint32_t identity_object[4]);

    protected:

        uint8_t __can_int_pin;
        int __baudrate;
        MCP_CAN *__CAN0; 
        uint8_t sync_period = 0;

    private:

        //Configuration flag
        bool is_heartbeat_timer_enable = true;
        bool is_sync_timer_enable = false;

        //Timer
        Timer<1> sync_timer;
        Timer<1> heartbeat_timer;

        void __send_CAN_message(INT32U id, INT8U ext, INT8U len, INT8U *buf);
        uint8_t __get_node_id();

};

#endif