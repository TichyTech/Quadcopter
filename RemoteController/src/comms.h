#include <RF24.h>
#include <nRF24L01.h> 
#include "config.h"
#include <SPI.h>
#include <comm_structs.h>

const byte CMDADD[4] = "DRR";  // receive address
const byte STATADD[4] = "DRT";  // transmit address

class Communication{
  private:
    RF24 radio;
  public:
    Communication();
    void setup_nrf();
    bool send_msg(Message& msg);
    bool receive_msg(Message& msg);
};