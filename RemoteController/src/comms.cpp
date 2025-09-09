#include "comms.h"

Communication::Communication(){
    radio = RF24(9, 10);
}

void Communication::setup_nrf(){

    while (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        delay(1000); 
    }
  
    radio.setPALevel(RF24_PA_MIN, 1);
    radio.setChannel(3);
    radio.setAddressWidth(3);
    radio.setDataRate(RF24_2MBPS);
    radio.enableDynamicAck();
    radio.setAutoAck(1);
    radio.setRetries(0, 4);

    radio.openWritingPipe(CMDADD); 
    radio.openReadingPipe(1, STATADD); 
    radio.startListening();   

    radio.printDetails();
    Serial.println();

}

bool Communication::send_msg(Message& msg)
{

    radio.stopListening(); 
    // radio.flush_tx();
    bool report = radio.write(&msg.data, sizeof(msg.data), 0);
    // delayMicroseconds(180);
    radio.startListening();
    if (msg.message_type() == MessageType::CONTROL) radio.flush_rx();
    return report;
}

bool Communication::receive_msg(Message& msg)
{
    uint8_t pipe;
    if(radio.available(&pipe)) radio.read(&msg.data, sizeof(msg.data));    
    else return 0;

    return 1;
}
