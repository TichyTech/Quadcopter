#include "M100-5883.h"

#define LAT_TO_METERS 0.0111319491667f
#define LON_TO_METERS (LAT_TO_METERS * COS_LAT)

GPS::GPS()
{
  fix_type = 0;
  last_fix_timestamp = 0;
}

void GPS::setup_gps(){
  Serial.println("Setting up u-blox GNSS");
  while (myGNSS.begin(Serial2) == false)
  {
    Serial.println(F("u-blox GNSS not detected."));
    #if ENFORCE_GPS == 0
    break; // do not wait for GPS if it is not enforced
    #endif
    delay (1000);
  }
  myGNSS.assumeAutoPVT(true, true);
  // myGNSS.enableDebugging(Serial, false);
  setup_DMA();
  my_update_gps();
}

__attribute__((aligned(256))) uint8_t _rx_buffer[256];
int _dma_chan;
uint16_t rx_user_index_ = 0;
uint16_t rx_dma_index_ = 0;

static void dma_irq_handler() {
  dma_hw->ints0 = 1u << _dma_chan;  // Clear the interrupt request.
  dma_channel_set_trans_count(_dma_chan, 256, true);
}

void setup_DMA(){
  _dma_chan = dma_claim_unused_channel(true);
  /// DMA uart read
  dma_channel_config rx_config = dma_channel_get_default_config(_dma_chan);
  channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
  channel_config_set_read_increment(&rx_config, false);
  channel_config_set_write_increment(&rx_config, true);
  channel_config_set_ring(&rx_config, true, 8);
  channel_config_set_dreq(&rx_config, DREQ_UART1_RX);
  channel_config_set_enable(&rx_config, true);
  dma_channel_configure(_dma_chan, &rx_config, _rx_buffer, &uart1_hw->dr, 256, true);
  dma_channel_set_irq0_enabled(_dma_chan, true);  // Tell the DMA to raise IRQ line 0 when the channel finishes a block

  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_enabled(DMA_IRQ_0, true);
}

uint16_t available(){
  rx_dma_index_ = 256 - dma_channel_hw_addr(_dma_chan)->transfer_count;
  return (rx_user_index_ <= rx_dma_index_) ? (rx_dma_index_ - rx_user_index_) : (256 + rx_dma_index_ - rx_user_index_);
}

byte read() {
  byte data = _rx_buffer[rx_user_index_];
  rx_user_index_ = (++rx_user_index_) & 0xFF;  // move user index
  return data;
}

byte buffer[100];
uint8_t buff_idx = 0;
const byte ubx_header[4] = {0xB5, 0x62, 0x01, 0x07};
bool GPS::my_update_gps(){
  // while(Serial2.available() > 0) {
  while(available() > 0) {
    // buffer[buff_idx] = Serial2.read();
    buffer[buff_idx] = read();  // read from DMA buffer
    if (buff_idx < 4 && buffer[buff_idx] != ubx_header[buff_idx]) {
      buff_idx = 0;  // reset buffer if header is not found
      continue;
    }
    buff_idx++;
    if (buff_idx < 100) continue; 
    buff_idx = 0; // reset on 100

    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for(int i = 2; i < 98; i++){
      CK_A += buffer[i];
      CK_B += CK_A;
    }
    // if (DEBUG){
      // for (int i = 0; i < 100; i++){
      //   Serial.print(buffer[i], HEX);
      //   Serial.print(" ");
      // }
      // if (CK_A == buffer[98] && CK_B == buffer[99]) Serial.println("GPS CHECKSUM MATCH");
      // else Serial.println("GPS checksum failed");
      
    // }
    parse_buffer();
    buff_idx = 0;

    if (fix_type != 3) return false;
    last_fix_timestamp = millis();  // record time of last message from gps
    return true;
  }
  return false;
}

bool GPS::update_gps(){
  if (!myGNSS.getPVT()) return false;

  fix_type = myGNSS.getFixType();
  if (fix_type != 3) return false;
  last_fix_timestamp = millis();  // record time of last message from gps

  //tow = myGNSS.getTimeOfWeek();

  latitude = myGNSS.getLatitude();  // [10^-7 deg]
  longitude = myGNSS.getLongitude();  // [10^-7 deg]
  altitude = myGNSS.getAltitudeMSL();  // [mm]

  n_vel = myGNSS.getNedNorthVel(); // [mm/s]
  e_vel = myGNSS.getNedEastVel(); // [mm/s]
  d_vel = myGNSS.getNedDownVel(); // [mm/s]

  // v_acc = myGNSS.getVerticalAccEst(); // [mm]
  // h_acc = myGNSS.getHorizontalAccEst(); // [mm]

  num_sv = myGNSS.getSIV();

  // pos DOP relies on DOP messages which are sent at 1Hz right now, so we would be waiting
  pos_DOP = myGNSS.getPositionDOP();  // scaled by 0.01

  return true;
}

int32_t extract_int(byte *buffer, int start){
  int32_t value = 0;
  for (int i = 0; i < 4; i++) value |= (buffer[start + i] << (i * 8));
  return value;
}

void GPS::parse_buffer(){

  fix_type = buffer[26];
  num_sv = buffer[29];

  longitude = extract_int(buffer, 30);
  latitude = extract_int(buffer, 34);
  altitude = extract_int(buffer, 38);

  n_vel = extract_int(buffer, 54);
  e_vel = extract_int(buffer, 58);
  d_vel = extract_int(buffer, 62);

}

Vector3 GPS::get_NWU_pos(){
  int32_t lat_delta = latitude - LAT_0;
  int32_t lon_delta = longitude - LON_0;

  float x = LAT_TO_METERS * lat_delta;
  float y = - LON_TO_METERS * lon_delta;
  float z = 0.001f * altitude;

  Vector3 pos = {x, y, z};
  return pos;
}

Vector3 GPS::get_NWU_speed(){
  float x_vel = 0.001f * n_vel;
  float y_vel = -0.001f * e_vel;
  float z_vel = -0.001f * d_vel;

  Vector3 vel = {x_vel, y_vel, z_vel};
  return vel;
}
