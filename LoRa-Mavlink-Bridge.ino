#include "c_library_v2/common/mavlink.h"
#include "heltec.h"

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6
#define BANDWIDTH 500E3
#define SPREAD 10

unsigned int counter = 0;
String rssi = "RSSI --";
String snr = "SNR --";
String packSize = "--";
String packet ;


void setup() {
  // MAVLink serial port
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->display();

  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setSpreadingFactor(SPREAD);
}

void loop() {
  if (LoRa.parsePacket()) { receive_mavlink_lora(); }
  if (Serial.available()) { receive_mavlink_serial(); }
  counter++;
}

void send_heartbeat() {
   static uint32_t last_time = millis();
   const uint32_t current_time = millis();
   constexpr const uint32_t heartbeat_interval = 1000; // 1 second
   if (current_time - last_time > heartbeat_interval) {
      last_time = current_time; // Update for the next loop
      
      static mavlink_message_t mavlink_message;
      static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
      static uint16_t mavlink_message_length = 0;
      
      if (mavlink_message_length == 0) { // Create message if not
        const int system_id = 1;
        const int component_id = 1;
        const int mavlink_type = MAV_TYPE_GENERIC;
        const int autopilot_type = MAV_AUTOPILOT_INVALID;
        const int system_mode = MAV_MODE_PREFLIGHT;
        const int custom_mode = 0x0000; // No flag
        const int mavlink_state = MAV_STATE_ACTIVE;
        mavlink_msg_heartbeat_pack(
          system_id, component_id, &mavlink_message, mavlink_type, autopilot_type, system_mode, custom_mode, mavlink_state
        );
        mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &mavlink_message);
      }
      Serial.write(mavlink_message_buffer, mavlink_message_length);
   }
}

void send_mavlink_lora(mavlink_message_t message) {
    digitalWrite(LED, HIGH);
    static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    static uint16_t mavlink_message_length = 0;
    mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &message);

    LoRa.beginPacket();
    LoRa.setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
    LoRa.write(mavlink_message_buffer, mavlink_message_length);
    LoRa.endPacket();
    
    Heltec.display->clear();
    Heltec.display->drawString(0 , 0 , "Sent "+ String(mavlink_message_length) + " bytes over LoRa");
    Heltec.display->drawString(0 , 40 , rssi);
    Heltec.display->drawString(60 , 40 , snr);
    Heltec.display->display();
    digitalWrite(LED, LOW);
}

void send_mavlink_serial(mavlink_message_t message) {
    digitalWrite(LED, HIGH);
    static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    static uint16_t mavlink_message_length = 0;
    mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &message);
    Serial.write(mavlink_message_buffer, mavlink_message_length);

    Heltec.display->clear();
    Heltec.display->drawString(0 , 0 , "Sent "+ String(mavlink_message_length) + " bytes over Serial");
    Heltec.display->drawString(0 , 40 , rssi);
    Heltec.display->drawString(60 , 40 , snr);
    Heltec.display->display();
    digitalWrite(LED, LOW);  
}

void receive_mavlink_serial() {
  static mavlink_message_t message;
  static mavlink_status_t status;
  
  // Read all data available from Serial and send over LoRa
  while(Serial.available() > 0) {
    uint8_t serial_byte = Serial.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status)) {
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            send_mavlink_lora(message);
            break;
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
            send_mavlink_lora(message);
            break;
        default:
            send_mavlink_lora(message);
            break;        
      }
    }
  }  
}

void receive_mavlink_lora(){
  static mavlink_message_t message;
  static mavlink_status_t status;
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  snr = "SNR " + String(LoRa.packetSnr(), DEC) ; 
  // Read all data available from Lora and send over Serial
  while(LoRa.available() > 0) {
    uint8_t serial_byte = LoRa.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status)) {
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            send_mavlink_serial(message);
            break;
        case MAVLINK_MSG_ID_HIGH_LATENCY2:
            send_mavlink_serial(message);
            break;
        default:
            send_mavlink_serial(message);
            break;        
      }
    }
  }
}
