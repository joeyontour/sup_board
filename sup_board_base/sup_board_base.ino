
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xF9, 0xAA, 0x68};


float board_voltage;
float power;
float energy;


int speed;


// Variable to store if sending data was successful
String success;


typedef struct struct_message_remote {
    int speed;
} struct_message_remote;

typedef struct struct_message_base {
    float board_voltage;
    float power;
    float energy;
} struct_message_base;


struct_message_base outgoingReadings;
struct_message_remote incomingReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Received: ");
  Serial.println(incomingReadings.speed);

  speed = incomingReadings.speed;

}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  getReadings();
 
  // Set values to send
  outgoingReadings.board_voltage = board_voltage;
  outgoingReadings.power = power;
  outgoingReadings.energy = energy;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  //updateDisplay();
  delay(1000);
}
void getReadings(){
  board_voltage = 24.5;
  power = 160;
  energy = 10;
}
