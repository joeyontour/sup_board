#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
#include "NotoSansBold36.h"

#include <esp_now.h>
#include <WiFi.h>




#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23

#define TFT_BL 4 // 
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35//25
#define BUTTON_2 0//26
#define BUTTON_3 27

#define AA_FONT_LARGE NotoSansBold36


TFT_eSPI tft = TFT_eSPI(135, 240);
Button2 btn3(BUTTON_3);

char buff[512];
int vref = 1100;

float voltage = -1;
float voltage_last = -1;
float board_voltage_last = -1;
float power_last = -1;
float energy_last = -1;
int speed_last = -1;


uint8_t broadcastAddress[] = {0x24, 0x62, 0xAB, 0xF9, 0x99, 0xBC};

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

struct_message_remote outgoingReadings;
struct_message_base incomingReadings;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//  Serial.print("\r\nLast Packet Send Status:\t");
 // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  Serial.print(incomingReadings.board_voltage);
  Serial.print(",");
  Serial.print(incomingReadings.power);
  Serial.print(",");
  Serial.println(incomingReadings.energy);

  board_voltage = incomingReadings.board_voltage;
  power = incomingReadings.power;
  energy = incomingReadings.energy;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  tft_setup();
  
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
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 100) {
      timeStamp = millis();
      int button_state_1 = digitalRead(BUTTON_1);
      int button_state_2 = digitalRead(BUTTON_2);
  
      if (button_state_1 == LOW) {
        change_speed(1);
      } else if (button_state_2 == LOW) {
         change_speed(-1);
      }
  
    // Set values to send
    outgoingReadings.speed = speed;
  
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
     
    if (result != ESP_OK) {
      Serial.println("Error sending the data");
    }
    display_data();
    
    }
}












void display_data()
{

uint16_t v = analogRead(ADC_PIN);
voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);

tft.setTextDatum(TR_DATUM);


if (voltage != voltage_last) {
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.drawString(String(voltage_last, 1),tft.width(),0);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(voltage, 1),tft.width(),0);
  voltage_last = voltage;
}

if (board_voltage != board_voltage_last) {
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.drawString(String(board_voltage_last, 1),tft.width(),50);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(board_voltage, 1),tft.width(),50);
  board_voltage_last = board_voltage;
}

if (power != power_last) {
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.drawString(String(power_last, 0),tft.width(),100);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(power, 0),tft.width(),100);
  power_last = power;
}

if (energy != energy_last) {
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.drawString(String(energy_last, 0),tft.width(),150);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(energy, 0),tft.width(),150);
  energy_last = energy;
}

if (speed != speed_last){
  tft.setTextColor(TFT_BLACK, TFT_BLACK);
  tft.drawString(String(speed_last),tft.width(),200);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(speed),tft.width(),200);
  speed_last = speed;
}

}



void button_init()
{

btn3.setPressedHandler([](Button2 & b) {
  change_speed(0);
});
  /*
btn3.setLongClickHandler([](Button2 & b) {
  
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_27, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_deep_sleep_start();
});
*/
}

void button_loop()
{
btn3.loop();
}

void tft_setup() {
  tft.init();
  
  tft.loadFont(AA_FONT_LARGE);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(2);
  tft.setTextSize(4);
  
  tft.setTextDatum(TL_DATUM);
  
  tft.drawString("V", 0,0);
  tft.drawString("V", 0,50);
  tft.drawString("W", 0,100);
  tft.drawString("Wh", 0,150);
  tft.drawString("S", 0,200);
  
  
  if (TFT_BL > 0) { // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h

  button_init();

  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    Serial.println("Default Vref: 1100mV");
  }
}
}

void change_speed(int dir) {
  if (dir == 0) {
    speed = 0;
  } else {
    speed += dir * 2;
  }
  if (speed < -100) {
    speed = -100;
  } else if (speed > 100) {
    speed = 100;
  }

}
