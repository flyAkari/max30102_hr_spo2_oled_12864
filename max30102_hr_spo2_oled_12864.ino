/* 基于https://github.com/moononournation/BloodOxygenHeartRateMeter修改 */
/* 作者：flyAkari 会飞的阿卡林 bilibili UID:751219
 * 本代码适用于ESP8266 NodeMCU + 12864显示屏 + MAX30102
 * 7pin SPI引脚，正面看，从左到右依次为GND、VCC、D0、D1、RES、DC、CS
 *    ESP8266 ---  OLED
 *      G     ---  GND
 *      5V    ---  VCC
 *      D5    ---  D0
 *      D7    ---  D1
 *      RST   ---  RES
 *      D3    ---  DC
 *      D8    ---  CS

      ESP8266 --- MAX30102
 *      5V    ---  VIN
 *      G     ---  GND
 *      D0    ---  INT
 *      D1    ---  SCL
 *      D2    ---  SDA
 */
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
// uncomment below line if cannot calculate readings
#define REVERSE_LED

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/D8, /* dc=*/D3, /* reset=*/3);
// Interrupt pin
const byte oxiInt = D0; // pin connected to MAX30102 INT

uint32_t elapsedTime, timeStart;

uint32_t aun_ir, aun_red;
uint32_t aun_ir_buffer[BUFFER_SIZE];  // infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE]; // red LED sensor data
float old_n_spo2;                     // Previous SPO2 value
uint8_t uch_dummy;
bool showMeasuring = false;

void setup()
{
  pinMode(oxiInt, INPUT); // pin D10 connects to the interrupt output pin of the MAX30102
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  delay(2000);
  u8g2.setFont(u8g2_font_unifont_tr);
  u8g2.setCursor(0, 14);
  u8g2.print("Initializing...");
  u8g2.setCursor(0, 28);
  u8g2.print("bilibili");
  u8g2.setCursor(0, 42);
  u8g2.print("FlyAkari");
  u8g2.setCursor(0, 56);
  u8g2.print("uid:751219");
  u8g2.sendBuffer();
  Wire.begin();

  Serial.begin(115200);
  Serial.println("Initializing");

  maxim_max30102_reset(); // resets the MAX30102
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy); // Reads/clears the interrupt status register
  maxim_max30102_init();                                  // initialize the MAX30102
  old_n_spo2 = 0.0;

  Serial.println(F("Time[s]\tSpO2\tHR\tRatio\tCorr"));
  timeStart = millis();
}

void print_hr_spo2(int val_hr, int val_spo2){
  if(val_hr > 999 || val_hr < 0){
    return;
  }
  if(val_spo2 > 100 || val_spo2 < 0){
    return;
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tr); 
  u8g2.setCursor(0,14);
  u8g2.print(" HR");
  u8g2.setCursor(60,14);
  u8g2.print("SPO2");
  
  char str_hr[4];  
  u8g2.setFont(u8g2_font_fur20_tr);
  itoa(val_hr, str_hr, 10);
  if(val_hr > 99){
    u8g2.drawStr(0, 38, str_hr);
  }else if(val_hr > 9){
    u8g2.drawStr(14,38, str_hr);
  }else{
    u8g2.drawStr(28,38, str_hr);
  }
  
  char str_spo2[4];
  u8g2.setFont(u8g2_font_fur30_tr); 
  itoa(val_spo2, str_spo2, 10);  
  if(val_spo2 > 99){
    u8g2.drawStr(55, 48, str_spo2);
  }else if(val_spo2 > 9){
    u8g2.drawStr(70, 48, str_spo2);
  }else{
    u8g2.drawStr(85, 48, str_spo2);    
  }
  
  u8g2.setFont(u8g2_font_fur11_tr); 
  u8g2.setCursor(13,50);
  u8g2.print("bpm");
  u8g2.setCursor(110,64);
  u8g2.print("%");
  u8g2.sendBuffer();   
}

void print_measuring(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tr); 
  u8g2.setCursor(0,14);
  u8g2.print(" HR");
  u8g2.setCursor(60,14);
  u8g2.print("SPO2");
  
  char str_hr[4];  
  u8g2.setFont(u8g2_font_fur20_tr);
  u8g2.drawStr(10, 38, "---");
  
  char str_spo2[4];
  u8g2.setFont(u8g2_font_fur30_tr); 
  u8g2.drawStr(65, 48, "---");

  u8g2.setFont(u8g2_font_unifont_tr);
  u8g2.setCursor(26,62);
  u8g2.print("Measuring...");    
  u8g2.sendBuffer();    
}

void print_press(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tr); 
  u8g2.setCursor(0,14);
  u8g2.print(" HR");
  u8g2.setCursor(60,14);
  u8g2.print("SPO2");
  
  char str_hr[4];  
  u8g2.setFont(u8g2_font_fur20_tr);
  u8g2.drawStr(10, 38, "---");
  
  char str_spo2[4];
  u8g2.setFont(u8g2_font_fur30_tr); 
  u8g2.drawStr(65, 48, "---");

  u8g2.setFont(u8g2_font_unifont_tr);
  u8g2.setCursor(20,62);
  u8g2.print("Keep pressing");    
  u8g2.sendBuffer();    
}

void loop()
{
  // Serial.println("looping");
  float n_spo2, ratio, correl; // SPO2 value
  int8_t ch_spo2_valid;        // indicator to show if the SPO2 calculation is valid
  int32_t n_heartrate;         // heart rate value
  int8_t ch_hr_valid;          // indicator to show if the heart rate calculation is valid
  int32_t i;

  // buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  // read BUFFER_SIZE samples, and determine the signal range
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    while (digitalRead(oxiInt) == 1)
      ; // wait until the interrupt pin asserts
    delay(1);
    // wdt_reset();
#ifdef REVERSE_LED
    maxim_max30102_read_fifo(&aun_ir, &aun_red); // read from MAX30102 FIFO
#else
    maxim_max30102_read_fifo(&aun_red, &aun_ir); // read from MAX30102 FIFO
#endif
    if (aun_ir < 5000)
    {
      break;
    }
    if (i == 0)
    {
      if(showMeasuring){
        print_measuring();
        showMeasuring = false;
      }
      Serial.print("Measuring...");
    }
    *(aun_ir_buffer + i) = aun_ir;
    *(aun_red_buffer + i) = aun_red;
  }

  if (aun_ir < 5000)
  {
    print_press();
    showMeasuring = true;
    Serial.print("Put On Finger");
  }
  else
  {
    // calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heartrate, &ch_hr_valid, &ratio, &correl);
    // Serial.println("rf_heart_rate_and_oxygen_saturation");
    elapsedTime = millis() - timeStart;
    elapsedTime /= 1000; // Time in seconds

    if (ch_hr_valid && ch_spo2_valid)
    {
      Serial.print(elapsedTime);
      Serial.print("\t");
      Serial.print(n_spo2);
      Serial.print("\t");
      Serial.print(n_heartrate, DEC);
      Serial.print("\t");
      Serial.print(ratio);
      Serial.print("\t");
      Serial.print(correl);
      Serial.println("");

      print_hr_spo2(n_heartrate, (int)(n_spo2+0.5));
      old_n_spo2 = n_spo2;
    }
  }
}
