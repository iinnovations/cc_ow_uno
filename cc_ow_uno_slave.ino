/*
*    Example-Code that emulates a CCOWUNO 1024 bits EEPROM
*
*   Tested with
*    - DS9490R-Master, atmega328@16MHz and teensy3.2@96MHz as Slave
*    - DS9490R master, ESP32 onewire library, LoBo port machine.ow library, Arduino 328P @ 8Mhz as slave (CC)
*    - tested on buspirate and two different real 1-wire masters (DS9490 and a PIC18-Device)
*/

#include "OneWireHub.h"
#include "CCOWUNO.h"
#include <Flash.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SPIFlash.h>


#define FLASH_SS      8 // and FLASH SS on D8
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

constexpr uint8_t pin_onewire   { 2 };

byte deviceROM[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

// Modes : 
//  0 : GPIO digital input
//  1 : GPIO digital output
//  2 : GPIO analog input

#define INIT 0
#define USEFLASH 1

/* 
 *  Config types:
 *  0 : input module
 *  1 : output module, all default states are 1
 *  
 */
byte config_type = 1;

uint8_t ioconfig[13] = {0,0,0,0,0, 2, 2, 2, 2, 2, 2, 2, 2 };
byte iovalues[13];

byte iopins[13] =   {3,4,5,6,7, A0,A1,A2,A3,A4,A5,A6,A7 };


byte devicetype = 0x01;
auto ccowuno = CCOWUNO(CCOWUNO::family_code, config_type, 0x02, 0x00, 0x00, 0x00, 0x01);

auto hub = OneWireHub(pin_onewire);

long last_read = 0;
long read_freq_ms = 2000;
long now=0;

void setup()
{
    Serial.begin(115200);
    Serial.println("OneWire-Hub CCOWUNO");

    
    if (config_type == 0){
      uint8_t this_ioconfig[] = {0,0,0,0,0, 2, 2, 2, 2, 2, 2, 2, 2 };
      for (int i=0;i<13;i++){
        ioconfig[i] = this_ioconfig[i];
      }
    }
    else if (config_type == 1){
      Serial.println("Initializing type 1");
      uint8_t this_ioconfig[] = {1,1,1,1,1, 1, 1, 1, 1, 1, 1, 2, 2 };
      for (int i=0;i<13;i++){
        ioconfig[i] = this_ioconfig[i];
        Serial.println(ioconfig[i]);
      }

      // This is not used for analog values. Only for digital get/set
      uint8_t this_iovalues[13] = {1,1,1,1,1, 1,1,1,1,1,1,0,0 };
      for (int i=0;i<13;i++){
        iovalues[i] = this_iovalues[i];
      }
    }
    
    if (USEFLASH){
      Serial.println("Initializing Flash");
      if (flash.initialize())
      {
        Serial.println(F("SPI Flash Init OK"));
    //    Serial.print(F("UniqueID (MAC): "));
    //    flash.readUniqueId();
    //    for (byte i=0;i<8;i++)
    //    {
    //      Serial.print(flash.UNIQUEID[i], HEX);
    //      Serial.print(' ');
    //    }
    //    Serial.println();
      }
      else {
        Serial.println(F("SPI Flash Init FAIL"));
      }
      if (INIT){
        Serial.println(F("Initializing"));
        
        storeparamsintoflash();
      }
      loadparamsfromflash();
    }
    Serial.println(F("Loading parameters into memory"));
    loadparamsintomemory();

    // Setup OneWire
//    ccowuno = CCOWUNO(CCOWUNO::family_code, deviceROM[0], deviceROM[1], deviceROM[2], deviceROM[3], deviceROM[4], deviceROM[5]);
//    Serial.println(ccowuno.ID);
    Serial.println("Attaching");
    hub.attach(ccowuno);

    Serial.println("Starting device with config:");
//    printparams()

    // Test-Cases: the following code is just to show basic functions, can be removed any time
//    Serial.println("Test Write Text Data to page 0");
//    constexpr char memory[] = "abcdefg-test-data full ASCII:-?+";
//    ccowuno.writeMemory(reinterpret_cast<const uint8_t *>(memory),sizeof(memory),0x00);
//
//    Serial.println("Test Write binary Data to page 1");
//    constexpr uint8_t mem_dummy[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
//    ccowuno.writeMemory(mem_dummy, sizeof(mem_dummy), 1*32);

//    Serial.print("Test Read binary Data to page 1: 0x");
//    uint8_t mem_read[16];
//    ccowuno.readMemory(mem_read, 16, 31); // begin one byte earlier than page 1
//    Serial.println(mem_read[3],HEX); // should read 
//
//    Serial.println("Test Set Page Protection for p1");
//    Serial.println(ccowuno.getPageProtection(1*32));     // ZERO
//    ccowuno.setPageProtection(1*32);
//    Serial.println(ccowuno.getPageProtection(1*32 - 1)); // ZERO, out of bounds
//    Serial.println(ccowuno.getPageProtection(1*32));     // ONE
//    Serial.println(ccowuno.getPageProtection(1*32 + 8)); // ONE
//    Serial.println(ccowuno.getPageProtection(1*32 +16)); // ONE
//    Serial.println(ccowuno.getPageProtection(2*32));     // ZERO, out of bounds
//
//    Serial.println("Test Set EPROM Mode for p2");
//    constexpr uint8_t mem_FF[] = { 0xFF, 0xFF };
//    ccowuno.writeMemory(reinterpret_cast<const uint8_t *>(mem_FF),sizeof(mem_FF),2*32);
//    Serial.println(ccowuno.getPageEpromMode(2*32));      // ZERO
//    ccowuno.setPageEpromMode(2*32);
//    Serial.println(ccowuno.getPageEpromMode(2*32 - 1));  // ZERO, out of bounds
//    Serial.println(ccowuno.getPageEpromMode(2*32));      // ONE
//    Serial.println(ccowuno.getPageEpromMode(2*32 + 8));  // ONE
//    Serial.println(ccowuno.getPageEpromMode(2*32 +16));  // ONE
//    Serial.println(ccowuno.getPageEpromMode(3*32));      // ZERO, out of bounds

    // ccowuno.clearMemory(); // begin fresh after doing some work

    Serial.println("config done");
}

void loop()
{
    // following function must be called periodically
    hub.poll();
    
    // Two bytes per analog input. Last byte is digital data. Total of 18 bytes with data CRC at end

    now = millis();
    if (millis()-last_read > read_freq_ms){

      Serial.println("updating config from memory");
      updateconfigfrommemory();
      
      Serial.println("reading");
      
      last_read = now;

      //                         6     6     7     7    8      8     9     9    10    10    11    11    12    12    13    13   DIO   CRC     0     1     2     3     4
      uint8_t mem_dummy[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
      
      byte digital_value = 0;
      for (int i=0;i<5;i++){
        if (ioconfig[i] == 0){
          pinMode(iopins[i], INPUT);      // sets the digital pin as input
          byte iovalue = digitalRead(iopins[i]);
          iovalues[i] = iovalue;
          mem_dummy[i+18]=iovalue;
          
          digital_value += iovalue << i;
          Serial.print(i);Serial.print(" : ");Serial.println(iovalue);
        }
        else if (ioconfig[i] == 1){
          Serial.print("OUTPUT MODE FOR PIN ");Serial.print(iopins[i]);Serial.print(" ");Serial.println(iovalues[i]);
          pinMode(iopins[i], OUTPUT);      // sets the digital pin as output

          byte iovalue = iovalues[i];
          if (iovalue == 1){
            Serial.println("SETTING HIGH");
            digitalWrite(iopins[i], HIGH);
          }
          else {
            Serial.println("SETTING LOW");
            digitalWrite(iopins[i], LOW);
          }
          mem_dummy[i+18]=iovalue;
          
          digital_value += iovalue << i;
          Serial.print(i);Serial.print(" : ");Serial.println(iovalue);
        }
      }
      Serial.print("digital value ");Serial.println(digital_value);
      mem_dummy[16] = digital_value;
     
      for (int i=0;i<8;i++){
        byte this_io_index = i+5;
        if (ioconfig[this_io_index] == 2){
          pinMode(iopins[this_io_index], INPUT);      // sets the digital pin 7 as input
          word analog_value = analogRead(iopins[this_io_index]);

          iovalues[this_io_index]=0xFF;
          
          Serial.print("analog value ");Serial.print(i);Serial.print(",");Serial.print(iopins[i+5]);Serial.print(" : ");Serial.println(analog_value);
          mem_dummy[i*2] = (byte)(analog_value >> 8); // MSB
          mem_dummy[i*2+1] = (byte)(analog_value);      // LSB
  
          Serial.print(mem_dummy[i*2]);Serial.print(","),Serial.println(mem_dummy[i*2+1]);
        }
      }
      mem_dummy[17] = ccowuno.crc8(mem_dummy, 17);

//      for (int i=0;i<17;i++){
//        Serial.print(mem_dummy[i]);Serial.print(" ");
//      }
//      Serial.println("");
      Serial.print("CRC ");Serial.println(mem_dummy[17]);
      
      ccowuno.writeMemory(mem_dummy, sizeof(mem_dummy), 0x00);
    }
} 
/* Internal memory

Page 1

0-15 : Inputs 7-13, MSB, LSB
16   : Digital data
17   : IO CRC


// on-board Flash Memory : 
// 0-12  : ioconfig[0] --> ioconfig[12]
// 13-30 : iovalues[0] --> iovalues[12] 

*/

void storeparamsintoflash() {
    Serial.println(F("Storing parameters"));
//    for (int i=0;i<6;i++){
//      EEPROM.write(i,deviceROM[i]);
//    }
//    byte offset = 6
    for (int i=0;i<13;i++){
      EEPROM.write(i,ioconfig[i]);

      // We store these because if io mode is set to output, we need to retain the output value
      EEPROM.write(i+13, iovalues[i]);
    }
}

void loadparamsfromflash() {
    Serial.println(F("Getting parameters from flash"));
    
    for (int i=0; i<13;i++){
      
      ioconfig[i] = EEPROM.read(i);
      Serial.print(i);Serial.print(":");Serial.println(ioconfig[i]);
      // We just grab values here for the case where we are in output mode
      iovalues[i] = EEPROM.read(i+13);
      
    }
}

void loadparamsintomemory(){
    Serial.println(F("Putting parameters into device memory"));
    
    uint8_t mem_dummy[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t mem_dummy2[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    Serial.println("ioconfig:");
    for (int i=0; i<13;i++){

      Serial.println(ioconfig[i]);
      mem_dummy[i] = ioconfig[i];

      // Only set the set value to a value if it is in GPIO OUT mode. Otherwise leave as zero
      if (ioconfig[i] == 1){
        mem_dummy2[i] = iovalues[i];
        Serial.print("Set to value ");Serial.println(mem_dummy2[i]);
      }
    }
   
    // load parameters of ioconfig into memory, starting at page 1 (second page), which starts at memory address 0x20 (32nd byte)
    ccowuno.writeMemory(mem_dummy, sizeof(mem_dummy), 0x20);
    // Put into set value registers
    ccowuno.writeMemory(mem_dummy2, sizeof(mem_dummy2), 0x40);
}

void updateconfigfrommemory(){
  // This function updates config from memory, if necessary. This is for when config or values are written externally
  // and need to be updated in local variables
  // We could just not use intermediate variables at all. 

  uint8_t mem_read[13];
  ccowuno.readMemory(mem_read, 13, 0x20);

  uint8_t mem_read2[13];
  ccowuno.readMemory(mem_read2, 13, 0x40);
  
  for (int i=0;i<13;i++){
    ioconfig[i] = mem_read[i];
    if (ioconfig[i] == 1){
      Serial.print(i);Serial.print(" configured as output. loading from stored value: ");Serial.println(mem_read2[i]);
      iovalues[i] = mem_read2[i];
    }
  }
}
