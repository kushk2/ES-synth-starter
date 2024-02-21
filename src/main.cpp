#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>

//for generating array
#include <iostream>
#include <cmath>
#include <array>

volatile uint32_t currentStepSize;

//Constants
  const uint32_t interval = 100; //Display update interval
  
//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  //Number of keys you want to define:
  const size_t numKeys = 12;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Phase Step Sizes for each of the 12 notes on the keyboard:


std::array<uint32_t, numKeys> stepSizes() {
    std::array<uint32_t, numKeys> result;

    const uint32_t A4_frequency = 440.0; // Reference frequency for A4
    const double twelfth_root_of_2 = pow(2.0, 1.0 / 12.0); // 12th root of 2

    // Calculate and store phase step sizes for the twelve keys from middle C and up
    for (int i = 0; i < numKeys; ++i) {
        uint32_t frequency = A4_frequency * pow(twelfth_root_of_2, i);
        uint32_t sampling_frequency = 44100.0; // Replace with your actual sampling frequency

        result[i] = static_cast<uint32_t>(pow(2.0, 32) * frequency / sampling_frequency);
    }

    return result;
}

const std::array<const char*, numKeys> noteNames = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

std::string keyIToStr(uint32_t num){
  return noteNames[num];
}


//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols(){

  std::bitset<4> result;

  const int Ci_pin[] = {C0_PIN, C1_PIN, C2_PIN, C3_PIN};

    for(int i = 0; i < size(result); i++){

      result[i] = digitalRead(Ci_pin[i]);
    }
  
  return result;
}

void setRow(uint8_t rowIdx) {
  int RAi_PIN[] = {RA0_PIN, RA1_PIN, RA2_PIN};

  std::bitset<3> rowSelect(rowIdx);

  digitalWrite(REN_PIN, LOW);

  for (int i = 0; i < rowSelect.size(); i++) {
    digitalWrite(RAi_PIN[i], rowSelect[i] ? HIGH : LOW); // Convert to 0 or 1
  }

  digitalWrite(REN_PIN, HIGH);
}


void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  std::array<uint32_t, numKeys> StepSizesArray = stepSizes();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next);  //Wait for next interval

  next += interval;

  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
  u8g2.setCursor(2,20);
  
  std::bitset<32> inputs;

  int numRowsToRead = 3;
  for(int i = 0; i < numRowsToRead; i++){
    setRow(i);
    delayMicroseconds(3);
    std::bitset<4> rowResult = readCols();

    for (int j = 0; j < 4; j++) {
        inputs[4*i + j] = rowResult[j];
    }
  }

  u8g2.setCursor(2,20);
  u8g2.print(inputs.to_ulong(),HEX); 

  u8g2.sendBuffer();          // transfer internal memory to the display

  //Toggle LED
  digitalToggle(LED_BUILTIN);
  
}