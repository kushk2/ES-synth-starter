#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>

#include <STM32FreeRTOS.h>

//for generating array
#include <iostream>
#include <cmath>
#include <array>
#include <String>


//Current Step Size & Current Note Name
volatile uint32_t currentStepSize;
const char* currentNote;

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

struct {
  std::bitset<32> inputs; 
  SemaphoreHandle_t mutex; 
} sysState;

//Phase Step Sizes for each of the 12 notes on the keyboard:

const uint32_t stepSizes[numKeys] = {
    138412872, 146730540, 155548208, 164865876, 174583544, 184801212,
    195518880, 206736548, 218454216, 230671884, 243389552, 256607220
};

const char* noteNames[12] = {
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B"
};


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

void scanKeysTask(void *pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS; //initiation interval is 50ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){

    vTaskDelayUntil( &xLastWakeTime, xFrequency ); //waits xfrequency til last loop execution

    int numRowsToRead = 3;
    for(int i = 0; i < numRowsToRead; i++){
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> rowResult = readCols();

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (int j = 0; j < 4; j++) {
          sysState.inputs[4*i + j] = rowResult[j];
      }
      xSemaphoreGive(sysState.mutex);
    }
    
    uint32_t localCurrentStepSize;

    localCurrentStepSize = 0; 
    bool input_i_state;
    for (int i = 0; i < numKeys; ++i) {

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      input_i_state = sysState.inputs[i];
      xSemaphoreGive(sysState.mutex);

      if (!input_i_state) {
        localCurrentStepSize = stepSizes[i];
        currentNote = noteNames[i];
      }
    }

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

  }
}

void displayUpdateTask(void *pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS; //initiation interval is 50ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); //waits xfrequency til last loop execution

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

    u8g2.setCursor(2,20);
    
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.inputs.to_ulong(),HEX);
    xSemaphoreGive(sysState.mutex);

    //scanKeysTask(NULL);
    
    u8g2.setCursor(2,10);
    u8g2.print("Played:"); 
    u8g2.setCursor(45,10);
    u8g2.print(currentNote);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);

  }
}




static uint32_t phaseAcc = 0;

void sampleISR() {
  phaseAcc += __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED); //accumulate phase size
  int32_t Vout = (phaseAcc >> 24) - 128; //set output phase as voltage as per sawtooth, -128 so that the offset=0, for adding other functions etc
  //Serial.println(phaseAcc);
  analogWrite(OUTR_PIN, Vout + 128); //add the offset at the end again
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

  //Initialise Hardware Timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //to initilaise and run the thread  
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,   //Function that implements task
    "scanKeys",     //text name for the task
    64,             //Stack Size in WORDS not bites
    NULL,           //Parameter passed into task
    2,              //Task priority
    &scanKeysHandle     
  );

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,   //Function that implements task
    "displayUpdate",     //text name for the task
    256,             //Stack Size in WORDS not bites
    NULL,           //Parameter passed into task
    1,              //Task priority
    &displayUpdateHandle     
  );

  //initialise semaphore
  sysState.mutex = xSemaphoreCreateMutex();

  //initialise scheduler
  vTaskStartScheduler();

}

void loop() {
  
}