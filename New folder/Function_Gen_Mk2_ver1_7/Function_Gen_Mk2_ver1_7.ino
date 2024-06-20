/*
 * SCULLCOM HOBBY ELECTRONICS
 * FUNCTION GENERATOR Mk2 PROJECT
 * Software Version 1.7
 * 4th June 2018
 */

//------------------------------------------------------- INCLUDES --------------------------------------------------------- 
#include <SPI.h>                                                                //Serial Peripheral Interface (SPI) library
#include <Wire.h>                                                               //I2C library                
#include <LiquidCrystal_I2C.h>                                                  //F Malpartida's NewLiquidCrystal library
                                    // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip

//------------------------------------------------------ CONSTANTS --------------------------------------------------------- 
const int DAC_CS = 10;                                                          //DAC chip select data pin
const int DAC_GAIN_X1 = 0x01;                                                    //set DAC gain to 1
const int DAC_GAIN_X2 = 0x00;                                                    //set DAC gain to 2

const int SINE_TRIANGLE = 9;
const int SINE_TRIANGLE_GAIN = A0;
const int FREQ_1 = 8;
const int FREQ_2 = 7;
const int FREQ_3 = 6;
const int OUTPUT_TYPE = A1;

const int FREQ_RANGE_SELECT = A3;
const int SINE_TRIANGLE_SQUARE_SELECT = A2;

const byte PIN_A = 2;                                                           //digital pin (also interrupt pin) for the A pin of the Rotary Encoder
const byte PIN_B = 3;                                                           //digital pin for the B pin of the Rotary Encoder

const int SPEED = 4;                                                            //digital pin for the frequency adjustment speed             

//----------------------------------------------------- GLOBAL VARIABLES -------------------------------------------------------- 
/*
 * Set the pins on the I2C chip used for LCD connections
 * ADDR,EN,R/W,RS,D4,D5,D6,D7
 * 0x27 is the default address of the LCD with I2C bus module (this address may vary with some type of modules if 0x27 does not work then try using 0x3F)
 */
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); 

int freq_range_state = 1;                                                         //integer to hold the frequency state selected (initial states selected)
int waveform_state = 1;                                                           //integer to hold the waveform state selected (initial states selected)
int adjustment_state = 1;                                                         //integer to hold the adjustment state selected (initial states selected)

unsigned long lastCount = 50;                                                     //keep track of last rotary value
volatile unsigned long factor = 1;                                                //number of rotary encoder steps to jump (default 1)
volatile unsigned long encoderMax = 4095;                                         //sets maximum Rotary Encoder value allowed
volatile unsigned long encoderMin = 2278;                                            //sets minimum Rotary Encoder value allowed
volatile unsigned long encoderValue = encoderMin;                                 //set encoder default position to the minimum set value - Updated by the ISR (Interrupt Service Routine)

//---------------------------------------------------- INTERRUPT SUB ROUTINE ------------------------------------------------------ 
void rotaty_encoder_interrupt()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  if (interruptTime - lastInterruptTime > 5) {                                    //if interrupts come faster than 5ms, assume it's a bounce and ignore
    if (digitalRead(PIN_B) == LOW){
     encoderValue = encoderValue - factor;
    } else {
      encoderValue = encoderValue + factor;
    }
    encoderValue = min(encoderMax, max(encoderMin, encoderValue));                //sets maximum range of rotary encoder
    
    lastInterruptTime = interruptTime;                                            //keep track of when we were here last (no more than every 5ms)
  }
}

//----------------------------------------------------------- SETUP FUNCTION ----------------------------------------------------- 
void setup(){
  
  Serial.begin(9600);                                                            //initialise the serial port (used for testing only)
 
  lcdSetup();                                                                    //setup the LCD display
  pinSetup();                                                                    //setup the pins used in the project
  
  attachInterrupt(digitalPinToInterrupt(PIN_A), rotaty_encoder_interrupt, LOW);  //attach the interrupt sub routine to pin A
  attachInterrupt(digitalPinToInterrupt(PIN_B), rotaty_encoder_interrupt, LOW);  //attach the interrupt sub routine to pin B
 
  SPI.begin();  //initialise the SPI
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  lcdWelcome(3000);                                                              //display the welcome message (for the time period set in brackets in milli seconds)

  /* 
   *  the default factor is set to 10 where the factor global is defined 
   *  print appropriate setting on LCD
   */
  lcdClearAndPrintLine(10,0,"Control:x1"); 

  setWaveform(HIGH,LOW,LOW);                                                     //set the start-up default waveform setting to Sine Wave
  lcdClearAndPrintLine(1,2,"Sine Wave");                                         //print default waveform type to LCD display
  Serial.println("SINE");                                                        //print default waveform type to serial monitor (used for testing only)

  setFrequencyRange(HIGH,LOW,LOW);                                               //set the start-up default frequency range setting to RANGE 1
  lcdClearAndPrintLine(1,1,"RANGE 1");                                           //print default frequency range setting to LCD display
  Serial.println("FREQ_1");                                                      //print default frequency range setting to serial monitor (used for testing only)
 
}

//-------------------------------------------------- MAIN PROGRAMME LOOP ----------------------------------------------------- 
void loop(){
  waveformTypeSelect();                                                          //select waveform type (Sine-Triangle-Square)
  freqRangeSelect();                                                             //select frequency range
  freqSpeedAdjust();                                                             //select frequency adjustment speed
  
  if (encoderValue != lastCount){                                                //if encoder value changes then print value to LCD
    Serial.println(encoderValue);                                                //used for testing only
    lcd.setCursor(1,0);
    lcd.print("       ");                                                        //
    lcd.setCursor(1,0);
    lcd.print(encoderValue);                                                     //
    lastCount = encoderValue;                                                    //Keep track of this new value
    
    setDacOutput(0, DAC_GAIN_X2, 1, encoderValue);                               //send encoder value to DAC
  }
}

//-------------------------------------------------- PIN SETUP HELPER FUNCTION ---------------------------------------------------

void pinSetup(void){                                                              //setup all the pins for this project
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(SPEED, INPUT);
  pinMode(DAC_CS, OUTPUT);
  pinMode (SINE_TRIANGLE,OUTPUT);
  pinMode (SINE_TRIANGLE_GAIN, OUTPUT);
  pinMode (OUTPUT_TYPE, OUTPUT);
  pinMode (FREQ_1, OUTPUT);
  pinMode (FREQ_2, OUTPUT);
  pinMode (FREQ_3, OUTPUT);
 
  pinMode (FREQ_RANGE_SELECT, INPUT_PULLUP);                                       //frequency range button set as input with internal pullup resistor
 
  pinMode (SINE_TRIANGLE_SQUARE_SELECT, INPUT_PULLUP);                             //waveform type button set as input with internal pullup resistor
  
  digitalWrite(DAC_CS, HIGH);                                                      //by default set the DAC Chip Select pin to HIGH to disable DAC at startup
}

//--------------------------------------------------- LCD SETUP ROUTINE -------------------------------------------------------

void lcdSetup(void){                                                              //function to set up the initial state of the LCD display

  lcd.begin(20, 4);                                                               //set up the LCD's number of columns and rows                          

  lcd.setBacklightPin(3,POSITIVE);                                                //BL, BL_POL         

  lcd.setBacklight(HIGH);                                                         //set LCD backlight to ON                   
}

//--------------------------------------------- INITIAL WELCOME MESSAGE ROUTINE ------------------------------------------------

void lcdWelcome(int display_time){                                                //function to dispaly LCD welcome message on the LCD (time period in brackets)
  lcd.clear();                                                                    //clear LCD display                   
  lcd.setCursor(6,0);                                      
  lcd.print("SCULLCOM");                                  
  lcd.setCursor(1,1);                                                             //
  lcd.print("Hobby Electronics");                          
  lcd.setCursor(1,2);
  lcd.print(" Function Generator");                                               //
  lcd.setCursor(5,3);
  lcd.print("Version 1.7");                                                       //
  delay(display_time);                                                            //time for welcome message to be displayed (in milli-seconds)
  lcd.clear();                                                                    //clear LCD display
}

//------------------------------------------------ SEND DATA TO DAC ROUTINE ----------------------------------------------------- 
void setDacOutput(byte channel, byte gain, byte shutdown, unsigned int value)
{
  byte lowByte = value & 0xff;
  byte highByte = ((value >> 8) & 0xff) | channel << 7 | gain << 5 | shutdown << 4;
  //digitalWrite(DAC_CS, LOW);                                                      //set DAC chip select pin LOW ready for data transfer
  PORTB &= 0xfb;                                                                    //set DAC chip select pin LOW ready for data transfer
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;                                                                     //set DAC chip select pin HIGH to disable data transfer                                                             
  //digitalWrite(DAC_CS, HIGH);                                                     //set DAC chip select pin HIGH to disable data transfer
}

//----------------------------------------------- FREQUENCY ADJUSTMENT SPEED ROUTINE --------------------------------------------
void freqSpeedAdjust(void){
  String display_text;                                                            //define "display_text" as a string
  if(digitalRead(SPEED) == HIGH){                                                 //poll for state of the frequency range button and check if it has been pressed
    
    adjustment_state = stateToggle(adjustment_state, 3);                          //if the button was pressed, toggle the state

//Depending on the current adjustment_state, switch to the correct factor and update the LCD display

    switch (adjustment_state) {
      case 1:
        factor = 1;
        display_text = "   x1";
        break;
      case 2:
        factor = 10;
        display_text = "  x10";
        break;
      case 3:
        factor = 100;
        display_text = " x100";
        break;
    }
    lcdClearAndPrintLine(15,0,display_text);
    
    lcd.setCursor(1,0);
    lcd.print("       ");                                                        //
    lcd.setCursor(1,0);
    lcd.print(encoderValue);        
    
    delay(200);                                                                    //allow time for cmos switching IC to settle to new state
  } 
}

//-------------------------------------------------- FREQUENCY RANGE SELECT ROUTINE -------------------------------------------
void freqRangeSelect(void){
  String display_text;
  if(digitalRead(FREQ_RANGE_SELECT) == 0){                                          //poll for state of the frequency range button and check if it has been pressed  */
    freq_range_state = stateToggle(freq_range_state, 3);                            //if the button was pressed, toggle the state

//Depending on the current freq_range_state, switch to the correct frequncy range and update the LCD display

    switch (freq_range_state) {
      case 1:
        setFrequencyRange(HIGH,LOW,LOW);
        display_text = "RANGE 1";
        break;
      case 2:
        setFrequencyRange(LOW,HIGH,LOW);
        display_text = "RANGE 2";
        break;
      case 3:
        setFrequencyRange(LOW,LOW,HIGH);
        display_text = "RANGE 3";
        break; 
    }
    lcdClearAndPrintLine(0,1,display_text);
    delay(200);                                                               //allow time for cmos switching IC to settle to new state                                   
  }  
}

//-------------------------------------------------- FREQUENCY RANGE SWITCHING STATE ROUTINE -------------------------------------------

/* Set the pins for either Range 1, Range 2 aor Range 3
 *  - RANGE 1: HIGH, LOW, LOW
 *  - RANGE 2: LOW, HIGH, LOW 
 *  - RANGE 3: HIGH, LOW, HIGH
 */
void setFrequencyRange (byte frequency_pin_1_value, byte frequency_pin_2_value, byte frequency_pin_3_value){
  digitalWrite(FREQ_1, frequency_pin_1_value);
  digitalWrite(FREQ_2, frequency_pin_2_value);
  digitalWrite(FREQ_3, frequency_pin_3_value);  
}

//------------------------------------------------------- WAVEFORM TYPE SELECT ROUTINE -------------------------------------------------
void waveformTypeSelect(void){
  String display_text;  
  if(digitalRead(SINE_TRIANGLE_SQUARE_SELECT) == 0){                            //poll for state of the waveform button and check if it has been pressed  
    waveform_state = stateToggle(waveform_state, 3);                            //if the button was pressed, toggle the state

//Depending on the current waveform_state, switch to the correct waveform and update the LCD display

    switch (waveform_state) {
      case 1:
        setWaveform(HIGH,LOW,LOW);
        display_text = "Sine Wave";
        break;
      case 2:
        setWaveform(LOW,HIGH,LOW);
        display_text = "Triangle Wave";
        break;
      case 3:
        setWaveform(HIGH,LOW,HIGH);
        display_text = "Square Wave";
        break;
    } 
    
    lcdClearAndPrintLine(0,2,display_text);                              
    delay(200);                                                                  //allow time for cmos switching IC to settle to new state 
  }
}

//------------------------------------------------------- WAVEFORM SWITCHING STATE ROUTINE -------------------------------------------------

/* Set the pins for either Sine, Triangle or Squar wave function
 *  - SINE: HIGH, LOW, LOW
 *  - TRIANGLE: LOW, HIGH, LOW 
 *  - SQUARE: HIGH, LOW, HIGH
 */
void setWaveform(byte sine_triangle, byte sine_triangle_gain, byte ouput_type){

  digitalWrite(SINE_TRIANGLE,sine_triangle);                                     //Low = triangle   High = sine

  digitalWrite(SINE_TRIANGLE_GAIN, sine_triangle_gain);                          //Low = RV14 (sine)  High = RV11 (triangle)
 
  digitalWrite(OUTPUT_TYPE, ouput_type);                                         //Low = sine or triangle   High = Square
}

//---------------------------------------------------------- LCD CLEAR AND PRINT TEXT FUNCTION ---------------------------------------------------------------
/* Clear a line and print the specified value at the cursor position */
void lcdClearAndPrintLine(int cursor_postion, int line_number, String display_text){
  lcd.setCursor(0,line_number);                                                 //set LCD cursor to correct column and row
  lcd.print("                    ");                                            //clear LCD row prior to new text
  lcd.setCursor(cursor_postion,line_number);                                    //set LCD cursor to correct column and row
  lcd.print(display_text);                                                      //print text to LCD
  Serial.println(display_text);                                                 //for testing only
}

//--------------------------------------------------------- TOGGLE SWITCH FUNCTION ----------------------------------------------------------
/* General function to toggle an integer value between n states */
int stateToggle(int current_value, int max_value){
  int outputValue = current_value;
  
  outputValue++;                                                                //increment the value by 1
  if(outputValue > max_value){                                                  //if the value is greater than the maximum, set it to 1
   outputValue = 1;
  }
  return outputValue; 
}
//------------------------------------------------------------------------------------------------------------------------------------------
