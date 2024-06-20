//SCULLCOM HOBBY ELECTRONICS
//MILLIVOLT METER USING LTC2400 24bit ADC CHIP
//Software version 5.00
//4.096 volt precision reference (ADR4540)
//
// LCD RS pin to digital pin 8
// LCD Enable pin to digital pin 7
// LCD D4 pin to digital pin 6
// LCD D5 pin to digital pin 5
// LCD D6 pin to digital pin 4
// LCD D7 pin to digital pin 3
// LCD R/W pin connect to ground

//LTC2400 SCK to digital pin 13
//LTC2400 SDO to digital pin 12
//LTC2400 CS  to digital pin 10

//Calibration button to digital pin 2

#include <SPI.h>                         //include SPI library (Serial Peripheral Interface)
#include <LiquidCrystal.h>               //include LCD library
#include <EEPROM.h>                      //include EEPROM library

LiquidCrystal lcd(8, 7, 6, 5, 4, 3);     //initialize the library with the numbers of the interface pins

const int LTC_CS = 10;                   //set ADC chip select pin to Arduino pin 10
const int CalButton = 2;                 //set calibration button to use D2
const int DecButton = 9;                 //set decimal places button to D9
long adcread;                            //reading from the ADC (LTC2400)
int CalSetup = 0;                        //calabration check
int DecPlaces = 0;
long cal;                                //calibration adjustment factor
float volt;                              //voltage reading
float v_ref = 4.096;                     //Reference Voltage used
String v;                                //string to select V, mV or uV on display
float trim = 0;                          //calibration error trim factor
const int samples = 4;                   //set number of sample readings to take
int ct = 190;                            //ADC converstion time (plus additional program loop time)
int d = 0;                               //integer that holds number of decimal places displayed on LCD
int dV = 6;                              //set number of displayed decimal places on Volt range
int dmV = 2;                             //set number of displayed decimal places on MilliVolt range
int duV = 0;                             //set number of displayed decimal places on MicroVolt range
int da = 0;                              //decimal places adjustment

long address = 0;                        //set EEPROM memory start address location

//***************************************************************************************************************
void setup() {
 
  pinMode (LTC_CS, OUTPUT);              //set LTC_CS (pin D10 on Arduino Nano) to OUTPUT mode
  digitalWrite(LTC_CS, HIGH);            //set LCT2400 chip select pin HIGH to disable
 
  SPI.begin();                           //initialise SPI bus
  SPI.setBitOrder(MSBFIRST);             //Sets the order of bits shifted out and in to SPI bus, MSBFIRST (most-significant bit first)
  SPI.setDataMode(SPI_MODE0);            //set SPI to Mode 0 (MOSI read on rising edge (CPLI=0) and SCK idle low (CPOL=0))
  SPI.setClockDivider(SPI_CLOCK_DIV16);  //divide Arduino clock by 16 to gave a 1 MHz SPI clock
 
  lcd.begin(16, 2);                      //set up the LCD's number of columns and rows 
  lcd.setCursor(0,0);                    //set LCD cursor to column 0, row O (start of first line)
  lcd.print("    SCULLCOM");             //print SCULLCOM to display with 5 leading spaces (you can change to your own)
  lcd.setCursor(0,1);                    //set LCD cursor to column 0, row 1 (start of second line)
  lcd.print("Hobby Electronic");         //print Hobby Electronics to display (you can change to your own)
  delay(3000);                           //3000 mSec delay for intro display
  lcd.clear();                           //clear dislay
  lcd.setCursor(0,0);                    //set LCD cursor to column 0, row O (start of first line)

  cal = (EEPROMreadlong(address));       //read calibration factor from EEPROM
   
  lcd.print("EEPROM Cal Level");
  lcd.setCursor(0,1);
  lcd.print(cal);                        //Briefly show calibration factor stored in EEPROM at switch on
  delay(3000);
  lcd.clear();
  
  lcd.setCursor(0,0);                    //set LCD cursor to column 0, row O (start of first line)
  lcd.print("Millivolt Meter");          //print Millivolt Meter to display

  for (int i=0;i<5;i++) {                //disregard the first few readings from ADC as they seem unstable?
   Spi_Read();
   delay(ct);
       }
  }

//***************************************************************************************

void loop() {
  
  CalSetup=digitalRead(CalButton);       //check if calibration button pressed
  if(CalSetup==HIGH)                     //if calibration button pressed (taken high to +5v)
  {
    Cal_Adjust();                        //go to Calibration sub-routine
    } else {                             //if calibration button not pressed continue with main loop
  
  DecPlaces=digitalRead(DecButton);
  if(DecPlaces==LOW)
  {
    Dec_Places_Adj();
    delay(500);
    } else {
    lcd.setCursor(15,1);
    lcd.print(d);
      
      
  
    lcd.setCursor(0, 1);                 //set LCD cursor to Column 0 and Row 1 (second row of LCD, first column)
    
    
  int i;
  long sum = 0;
  for (i=0; i<(samples); i++)            //loop to take sample readings
  {
    adcread = Spi_Read();                //read data from LTC2400 ADC
    delay(ct);                           //allow ADC converstion time
    sum += adcread;                      //sum equals running total
    }
  
  sum = sum /samples;                    //calculate average by dividing total readings value by number of samples taken
  sum = sum>>4;                          //shift result 4 places to the right to remove lowest 4 bits (sub LSB's)
  sum = sum - (cal - trim);              //add calibration factor (and trim factor if used)
  
  volt = sum;                            //result changed to a float type and named volt (represents number with fractions)
  volt = volt * 10;                      //muliply voltage reading by x10 to adjust for input resistor divider network
  volt = volt * v_ref / (16777216);      //convert reading in to Volts ready for the display (max scale 24 bit number)

  
   if (volt <0.001)                      //check if voltage reading is below 0.001 volt
  {          
  volt = volt * 1000000;                 //if so multiply reading by 1000000 and display as MicroVolt
  v = "uV";                              //if below 0.001 volt use letters uV on display after voltage reading
  d = duV;                               //set display decimal places 
    
    } else if (volt <1){                 //check if voltage reading is below 1 volt
    volt = volt * 1000;                  //if below 1 volt multiply by 1000 and display as Millivolt
    v = "mV";                            //if below 1 volt use letters mV on display after voltage reading
    d = dmV;                             //set display decimal places  
  
      } else {
      v = "V";                           //if 1 volt or higher use letter V on display after voltage reading
      d = dV;                            //set display characters to show V
    }
  d = d + da;                            //add decimal place display adjustment factor
  lcd.setCursor(0, 1);                   //set the LCD cursor to column 2, row 1 (second line of LCD display 3 positions in from left)
  lcd.print(volt,d);                     //print voltage as floating number to 6 decimal places
  lcd.print(" ");                        //add one blank space after voltage reading
  lcd.print(v);                          //print either uV, mV or V to LCD display (dependant on reading)
  lcd.print("    ");                     //add 4 blanking spaces to remove unwanted characters
     }
  }
}
 //***********************************************************************************************************************
long Spi_Read(void){                    //SPI(Serial Peripheral Interface) read sub-routine to read data form the LTC2400 ADC
                                        //and transfer 8 bits (1 byte) at a time - total of 4 bytes.
  long result = 0;                      //result represents rolling total of the bytes transferred
  long b;                               //b is result of reading ADC output bytes
  digitalWrite(LTC_CS, LOW);            //LTC2400 chip select pin taken low to allow data transfer from Adc
  delayMicroseconds(1);                 //timing delay but may not be required
  
  if (!(PINB & (1 << 4))) {             //check to see if ADC is ready
  
  
  b = SPI.transfer(0xff);               //transfer first byte most significant bits first.
  b &=0x0f;                             //discard first 4 status bits (bits 31 to 25) mask received data with binary 00001111
  result = b;                           //result after removing first 4 bits (replacing them with 0's)
  result <<= 8;                         //shift first byte left by 8 places
  b = SPI.transfer(0xff);               //transfer second byte most significant bits first.
  result |=b;                           //add second byte to first byte by using the OR function (now 12 bits)
  result = result<<8;                   //shift result left by 8 places
  b = SPI.transfer(0xff);               //transfer third byte most significant bits first.
  result |=b;                           //add third byte to result by using the OR function (now 20 bits)
  result = result<<8;                   //shift result left by 8 places
  b = SPI.transfer(0xff);               //transfer fourth byte most significant bits first.
  result |=b;                           //add fourth byte to result by using the OR function (now 28 bits)
  
  digitalWrite(LTC_CS, HIGH);           //LTC2400 chip select pin taken high disables ADC output.
  
  return(result);                       //return with result as the 28 bit data representing the voltage (including 4 sub LSB's)
  }
}
//****************************************************************************************************
long Cal_Adjust(void) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibration");
  lcd.setCursor(0,1);
  lcd.print("Short input lead");
  delay(2000);
    
  int i;
  long sum = 0;
  for (i=0; i<(samples); i++)           //loop to take sample readings
      {
    adcread = Spi_Read();               //read data from LTC2400 ADC
    delay(ct);
    sum += adcread; 
    }
  
  sum = sum /samples;
  sum = sum>>4;                          //shift result 4 places to the right to remove lowest 4 bits
  cal = sum;
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("Adjust Factor");
  lcd.setCursor(0,1);
  lcd.print(cal);                        
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Millivolt Meter");
  
  EEPROMwritelong(address,cal);          //store calibration factor in EEPROM
  }
 
//**************************************************************************************************************************
//Number of decimal places subroutine   
   int Dec_Places_Adj(void) {
     da = da + 1;          //increment display decimal places by one
     if (d > 5) {          //reset to zero if decimal places greater than 6 on display
     da = 0;
    
     duV = 0;
     dmV = 0;
     dV = 0;
    
    lcd.setCursor(6,1);
    lcd.print("        ");  //8 spaces to clean up display
   }
   
   }
//*************************************************************************************************************************
//Routine to write a 4 byte (32 bit) long to EEPROM at specified addresses
void EEPROMwritelong(int address, long value)    
    {
    byte four = (value & 0xFF);            //four = least significant byte
    byte three = ((value>>8) & 0xFF);
    byte two = ((value>>16) & 0xFF);
    byte one = ((value>>24) & 0xFF);       //one = most significant byte
    
    EEPROM.write(address, four);           //write the four bytes into EEPROM
    EEPROM.write(address +1, three);
    EEPROM.write(address +2, two);
    EEPROM.write(address +3, one);
    }

//routine to read back 4 bytes and return with (32 bit) long as value
long EEPROMreadlong(long address)
    {
      long four = EEPROM.read(address);    //read the 4 bytes from EEPROM
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);
      return (four)+(three << 8)+(two << 16)+(one << 24);
    }
  //**************************************************************************************************
