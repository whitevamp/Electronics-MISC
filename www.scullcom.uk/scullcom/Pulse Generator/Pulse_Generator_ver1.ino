//SCULLCOM HOBBY ELECTRONICS
//Pulse Generator version 1 (PART 1)
//LiquidCrystal_I2C.h and Adafruit_MCP4725.h libraries need to be installed links to library downloads below:
//https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/LiquidCrystal_V1.2.1.zip 
//https://github.com/adafruit/Adafruit_MCP4725/archive/master.zip
//


#include <Wire.h>                                       //I2C communications library using analog pins A4(SDA), A5(SCL)
#include <LiquidCrystal_I2C.h>                          //LCD I2C Library           

#include <Adafruit_MCP4725.h>                           //DAC library

Adafruit_MCP4725 dac;                                   //constructor

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);              // 0x27 is the default I2C bus address of LCD display

const int PulseType = A2;

int voltage = 0;                                        //selector switch voltage output reading
int divider =0;                                         //divider setting for frequency range setting
float mod_voltage = 0;                                  //duty cycle voltage level
float duty = 0;                                         //duty cycle reading
int polarity = 0;                                       //pulse polarity setting

void setup() {
  pinMode (PulseType, INPUT_PULLUP);                    //set PulseType pin as input with internal pullup

  dac.begin(0x61);                                      //the DAC I2C address with MCP4725 pin A0 set high

  lcd.begin(20,4);                                      //initialize the lcd for 20 columns 4 line display
  lcd.setBacklightPin(3,POSITIVE);                      // BL, BL_POL
  lcd.setBacklight(HIGH);

  lcd.home();                                           //set LCD cursor to top left (home)
  lcd.print("      SCULLCOM");  
  lcd.setCursor(0,1);                                   //set LCD cursor to start of second line
  lcd.print (" Hobby Electronics");
  lcd.setCursor(0,2);
  lcd.print("  Pulse Generator");
  lcd.setCursor(0,3);
  lcd.print("     Version 1.0");
  delay(4000);                                          //sets into screen delay
  lcd.clear();                                          //clear LCD display 
  lcd.setCursor(0,0);                                   //set LCD cursor to top left (home)
  lcd.print("Frequency Range");                         //print "Pulse Generator" on top line of LCD
}

void loop() {

  voltage = analogRead(A0);                               //read voltage from selector switch

  lcd.setCursor(0,1);                                     //set LCD cursor to start of second line
  FrequencyRange();                                       //print Frequency Range selected

  polarity = digitalRead(PulseType);                      //read polarity of Pulse Switch
  if(polarity==LOW)                                       //if LOW pulse is set positive
  {                                                       //
   PositivePulse();
    } else if 
    (polarity==HIGH)                                      //if HIGH pulse is set negative
    NegativePulse();

 dac.setVoltage(divider,false);                           //set DAC output voltage for Frequency Range selected
 delay(500);                                              //short delay

 DutyCycle();                                             //print Duty Cycle setting on LCD.
 
}

//************************************************************************************************

void DutyCycle()
{
  mod_voltage = analogRead(A1);                           //read Duty Cycle voltage
  mod_voltage = (mod_voltage * 5) / 1024.0;               //calculate voltage

   if (mod_voltage <0.14)                                 //if voltage is lower than 0.14v
  {
  mod_voltage = 0.14;                                     //set lower limit to be 0.14v
  }
  else if (mod_voltage > 0.86)                            //if voltage is higher than 0.86v
  {
  mod_voltage = 0.86;                                     //set upper limit to 0.86v
  }
  else
  {
  mod_voltage = mod_voltage;                              //if voltage within limits then no change
  }

   duty = ((mod_voltage-0.1)/0.8)*100;                    //calculate Duty Cycle as a percentage
   lcd.setCursor(0,2);                                    //set LCD cursor to start of row 3
   lcd.print("Duty Cycle = ");                            //print "Duty Cycle ="
   if (duty<10)                                           //if duty cycle is below 10
   {
    lcd.print(" ");                                       //clear leading digit with a space
   }
   
   lcd.print(duty,2);                                     //print duty cycle to 2 decimal places
   lcd.print("%");                                        //pring "%" after duty cycle reading
   return;
}

//*******************************************************************************************************

void PositivePulse()
{
if (voltage >100 && voltage <200){
  divider = 391;
  }
  else if (voltage >250 && voltage <350){
  divider = 651;
  }
  else if (voltage >400 && voltage <500){
  divider = 911;
  }
  else if (voltage >550 && voltage <600){
  divider = 1172;
  }
  else if (voltage >700 && voltage <800){
  divider = 1432;
  }
  else if (voltage >850 && voltage <950){
  divider = 1693;
  }
  else if (voltage >1000) {
  divider = 1953;
  }  
  else {
    divider = 0;
  }
return;
}

//********************************************************************************************************

void NegativePulse()
{
if (voltage >100 && voltage <200){
  divider = 391 + 3385;
}
else if (voltage >250 && voltage <350){
  divider = 651 + 2865;
}
  else if (voltage >400 && voltage <500){
  divider = 911 + 2344;
}
else if (voltage >550 && voltage <600){
  divider = 1172 + 1823;
}
  else if (voltage >700 && voltage <800){
  divider = 1432 + 1302;
}
else if (voltage >850 && voltage <950){
  divider = 1693 + 781;
}
else if (voltage >1000){
  divider = 1953 + 260;
} 
  else {
    divider = 0 + 3906;
  }
  return;
}

//********************************************************************************************************

  void FrequencyRange()
  {
  if (voltage >100 && voltage <200){
  lcd.print("15.63KHz to 250.0KHz");
}
else if (voltage >250 && voltage <350){
  lcd.print("3.906KHz to 62.50KHz");
}
  else if (voltage >400 && voltage <500){
  lcd.print("976.60Hz to 15.63KHz");
}
else if (voltage >550 && voltage <600){
  lcd.print("244.10Hz to 3.906KHz");
}
  else if (voltage >700 && voltage <800){
  lcd.print("61.040Hz to 976.60Hz");
}
else if (voltage >850 && voltage <950){
  lcd.print("15.260Hz to 244.10Hz");
}
else if (voltage >1000){
  lcd.print("3.8150Hz to 61.040Hz");
} 
  else {
    lcd.print ("62.50KHz to 1MHz    ");
  }
   
  return;
  }
  
//*******************************************************************************************************
