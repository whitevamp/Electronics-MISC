//SCULLCOM HOBBY ELECTRONICS
//DC VOLTAGE CALIBRATOR
//Using TFT Display with Touch Screen
//version 2.0 with both 1.024V and 4.096V references
//14th December 2017

//Version 2.0

#include <Adafruit_GFX.h>       //https://github.com/adafruit/Adafruit-GFX-Library/archive/master.zip
#include <Adafruit_ILI9341.h>   //https://github.com/adafruit/Adafruit_ILI9341/archive/master.zip
#include <URTouch.h>            //http://www.rinkydinkelectronics.com/download.php?f=URTouch.zip
#include <MCP4922.h>            //https://github.com/helgenodland/MCP4922-Arduino-SPI-Library/archive/master.zip
#include <SPI.h>

// Pins for TFT
#define TFT_DC 9                    // Pin connection D/C display (data/command)
#define TFT_CS 10                   // Pin of CS display output connection
#define TFT_RST 8                   // Pin of output connection RESET (If connected to power or button, then comment out this line, and uncomment the next one)
// #define TFT_RST -1               // If the display of the RESET is connected to the power supply or the RESET button on the Arduino
#define TFT_MISO 12                 // Pin of display output connection SDO(MISO)
#define TFT_MOSI 11                 // Pin of display output connection SDI(MOSI)
#define TFT_CLK 13                  // Pin of display output connection SCK

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);  // Create a display object and inform the library of the pinout for working with graphics

// Pins for Touch Screen
#define t_SCK 3                     // Pin of display output connection T_CLK
#define t_CS 4                      // Pin of display output connection T_CS
#define t_MOSI 5                    // Pin of display output connection T_DIN
#define t_MISO 6                    // Pin of display output connection T_DOUT
#define t_IRQ 7                     // Pin of display output connection T_IRQ    

URTouch ts(t_SCK, t_CS, t_MOSI, t_MISO, t_IRQ); // Create an object of the sensor module and inform the library of the pinout for working with it

MCP4922 DAC(11, 13, 2, 14);         // (MOSI,SCK,CS,LDAC) define Connections for UNO_board,
float volts = 0.000;                //set initial voltage
float voltsU = 0.000;

int x, y;                           //TFT screen cooridinates

char myInput[6];                    //store number from keypad  - was [5]
int n = 0;                          //index for myInput array
char key;
int keyDelay = 500;                 //was 1000 (change value as required)

void setup()
{
  
  Serial.begin(9600);               //start serial monitor at 9600 baud - used for testing only
  pinMode(A0, OUTPUT);              //set A0 as a digital output pin
  digitalWrite(A0, LOW);            //set A0 output LOW
  pinMode(A1, OUTPUT);              //set A1 as a digital output pin
  digitalWrite(A1, LOW);            //set A1 output LOW
  pinMode(A2, OUTPUT);              //set A2 as a digital output pin
  digitalWrite(A2, LOW);            //set A2 output LOW
  pinMode(A3, OUTPUT);              //set A3 as a digital output pin
  digitalWrite(A3, LOW);            //set A3 output LOW
  pinMode(A4, OUTPUT);              //set A4 as a digital output pin
  digitalWrite(A4, LOW);            //set A4 output LOW  


  volts = 0;                        //reset DAC output to zero at statup
  dacOutput();                      //set DAC output to volts
  voltsU = 1;                       //sets voltsU to be different to volts so first voltage entry is accepted

  tft.begin();                      //Initialize the start of work with the graphic display
  tft.setRotation(1);               //we translate the display into landscape orientation

  ts.InitTouch();                   //Initialize the touchscreen display module
  ts.setPrecision(PREC_MEDIUM);     //Determine the necessary accuracy of the processing of the pressures: PREC_LOW - low, PREC_MEDIUM - medium, PREC_HI - high, PREC_EXTREME - maximum

  tft.fillScreen(ILI9341_BLACK);    //fill TFT display with black

  setCommonDisplay();               //display common areas for border frame, title and footer.
  mainInputDisplay();               //display common input buttons on main screen
  displayVoltage();                 //display set voltage on main display screen
}

void loop()
{

  while (true)
  {

    readTouchScreen();                //check for input fron touch screen
    {
      if ((y >= 180) && (y <= 200))   //keypad top row coordinates
      {
        if ((x >= 10) && (x <= 90))   //Voltage SET button selected
        {
          tft.fillRect(10, 50, 305, 160, ILI9341_BLACK);  //clear main area of display and make black
          setVoltageKeypad();         //draw keypad entry buttons
          keyPadEntry();              //keypad input entry routine
        }
      }

      if ((y >= 180) && (y <= 200))   //keypad top row coordinates
      {
        if ((x >= 110) && (x <= 180)) //Output ON button selected
        {
          digitalWrite(A2, HIGH);     //switch output voltage ON
        }
      }

      if ((y >= 180) && (y <= 200))   //keypad top row
      {
        if ((x >= 200) && (x <= 280)) //Output OFF button selected
        {
          digitalWrite(A2, LOW);      //switch output voltage OFF
        }
      }

    }

    Serial.print("voltsU = ");
    Serial.println(voltsU);
    Serial.print("volts = ");
    Serial.println(volts);

    if (voltsU != volts) {
      displayVoltage();                           //print reference voltage on TFT main display
    }

    if (voltsU != volts) {
      dacOutput();
    }

    Serial.println(volts, 3);                           //used for testing only
  }
}

void setVoltageKeypad()
{
  tft.fillRect(70, 90, 180, 120, ILI9341_RED);          //fill keypad area with RED

  for (int i = 70; i <= 190; i = i + 60) {              //coordinates for drawing keypad outline
    for (int p = 90; p <= 180; p = p + 30) {            //coordinates for drawing keypad outline
      tft.drawRect(i, p, 60, 30, ILI9341_WHITE);        //draw keypad outline in WHITE
    }
  }

  tft.setTextColor(ILI9341_YELLOW);                     //Determine the color of text for display
  tft.setTextSize(2);                                   //Determine the font size for display
  int t = 1;
  for (int p = 100; p <= 160; p = p + 30) {             //routine to print keypad numbers 1 to 9
    for (int i = 95; i <= 215; i = i + 60) {            //
      tft.setCursor(i, p);
      tft.print(t);
      t = t + 1;
    }
  }

  tft.setCursor(95, 185);               // Determine the coordinates for decimal point on keypad
  tft.print(".");                       // Print decimal point on keypad

  tft.setCursor(155, 190);              // Determine the coordinates for 0 on keypad
  tft.print("0");                       // Print 0 on keypad

  tft.setCursor(205, 190);              // Determine the coordinates for Delete (back space)on keypad
  tft.print("Del");                     // Print Del on keypad


  tft.fillRect(255, 180, 60, 30, ILI9341_RED);    //Draw RED button for SET Voltage on keypad
  tft.drawRect(255, 180, 60, 30, ILI9341_WHITE);  //Draw WHITE outline on SET button on keypad
  tft.setTextColor(ILI9341_YELLOW);               // Determine the color of text for display
  tft.setTextSize(2);                             // Determine the font size for display
  tft.setCursor(270, 190);                        // Determine the coordinates for printing SET on keypad
  tft.print("SET");                               // Print SET on keypad

}

void setCommonDisplay()
{
  tft.drawRect(0, 0, 320, 240, ILI9341_MAGENTA);  //print a magenta border to display
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_GREEN);                //Determine the color of text for display
  tft.setCursor(10, 10);                          //Determine the coordinates of the upper-left corner of the output area
  tft.print("Voltage Reference");                 //Display the text


  tft.setTextColor(ILI9341_WHITE);                //Determine the color of text for display
  tft.setTextSize(2);                             //Determine the font size for display
  tft.setCursor(5, 220);                          //Determine the coordinates of the upper-left corner of the output area
  tft.print("Scullcom Hobby Electronics");        //Display the text
}

void mainInputDisplay()
{
  // make the color selection boxes
  //tft.fillRect(Hpos, Vpos, width, Height, Colour);
  //colours Black, White, Red, Yellow, Green, Cyan, Blue, Magenta
  for (int i = 20; i <= 220; i = i + 100) {
    tft.drawRect(i, 180, 80, 30, ILI9341_RED);        // draw rectangle around box
    tft.fillRect(i, 180, 80, 30, ILI9341_WHITE);      // fill rectangle
  }

  tft.setTextColor(ILI9341_BLUE);     //Determine the color of text for display
  tft.setTextSize(2);                 //Determine the font size for display
  tft.setCursor(30, 187);             //Determine the coordinates of the upper-left corner of the output area
  tft.print("SET V");                 //Display the text
  tft.setCursor(150, 187);            //Determine the coordinates of the upper-left corner of the output area
  tft.print("ON");                    //Display the text
  tft.setCursor(245, 187);            //Determine the coordinates of the upper-left corner of the output area
  tft.print("OFF");                   //Display the text
}

void readTouchScreen()                //touch screen check routine
{
  if (ts.dataAvailable())
  {
    ts.read();
    x = ts.getX();
    y = ts.getY();

    Serial.println("  ");
    Serial.print("X = "); Serial.print(x);
    Serial.print("    Y = "); Serial.print(y);
  }
}

void keyPadEntry()                    //keypad entry routine
{
  while (true)
  {
    readTouchScreen();

    tft.setTextSize(3);               //Determine the font size for display
    tft.setCursor(180, 50);           //Determine the coordinates of the upper-left corner of the output area
    tft.print("Volt");

    if ((y >= 90) && (y <= 110))
    {
      if ((x >= 60) && (x <= 110))
      {
        Serial.println("KEY 1 pressed");
        key = '1';
        addNewNumber();
      }
    }

    if ((y >= 90) && (y <= 110))              //keypad 1st row
    {
      if ((x >= 120) && (x <= 170))           //Keypad 2 return button
      {
        Serial.println("KEY 2 pressed");
        key = '2';
        addNewNumber();
      }
    }

    if ((y >= 90) && (y <= 110))              //keypad 1st row
    {
      if ((x >= 180) && (x <= 230))           //Keypad 3 return button
      {
        Serial.println("KEY 3 pressed");
        key = '3';
        addNewNumber();
      }
    }

    if ((y >= 120) && (y <= 140))             //keypad 2nd row
    {
      if ((x >= 60) && (x <= 110))            //Keypad 4 return button
      {
        Serial.println("KEY 4 pressed");
        key = '4';
        addNewNumber();
      }
    }

    if ((y >= 120) && (y <= 140))              //keypad 2nd row
    {
      if ((x >= 120) && (x <= 170))            //Keypad 5 return button
      {
        Serial.println("KEY 5 pressed");
        key = '5';
        addNewNumber();
      }
    }

    if ((y >= 120) && (y <= 140))               //keypad 2nd row
    {
      if ((x >= 180) && (x <= 230))             //Keypad 6 return button
      {
        Serial.println("KEY 6 pressed");
        key = '6';
        addNewNumber();
      }
    }

    if ((y >= 150) && (y <= 170))               //keypad 3rd row
    {
      if ((x >= 60) && (x <= 110))              //Keypad 7 return button
      {
        Serial.println("KEY 7 pressed");
        key = '7';
        addNewNumber();
      }
    }

    if ((y >= 150) && (y <= 170))               //keypad 3rd row
    {
      if ((x >= 120) && (x <= 170))             //Keypad 8 return button
      {
        Serial.println("KEY 8 pressed");
        key = '8';
        addNewNumber();
      }
    }

    if ((y >= 150) && (y <= 170))               //keypad 3rd row
    {
      if ((x >= 180) && (x <= 230))             //Keypad 9 return button
      {
        Serial.println("KEY 9 pressed");
        key = '9';
        addNewNumber();
      }
    }


    if ((y >= 180) && (y <= 200))               //keypad 4th row
    {
      if ((x >= 60) && (x <= 110))              //Keypad . return button
      {
        Serial.println("KEY . pressed");
        key = '.';
        addNewNumber();
      }
    }

    if ((y >= 180) && (y <= 200))               //keypad 4th row
    {
      if ((x >= 120) && (x <= 170))             //Keypad 0 return button
      {
        Serial.println("KEY 0 pressed");
        key = '0';
        addNewNumber();
      }
    }

    if ((y >= 180) && (y <= 200))               //keypad 4th row
    {
      if ((x >= 180) && (x <= 230))             //Keypad Del return button
      {
        Serial.println("KEY Del pressed");

        key = ' ';
        n = n - 1;
        if (n <= 0) {
          n = 0;
        }
        myInput[n] = key;
        myInput[n + 1] = '\0';

        Serial.print(n);  //test only

        tft.setTextSize(3);                               //Determine the font size for display
        tft.setCursor(70, 50);                            //Determine the coordinates of the upper-left corner of the output area
        tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);  //Determine the color of text and background for display
        tft.print(myInput);                               //Display the text
        delay(keyDelay);
        x = 0;
        y = 0;
      }
    }

    if ((y >= 180) && (y <= 200))                         //
    {
      if ((x >= 240) && (x <= 310))                       //Keypad SET return button
      {
        Serial.println("volt set");
        volts = atof(myInput);                            //convert voltage string to float
        x = 0;
        y = 0;
        tft.fillRect(10, 50, 305, 160, ILI9341_BLACK);
        mainInputDisplay();

        for ( int i = 0; i < sizeof(myInput);  ++i )      //clear myInput array
          myInput[i] = (char)0;                           //clear myInput array

        n = 0;
        return;
      }
    }
  }
}

void dacOutput()                      //set DAC output voltage routine
{

  if (volts >= 8.001) {
    volts = volts / 4;
    digitalWrite(A4, LOW);
    digitalWrite(A1, HIGH);           //set INA105 gain switch to x2
    digitalWrite(A3, HIGH);           //set INA105 gain switch to x2
    SPI.begin();
    DAC.Set((volts * 1000), 0);
    //delay(100);
    SPI.end();
    volts = volts * 4;                //set TFT display reading to input reading

  } else if (volts > 4.000 && volts < 8.001) {
    volts = volts / 2;
    digitalWrite(A4, LOW);
    digitalWrite(A1, LOW);            //set INA105 gain switch to x2
    digitalWrite(A3, HIGH);
    SPI.begin();
    DAC.Set((volts * 1000), 0);
    //delay(100);
    SPI.end();
    volts = volts * 2;                //set TFT display reading to input reading

  } else if (volts > 1.000 && volts < 4.001) {
    digitalWrite(A4, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A3, LOW);
    SPI.begin();
    DAC.Set((volts * 1000), 0);
    //delay(100);
    SPI.end();
  }else{
    
    digitalWrite(A4, HIGH);
    digitalWrite(A1, LOW);
    digitalWrite(A3, LOW);
    SPI.begin();
    DAC.Set((volts * 4000), 0);
    //delay(100);                     //optional delay if needed
    SPI.end();    
  }
 
  voltsU = volts;
}

void displayVoltage()
{
  tft.setTextSize(4);                                 //Determine the font size for display (text size was 4)
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);    //Determine the color of text for display
  tft.setCursor(20, 100);                             //Determine the coordinates of the upper-left corner of the output area
  tft.print("VOLTS ");                                //Display the text
  tft.print(volts, 3);
}

void checkMainSelection()
{
  while (true)
  {
    readTouchScreen();

    if ((y >= 180) && (y <= 255)) //keypad top row
    {
      if ((x >= 10) && (x <= 90)) //SET button
      {
        setVoltageKeypad();   //
        keyPadEntry();
        mainInputDisplay();
      }
    }
  }
}

void addNewNumber()                   //echo keypad entry numbers to display
{
  if (n <= 4)
  {
    myInput[n] = key;
    myInput[n + 1] = '\0';
    n++;
  } else {
    n = 4;

    myInput[n + 1] = '\0';
  }
  tft.setTextSize(3);               // Determine the font size for display
  tft.setCursor(70, 50);            // Determine the coordinates of the upper-left corner of the output area
  tft.print(myInput);               // Display the text
  delay(keyDelay);
  x = 0;
  y = 0;
}

