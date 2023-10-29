

/*******************************************************************************
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTY AND SUPPORT
 * IS APPLICABLE TO THIS SOFTWARE IN ANY FORM. CYTRON TECHNOLOGIES SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Descrioption:
 * 
 * DC Motor speed controller useing RC (Radio Control) transmitter/receiver 
 * allowing fine contol forward and backwards.
 * 
 * Utilises a Transmitter switch to Arm or Disarm (stop) the motor.
 * To Arm (allow motor to run) the Arm switch must be set and speed zero. This is
 * a saftey feature to prevent the motor spiining up unintentially.
 * 
 * Setting the saftey switch while the motor is running will force motor to stop.
 * 
 * Uses:
 *  LCD 1602
 *  MDC30 R2: Motor Speed Controller
 *  AndyMark, am-0255 DC Motor 
 * 
 *******************************************************************************/
//***********************
// Application headers
//**********************
#include<Arduino.h>
#include <Ticker.h>
#include "CytronMotorDriver.h"
#include <LiquidCrystal.h>

//***********************
// Application functions
//**********************
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);
bool redSwitch(byte channelInput, bool defaultValue);
void setPWROff();
int buttonValue(int);
int autoPWRoff();

// Radio control channels (customise to suit RC being used)
 #define CH1 25
 #define CH5 34

 // LCD 
 #define LCD_EN 4                     // A5
 #define LCD_RS 21
 #define LCD_D4 27
 #define LCD_D5 33
 #define LCD_D6 15
 #define LCD_D7 32

 // power control
 #define PWR 14
 #define PWROFF 39                    // A3
 #define DEBOUNCE_TIME    50          // the debounce time in millisecond.
 #define INACTIVITY_TIME  300000      // After 5 Minutes of inactivity, power off
 int lastButtonState = LOW;           // the previous flickerable state from the input pin
 int buttonState;                     // last reading state from the input pin
 
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled

 unsigned long lastActivityTime = 0;


int ch1Value;
bool ch5Value;
bool bArm = false;
bool bActivity = true; //If false it means that system is in use, if false switch off after x minuites
bool bCheckPWRButton = false;

// Configure the motor driver.
// CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.

CytronMD motor(PWM_DIR, 26, 12);  // PWM = Pin A0, DIR = Pin x.
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() 
{
  motor.setSpeed(0);    // Stop.
  
  // Power control 
  pinMode( PWR, OUTPUT);
  digitalWrite(PWR, true);  // switch on relay

  Serial.begin(115200);
  while (!Serial)
        delay(1000);
  Serial.println("Starting motor Controller");

  // Motor controller pins
  pinMode(CH1, INPUT);  // speed
  pinMode(CH5, INPUT);  // switch
  
  // LCD pins
  pinMode( LCD_EN, OUTPUT);
  pinMode( LCD_RS, OUTPUT);
  pinMode( LCD_D4, OUTPUT);
  pinMode( LCD_D5, OUTPUT);
  pinMode( LCD_D6, OUTPUT);
  pinMode( LCD_D7, OUTPUT);

  //PWR Control
  pinMode(PWROFF, INPUT);  // If high power off. Note that its high initially during power ON

  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("    ObiChase    ");
}

// The loop routine runs over and over again forever.
void loop() 
{

  // A single button to control On and Off
  if (buttonValue(PWROFF) == 1)  // Finger if off
  {
    bCheckPWRButton = true; 
    //lcd.setCursor(0,0);
    //lcd.print("1");
    //Serial.println("Button value: finger off");
  }
  if (bCheckPWRButton == true)
  {
    if (buttonValue(PWROFF) == 0)
    {
      //lcd.setCursor(0,0);
      //lcd.print("0");
      //Serial.println("Button value: finger on");
      setPWROff();
    }
  }
  
if (autoPWRoff() == true)
{
  setPWROff();
}

  //
  // motor control processing
  //
  if (bArm == false)
  {
    // do nothing until speed == 0)
    while (ch1Value != 0  )
    {
      //lcd.display();
      ch1Value = readChannel(CH1, -250, 250, 0);
      lcd.setCursor(0,1);
      lcd.print("Set speed to 0  ");
      //Serial.println("Set speed to 0");
      delay(1000); // FIXTHIS - why is there a delay?
    }
  }
  // check if motor armed - Saftey switch  
  ch5Value = redSwitch(CH5, false);
  if ( ch5Value == 0)
  {
    bArm = true;   // motor demand from rc is 0
    bActivity = true;
    
    lcd.setCursor(0,1);
    lcd.print("Saftey off      ");
    //Serial.print("Saftey off");
  }
  else
  {
    motor.setSpeed(0);    // Stop.
    bArm = false;
    bActivity = false;

    lcd.setCursor(0,1);
    lcd.print("Saftey on : Stop");
    //Serial.println("Saftey on : Stop");
  }

  if (bArm == true)    // if true the motor can start 
  {
    lcd.display();
    ch1Value = readChannel(CH1, -250, 250, 0);

    if (ch1Value > 5 || ch1Value <-5)  //if Armed and active then do not power off
    {
      bActivity = true;
      Serial.println("Activity");

    } 
    else
    {
      bActivity = false;
      //Serial.println("No Activity");
    }  
    lcd.setCursor(0,1);
    lcd.print("Saftey off: "); lcd.print(ch1Value);

    motor.setSpeed(ch1Value);  // Run forward at 50% speed.
    //ch3Value = readChannel(CH3, -100, 100, -100);
    //Serial.print("Ch1: ");
    //Serial.println(ch1Value);

  }

}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue)
{
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) 
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Red the channel and return a boolean value
bool redSwitch(byte channelInput, bool defaultValue)
{
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setPWROff()
{ 
  motor.setSpeed(0);    // Stop.
  Serial.println("Switching off");
  digitalWrite(PWR, false);  // switch on relay 
}

int buttonValue(int buttonPin)
{
  int currentState;              // the current reading from the input pin
  currentState = digitalRead(buttonPin);
  if (currentState != lastButtonState) 
  {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) 
  {
    // if the button state has changed:
    if(currentState != buttonState)
    {
      buttonState = currentState;
    }
  }
  lastButtonState = currentState;
  return buttonState;
}

int autoPWRoff()
{
  if (bActivity == true) 
  {
    // reset the debouncing timer
    lastActivityTime = millis();
  }
  //Serial.print("Inactivity Period: ");
  //Serial.println(millis() - lastActivityTime);
  if ((millis() - lastActivityTime) > INACTIVITY_TIME) 
  {
    // if the button state has changed:
    //Serial.println("Power off");
    return true;
  }
  return false;
}
