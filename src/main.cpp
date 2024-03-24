

/*******************************************************************************
 *
 * Descrioption:
 * 
 * DC Motor speed controller useing RC (Radio Control) transmitter/receiver 
 * or PS4 Bluetooth controller allowing motor speed and direction contol.
 * 
 * Utilises Arm or Disarm control to prevent motor start by accident. 
 * Uses RC switch with RC and for Bluetooh, the bottom L3 switch. Probably better 
 * for right handed person.
 * 
 * To Arm. the Arm switch must be set and speed be zero. This is
 * a saftey feature to prevent the motor spiining up unintentially.
 * 
 * Setting the saftey switch while the motor is running will force motor to stop.
 * 
 * 
 * to make the PS4 Contoller library work you need the Bluetooth MAC address of the ESP 
 * configured in the PS4 Controller.
 * See https://sixaxispairtool.software.informer.com/ on how to do this. I only got the MS Windows version to work.
 * 
 * Uses:
 *  LCD 1602
 *  MDC30 R2: Motor Speed Controller
 *  AndyMark, am-0255 DC Motor 
 *  A PS4 Contoller or,
 *  an RC TX and RX with atleast two chamels. A throttle channel to also control direction. 
 * 
 *******************************************************************************************/

//***********************
// Application headers
//**********************
#include<Arduino.h>
//#include <Ticker.h>
#include "CytronMotorDriver.h"
#include <LiquidCrystal.h>
#include <PS4Controller.h>
#include "esp_bt_device.h"

void displayPS4controls();
String getDeviceAddress();
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);
bool readSwitch(byte channelInput, bool defaultValue);
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
#define PWR              14
#define PWROFF           39          // A3
#define DEBOUNCE_TIME    50          // the debounce time in millisecond.
#define INACTIVITY_TIME  300000      // After 5 Minutes of inactivity, power off
#define PS4_CNTRL        0           // Controller = Bluetooth connected PS4 controller
#define RC_CNTRL         1
#define CNTRL_TYPE       PS4_CNTRL
#define IDLE_TOLERENCE   2           // PS4 controller Right stick does not sit on 0 when idle
#define DISPLAY_DELAY    50

int lastButtonState = LOW;           // the previous flickerable state from the input pin
int buttonState;                     // last reading state from the input pin
 
unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long lastActivityTime = 0;

int i = 0;
int j = 0;
int  ch1Value;
bool ch5Value;
bool bArm = false;
bool bActivity = true; //If false it means that system is in use, if false switch off after x minuites
bool bCheckPWRButton = false;

// Configure the motor driver.
CytronMD motor(PWM_DIR, 26, 12);  // PWM = Pin A0, DIR = Pin x.
// Configure the LCD Display.
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

void setup() {
  String btMAC;
  motor.setSpeed(0);    // Stop.
  
  Serial.begin(115200);
  while (!Serial)
        delay(1000);
  PS4.begin();
  
  btMAC = getDeviceAddress();
  Serial.println(btMAC);
  
  // Power control 
  pinMode(PWR, OUTPUT);
  pinMode(PWROFF, INPUT);  // If high power off. Note that its high initially during power ON
  digitalWrite(PWR, true);  // switch on relay

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

  lcd.begin(16, 2);

  //**************************************
  // Print a start-up message to the LCD.
  //**************************************
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("    ObiChase    ");
  lcd.setCursor(0,1);
  if (CNTRL_TYPE == PS4_CNTRL)
  {
    //lcd.print(btMAC.c_str());
    lcd.print("Using PS4");
  }
  else
    lcd.print("Using RC TX & RX");
 
  delay(5000);
}

void loop() 
{
  // Below has all accessible outputs from the controller
  //if (PS4.isConnected()) 
  //{
  //  displayPS4controls();
  //}
  // A single button to control On and Off and display useful info
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
      delay(3000);
      if (buttonValue(PWROFF) == 0)
      {

      //lcd.setCursor(0,0);
      //lcd.print("0");
      //Serial.println("Button value: finger on");
         setPWROff();
      }   
    }
  }
  
  if (autoPWRoff() == true)
  {
    setPWROff();
  }

  //*******************************
  // motor control processing
  //*******************************
  if (bArm == false)
  {
    // do nothing until speed == 0)
    while (ch1Value != 0  )
    {
        //lcd.display();
        if (CNTRL_TYPE == RC_CNTRL)
        {
          ch1Value = readChannel(CH1, -250, 250, 0);
        }  
        else
        {
          ch1Value = map( PS4.RStickY(), -127, 127, -249, 249);
          if (ch1Value <= IDLE_TOLERENCE && ch1Value >= -IDLE_TOLERENCE)   // PS4 contoller does not seem to zero
          {
            ch1Value = 0;
          } 
        }
        if (ch1Value !=0)
        {
          lcd.setCursor(0,1);
          lcd.print("Set speed to 0  ");
          Serial.println("Set speed to 0");
          Serial.println(ch1Value);
        }
        delay(1000); // FIXTHIS - why is there a delay? Just to slow down prints
    }
  }

  // Check Saftey switch - on or off
  if (CNTRL_TYPE == RC_CNTRL)
    ch5Value = readSwitch(CH5, false);
  else
    ch5Value = !PS4.L3();   //FIXTHIS value is opposite from readswitch

  //Serial.print(ch5Value); Serial.print(ch1Value);
  if ( ch5Value == 0 )
  {
    bArm = true;   // motor demand from rc is 0
    bActivity = true;
    //Serial.println("Saftey off");
  }
  else
  {
    motor.setSpeed(0);    // Stop.
    bArm = false;
    bActivity = false;
    if (j % DISPLAY_DELAY == 0) 
    {
      lcd.setCursor(0,1);
      lcd.print("Saftey on : Stop");
      Serial.println("Saftey on : Stop");
      j=0;
    }
    j++;
  }

  //***************************************************************
  // At this point motor is stopped and either we are armed or not
  //***************************************************************
  if (bArm == true)    // if true the motor can start 
  {
    lcd.display();
    if (CNTRL_TYPE == RC_CNTRL)
    {
      ch1Value = readChannel(CH1, -250, 250, 0);
    }
    else
    {
      //PS4 contoller range is +/-127
      //ch1Value = map( PS4.RStickY(), -127, 127, -249, 249);
      ch1Value = map( PS4.RStickY(), -127, 127, -40, 40);
      if (ch1Value <= IDLE_TOLERENCE && ch1Value >= -IDLE_TOLERENCE)   // PS4 contoller does not seem to zero
            ch1Value = 0; 
    }

    if (ch1Value > IDLE_TOLERENCE || ch1Value <-IDLE_TOLERENCE)  //if Armed and active then do not power off
    {
      bActivity = true;       // reset auto power off timer
      //Serial.println("Activity");
    } 
    else
    {
      bActivity = false;
      //Serial.println("No Activity");
    }  
    if (i % DISPLAY_DELAY == 0) 
    {
      lcd.setCursor(0,1);
      //lcd.print("                ");
      lcd.print("Saftey off: "); lcd.print(ch1Value); lcd.print("    ");
      Serial.print("Saftey off: "); Serial.println(ch1Value);
      i = 0;
    }
    i++;

    motor.setSpeed(ch1Value);  
    //Serial.print("Ch1: ");
    //Serial.println(ch1Value);
  }
}

//******************************************************
// Get Bluetooth MAC Address
// Need to configure this Address into the controller 
//******************************************************
String getDeviceAddress() 
{
  String btMAC;
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    btMAC = btMAC + str;
    if (i < 5)
    {
      btMAC = btMAC + ":";
    }
  }
  return btMAC;
}

//
// Power off Motor - ESP is still running FIXTHIS - try deep sleep?
//
void setPWROff()
{ 
  motor.setSpeed(0);    // Stop.
  //Serial.println("Switching off");
  digitalWrite(PWR, false);  // switch on relay 
}

//**********************
// Get Button state
//**********************
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
//*****************************************************
// Auto power off feature
// if no activity for INACTIVITY_TIME then power off 
//*****************************************************
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






void displayPS4controls()
{
  if (PS4.Right()) Serial.println("Right Button");
    if (PS4.Down()) Serial.println("Down Button");
    if (PS4.Up()) Serial.println("Up Button");
    if (PS4.Left()) Serial.println("Left Button");

    if (PS4.Square()) Serial.println("Square Button");
    if (PS4.Cross()) Serial.println("Cross Button");
    if (PS4.Circle()) Serial.println("Circle Button");
    if (PS4.Triangle()) Serial.println("Triangle Button");

    if (PS4.UpRight()) Serial.println("Up Right");
    if (PS4.DownRight()) Serial.println("Down Right");
    if (PS4.UpLeft()) Serial.println("Up Left");
    if (PS4.DownLeft()) Serial.println("Down Left");

    if (PS4.L1()) Serial.println("L1 Button");
    if (PS4.R1()) Serial.println("R1 Button");

    if (PS4.Share()) Serial.println("Share Button");
    if (PS4.Options()) Serial.println("Options Button");
    if (PS4.L3()) Serial.println("L3 Button");
    if (PS4.R3()) Serial.println("R3 Button");

    if (PS4.PSButton()) Serial.println("PS Button");
    if (PS4.Touchpad()) Serial.println("Touch Pad Button");

    if (PS4.L2()) {
      Serial.printf("L2 button at %d\n", PS4.L2Value());
    }
    if (PS4.R2()) {
      Serial.printf("R2 button at %d\n", PS4.R2Value());
    }

    if (PS4.LStickX()) {
      Serial.printf("Left Stick x at %d\n", PS4.LStickX());
    }
    if (PS4.LStickY()) {
      Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    }
    if (PS4.RStickX()) {
      Serial.printf("Right Stick x at %d\n", PS4.RStickX());
    }
    if (PS4.RStickY()) {
      Serial.printf("Right Stick y at %d\n", PS4.RStickY());
    }

    if (PS4.Charging()) Serial.println("The controller is charging");
    if (PS4.Audio()) Serial.println("The controller has headphones attached");
    if (PS4.Mic()) Serial.println("The controller has a mic attached");

    Serial.printf("Battery Level : %d\n", PS4.Battery());

    Serial.println();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(1000);
}

