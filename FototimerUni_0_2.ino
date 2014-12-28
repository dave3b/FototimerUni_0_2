#define nop() __asm volatile ("nop")
#if 1
nop();
#endif
/* ^^^ above bugfix for preprozessor, don't relay on it, it doesn't work propably ^^^
*******************************************************************************************/

// enable C++ style printing. C++ syntax would be:
//     cout << "Variable alpha is " << alpha << endl;
// Using the following, Arduino could print the same to the simulated USB port like this:
//     Serial << "Variable alpha is " << alpha << "\n";
// Can even use to print to the lcd:
//     lcd << "alpha=" << alpha;
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }

/*********************************************************************************************
FotoTimerUni 0.1
based on FotoTimerYun0.31

- rotary encoder increment or decrement values
- timer controlled by hardware timer interrupt
- collision calculation
- camera reaction time correction
- settings an config saved in EEPROM
- elementary timer functions outscourced
- calculate exposure ramp function without table
*** under development: ***
- serial communication for automatic ISO switch
  (without bridge library @ yun, or a RaspberryPi as slave connected via USB)
- use USB-Host-Shield for automatic ISO switch (on Mega ADK)
- compile Options for serveral hardware and ISO switch solutions
 */
 
/* *********************************************
** IMPORTANT! FIRST DEFINE HARDWARE PLATFORM! **
***********************************************/
//#define HARDWARE_UNO  // uncomment this, if a Arduino UNO is used
#define HARDWARE_YUN // uncomment this, if a Arduino YUN is used
//#define HARDWARE_MEGA // // uncomment this, if a Arduino Mega ADK is used

/* *********************************************
** IMPORTANT! SECOND DEFINE ISO REMOTE TYPE!  **
***********************************************/
//#define ISOREMOTE_SERIAL  // uncomment this, if a RaspberryPi as slave is used
#define ISOREMOTE_YUN  // uncomment this, if a Arduino YUN is used
//#define ISOREMOTE_USBHOST  // uncomment this, if a Arduino Meaga ADK is used

// set pin numbers and other constants:
const int batteryPin = 1;  // the number of the analog input used for the battery watching
const int encoderPinA = 3;  // the number of the encoder A digital input pin
const int encoderPinB = 4;  // the number of the encoder B digital input pin
const int encoderPinE = A4;  // the number of the encoder Enter digital input pin
#ifdef HARDWARE_MEGA // MEGA ADK needs other Pin setup, because pin7 is used for USB interupt
  // not used on MEGA ADK     // the number of the option signal digital output pin
  const int shutterPin = 6;   // the number of the shutter signal digital output pin
#else
  const int optionPin = 6;    // the number of the option signal digital output pin
  const int shutterPin = 7;   // the number of the shutter signal digital output pin
#endif
const int flashPin = 2;    // the number of the flash signal digital input pin
const int contrastPin = 5;   // the number of the pwr signal pin for contrast voltage
const float intermax = 5940.0; // maximaler interval [S], 5940S = 99M
const float expomax = 5792.618457;  // maximaler exposure time [S], 5940S = 99M
const unsigned long pausechecktime = 5; // time [mS] before shutter to check pause
const unsigned long flashlimit = 1000; // 1000 mS timeout limit for flashback signal
const char softvers[] = "FototimerUni 0.1";

const bool SKIP_INTRO = true;

// include library codes:
#include <LiquidCrystal.h>
#include <MsTimer2.h>
#include <EEPROM.h>
#include <EEPROManything.h>
#include <analogKeyboard.h>
#include <FotoTimerSymbols.h>
#include <exposurevary.h>
#include <PrintSubFunctions.h>

#ifdef ISOREMOTE_YUN
  #include <yunremote.h>
#endif

#ifdef ISOREMOTE_USBHOST
  //#include <megaremote.h>
#endif

#ifdef ISOREMOTE_SERIAL
  //#include <isoremote.h>
#endif





// variables for keyboard
//int analogKey = 1024; // analog value of keyboard input to compare
int keycode = 12; // 6 when six, 12 when twelve keys
int keyValueH = 0; // horizontal key direction
int keyValueV = 0; // vertical key direction
int temp_input = 0; // input buffer integer
float temp_infloat = 0; // input buffer float
boolean coarse = false; // switches coarse and fine, false = fine, true = coarse
// variables for encoder and input handling
int encoderA = LOW;
int encoderB = LOW;
int encoderE = HIGH;
volatile int encoderValue = 0; //volatile, used in interups routine
// variables for timer handling
boolean option = false;
boolean shutter = false; // shutter signal output
volatile unsigned long nextaction = 0; // time in mS to next timer action, volatile, used in interups routine
boolean timerpause = true; // start in pause mode
int isostate = 0; // ISO switch handling
volatile int timerstate = 2; // represents the actually state of the timer function, volatile, used in interups routine
// variables to measure and calculate the shutter delay
// startflash - startshutter = shutterdelaytime
unsigned long startshutter = 0; // timestamp when shutter state starts
unsigned long stopshutter = 0; // timestamp when shutter state stops
volatile unsigned long startflash = 0; // timestamp when flash signal starts, volatile, used in interups routine
unsigned long stopflash = 0; // timestamp when flash signal stops
int shutterdelaytime = 0; // calculated shutterdelaytime between shutter start and flash start in mS
int expooffset = 0; // fix time correcture in mS for exposure time
boolean delayactive = false; // no shutter delay measurement
int aftershutterdelay = 750; // minimum time in mS between two pictures
// variables for timer parameters
float intervaltime = 0; // real value loaded at start from EEPROM
float intervalramp = 0; // real value loaded at start from EEPROM
float exposuretime = 0; // real value loaded at start from EEPROM
float exposureramp = 0; // real value loaded at start from EEPROM
int isolevel = 0; // ISO level @ camera, read via USB
int isolvold = 0; // last ISO level before switch
int isomin = 50; // minimum ISO level, stored in config
int isomax = 3200; // maximum ISO level, stored in config
float isotrigger = 0; // pre trigger time for automatik ISO switch
// variables user interface and control
int menuescreen = 0; // main menue screen state
int settings1state = 0; // settings first level menue screen state
int settings2state = 0; // settings second level menue screen state
int configstate = 0; // config menue screen state
int functionscreen = 0; // function menue screen state
int checklist = 0; // counter for checklist function
// temp and misc variables
int temp_int = 0; // universal tmp interger variable
float temp_float = 0; // universial tmp float variable
float temp2_float = 0; // second tmp float variable
int piccount = 0; // picture counter
int picgoal = 0; // the number picture goal
float collision = 0; // time to exposure collision interval
int contrastValue = 0; // display contrast level
int lowbattery = 611; // level when battery is low Voltage=AnalogIn*0.01014, stored in config
// temp time counter
int loopactioncounter = 0;  
// debug
unsigned long timestamp1 = 0; // first timestamp
unsigned long timestamp2 = 0; // second timestamp

// initialize the library with the numbers of the interface pins
/*  The LCD circruit
 * LCD RS pin to digital pin 13
 * LCD Enable pin to digital pin 12
 * LCD D4 pin to digital pin 11
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 9
 * LCD D7 pin to digital pin 8
 * LCD R/W pin to ground
 * LCD V0 (Pin 3) over RC and OPAMP to PWR Pin5 (Contrast)
*/
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);


// EEPROM load and save functions
void loadConfig(void);
void saveConfig(void);
void loadSettings(void);
void saveSettings(void);
// programm funtion for better codse reading
void newCycle(void);
void statusScreen(void);
void settings1Srceen(void);
void settings2Srceen(void);
void configScreen(void);
void functionScreen(void);
void collision_calculation_manually(void);
void collision_calculation_automatic(void);
void iso_settings_manually(void);
void iso_settings_automatic(void);
void iso_switch_up(void);
void iso_switch_down(void);
// display time human readable
void printtime(float);

/****************************************
*** first execute once the setup only ***
****************************************/
void setup()
{
  // variable to setup custom display characters
  uint8_t newLCDchar[8];
  
  // first load config and settings
  loadConfig();
  loadSettings();
  // set analog Voltage for contrast
  pinMode(contrastPin, OUTPUT);
  analogWrite(contrastPin, contrastValue);
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16,2);
  // create own charaters
  for (temp_int = 0; temp_int < 8; temp_int++)
  {
    // get symbol from libary
    FotoTimerSymbols(newLCDchar,temp_int);
    // write symbol into LCD RAM
    lcd.createChar(uint8_t(temp_int),newLCDchar);
  }
  // print softwareversion message to the LCD.
  lcd.setCursor(0, 0);
  lcd << softvers;
  // set up the analog reference
  analogReference(EXTERNAL);
  // initialize encoder input pins
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinE, INPUT);
  // initialize foto interface pins
  // set them to high, because low active
  pinMode(flashPin, INPUT);
  #ifndef HARDWARE_MEGA
    pinMode(optionPin, OUTPUT);
    digitalWrite(optionPin, HIGH);
  #endif
  pinMode(shutterPin, OUTPUT);
  digitalWrite(shutterPin, HIGH);
  #ifdef HARDWARE_YUN
    // on YUN, the interupts are "twisted"
    // flash signal on pin 2 interrupt
    // interrupt attached at pause/start
    //attachInterrupt(1, timerflash, FALLING);
    // config interrupts for rotary encoder on pin 3
    attachInterrupt(0, getEncoder, FALLING);
  #else
    // flash signal on pin 2 interrupt
    // interrupt attached at pause/start
    //attachInterrupt(0, timerflash, FALLING);
    // config interrupts for rotary encoder on pin 3
    attachInterrupt(1, getEncoder, FALLING);  
  #endif
  // start delay for reading welcome message
  temp_int = 7; 
  do
    {
      lcd.setCursor(15,1);
      lcd.write(char(7-temp_int));
      lcd.setCursor(0, 1);
      lcd.print(temp_int);
      lcd.print(" Seconds to go");
      --temp_int;
      if( !SKIP_INTRO ) delay(1000);
    } while (temp_int > 0);
  // after startup (7 seconds time since power on ...)
  // start ISO speed remote functionality
  isoremotebegin();
  // clear the display  
  lcd.clear();
  // clear temporaly variables
  temp_input = 0;
  temp_int = 0;
  // prepare the first timer cyclus after 1S (1000mS)
  nextaction = 1000;
  MsTimer2::set(nextaction,timerpausecheck);
  // clear the display  
  lcd.clear();
  // and go for the timer interupt function
  MsTimer2::start();
}

/******************************************
*** the absolute one and only main loop ***
******************************************/
void loop() 
{
    // counter for simple timer actions
    ++loopactioncounter;
    // after 10 loops, reset counter
    if (loopactioncounter > 10)
    {
        loopactioncounter = 0;
    }
    // check periodically for ISO speed settings message
    if (loopactioncounter == 6)
      {
      // no check for success
      getanswer(&isolevel);
      }    
    // check if new cycle beginns
    // calculate the parameters
    if (timerstate < 0)
    {
        newCycle();
    }
    // get pressed key
    keycode = getKey(&keyValueH, &keyValueV);
    // user interface handling, switch between the screens 
    switch (menuescreen)
    {
      case 0:
        statusScreen();     
      break;
      case 1:
        settings1Srceen();
      break;
      case 2:
        settings2Srceen();
      break;
      case 3:
        functionScreen();
      break;
      case 4:
        configScreen();
      break;
    }
    // ------- global feature ------------
    // display timerstate on LCD
    lcd.setCursor(15,1);
    switch(timerstate)
    {
      case 2:
        if (timerpause == false)
         {
         lcd.print("\x04");
         }
        else
         {
         lcd.print("\xF3");
         } 
      break;
      case 1:
        lcd.print("\x05");
      break;
      case 0:
        lcd.print("\x03");
      break;
      }
    lcd.setCursor(0,1);
    if ( analogRead(batteryPin) < lowbattery && loopactioncounter > 6)
      {
      lcd.print("\x02");
      }
    else if (isolevel > 0 )
      {
      lcd.print("\x06");
      }
    else
      {
      lcd.print(" ");
      }
    delay(100);
}

/********************************************
********************************************* 
  sub functions beginns here
*********************************************
*********************************************/
 
 // rotary encoder interrupt
 void getEncoder()
 {
  encoderA = digitalRead (encoderPinA);
  encoderB = digitalRead (encoderPinB);
  if (encoderA == LOW && encoderB == LOW && encoderE == HIGH)
   {
   ++encoderValue;
   }
 
  if (encoderA == LOW && encoderB == HIGH && encoderE == HIGH)
   {
   --encoderValue;
   }
  // end encoder interrupt
  }
 
 // ****************************************************
 //encoder enter button handling
  void getEnter()
  {
  encoderE = digitalRead (encoderPinE);
  if (encoderE == LOW)
    {
    // toggle fine and coarse
    coarse = !coarse;
    }
  delay(100);
  }
 
 // ****************************************************
 // timer interrupt functions
 // ****************************************************
 void timerpausecheck()
    {
    MsTimer2::stop(); // stop timer
    // next timer action after pausechecktime (5mS)
    nextaction = pausechecktime;
    if (timerpause == true)
      {
      // come to check pause again at next timer interrupt
      MsTimer2::set(nextaction,timerpausecheck);
      }
    else 
      {
      // go on to shutter signal at next timer interrupt
      MsTimer2::set(nextaction,timerstartshutter);
      }
    MsTimer2::start(); // start timer
    }

void timerstartshutter()
    {
    // time for shutter
    // LOW activ signals here
    digitalWrite(shutterPin, LOW);
    startshutter = micros(); // timestamp
    //noInterrupts(); // deactivate interrupts
    MsTimer2::stop(); // stop timer
    timerstate = 1; // sign shutter active, waiting for flash signal to user
    // calculate the shutter signal end  
    // exposure time + shutter delay time
    // (the shutter signal was given the exposure and delay time before)
    // (delay can be negative, so must be handled seperatly by sign)
    if (expooffset < 0)
      {
      nextaction = long( exposuretime * 1000) - long(abs(expooffset));
      }
    else
      {
      nextaction = long( exposuretime * 1000) + long(expooffset);
      }  
    // MsTimer2 not startet, because waiting for flash signal is a real hardware interrupt
    // check if flash signal is used, when not, next action now
    if (delayactive == false)
      {
      timerflash();
      }
    //interrupts(); // activate interrupts
    }

void timerflash()
  {
  startflash = micros(); // timestamp
  // nextaction was calculated the function before --> timerstartshutter()
  MsTimer2::set(nextaction,timerstopshutter);
  MsTimer2::start(); // start timer
  timerstate = 0; // sign shutter activ to user
  }

void timerstopshutter()
  {
  // time for shutter end
  // LOW activ signals here
  digitalWrite(shutterPin, HIGH);
  // use the two timestamps to calculate the measured delay from shutter via flash signal
  if (delayactive == true)
    {
    shutterdelaytime = int((startflash - startshutter)/1000);
    }
  else
    {
    shutterdelaytime = 0;
    }
  if (shutterdelaytime > 500)
    {
    shutterdelaytime = 0;
    } 
  // the next action ist pause check
  // interval - exposure time - expooffset - shutterdelaytime time - pause check time
  // we can use from last timer action: nextaction = exposure time - expooffset
  // in collision case, a fix time is used
  // 500mS buffer time
  if (intervaltime - float(aftershutterdelay)/1000 - exposuretime - float(expooffset)/1000 -float(pausechecktime)/1000 < 0)
    {
    nextaction = long(aftershutterdelay) + 10;
    }
  // not collision case, time is calculated
  else
    {  
    nextaction = long(intervaltime*1000) - nextaction - pausechecktime;
    }
  // when possible, shutterdelay is used in calculation
  if (delayactive == true && nextaction > long(shutterdelaytime))
    {
    nextaction = nextaction - long(shutterdelaytime);
    }
  MsTimer2::set(nextaction,timerpausecheck);
  MsTimer2::start(); // start timer
  timerstate = -1; // sign wait for next cycle to user
  }
    
  /*********************************
  do all things for a new cycle
  **********************************/
void newCycle()
  {
  // reset timer state
  timerstate = 2;
  // increment picture count
  ++piccount;
  // calculate the timer parameters for the next cycle
  // calculate new interval time
  temp_float = intervaltime * intervalramp / 600;
  intervaltime = intervaltime * pow(2,temp_float);  
  // calculate new exposuretime
  temp_float = intervaltime * exposureramp / 600;
  exposuretime = exposuretime * pow(2,temp_float);
  // calculate collision time and manage ISO switch
  if (isolevel == 0)
    {
    // manually mode
    collision_calculation_manually();
    }
  else
    {
    // automatic mode
    collision_calculation_automatic();    
    }  
  }


/**********************************************
**********  main status screen ***************/
void statusScreen()
  {
  // first status screen, display
  // * first line *
  // - exposure ramp
  // - exposure time
  // - interval time
  // * second line *
  // - framecount
  // - time till frame goal
  // - collision / ISO level
  
  lcd.setCursor(0, 0);
  if (exposureramp >= 0)
    {
    lcd.print("+");
    }
  lcd.print(exposureramp,1);
  //lcd.print(" ");
  lcd.setCursor(4, 0);
  lcd.print("|");
  // alternativ display for better human reading
  printtime(exposuretime);
  lcd.setCursor(11, 0);
  lcd.print("|");
  // interval time
  // alternativ display for better human reading
  printtime(intervaltime);
  // frame / picture counter
  lcd.setCursor(1, 1);
  lcd.print(piccount);
  lcd.print("   ");
  // time for reaching the goal of frames / pictures
  // alternativ during exposure, exposure countdown
  lcd.setCursor(5, 1);
  lcd.print("|");
  if (timerstate == 0)
    {
    temp_int = int((startflash + long(exposuretime * 1000000) - micros())/1000000);
    if (temp_int < 0)
      {
      temp_int = 0;
      }
    lcd.print(temp_int);  
    }
  else
    {
    lcd.print(int(float(picgoal-piccount)*intervaltime/60));
    }
  lcd.print("  ");
  // collision forecast - ISO level
  lcd.setCursor(9, 1);
  lcd.print("|");
    // alternativ display for better human reading
    if ( timerpause == false)
      {
      if (exposureramp != 0)
        {
        if (collision > 150)
          {
          lcd.print(int(collision/60));
          lcd.print("min  ");
          }
        else
          {
          lcd.print(int(collision));
          lcd.print("Sec  ");
          }         
        }
      else
        {
        lcd.print(" --- ");
        }
      }  
    else
      {
      lcd.print("Pause");
      } 
   
    
  // key actions
  if (keycode < KEY_CODE_NEUTRAL || encoderValue != 0)
    {
    exposureramp = exposureramp + float(keyValueV) / 10;
    exposureramp = exposureramp + float(encoderValue) / 10;
    if (exposureramp > 5)
      {
      exposureramp = 5;
      }
    if (exposureramp < -5)
      {
      exposureramp = -5;
      }
    if (abs(exposureramp) < 0.1)
      {
      exposureramp = 0;
      }
    menuescreen = keyValueH;
    if (menuescreen < 0)
      {
      menuescreen = 2;
      }
    if (keycode == KEY_CODE_OK)
      {
      // get iso level now
      getisolevel();
      // loop until success
      while ( getanswer(&isolevel) == 0 );
      // if flashback function is used, activate interrupt
      if (delayactive == true)
        {
        #ifdef HARDWARE_YUN
          // on YUN, the interupts are "twisted"
          // flash signal on pin 2 interrupt
          attachInterrupt(1, timerflash, FALLING);
        #else
          // flash signal on pin 2 interrupt
          attachInterrupt(0, timerflash, FALLING);
        #endif
        }  
      timerpause = false;
      }
    if (keycode == KEY_CODE_CANCEL)
      {
      #ifdef HARDWARE_YUN
        // on YUN, the interupts are "twisted"
        // flash signal on pin 2 interrupt
        detachInterrupt(1);
      #else
        // flash signal on pin 2 interrupt
        detachInterrupt(0);
      #endif  
      timerpause = true;
      }
    // done with key actions
    keycode = KEY_CODE_NEUTRAL;
    encoderValue = 0;
    }
  } // end status screen 1
    
/******************************
****  settings screen one *****/
void settings1Srceen()
  {
  if (isolevel == 0)
    {
    iso_settings_manually();
    }
  else
    {
    iso_settings_automatic();
    }
  lcd.setCursor(1,1);
  lcd.print("Et");
  lcd.write(uint8_t (cursor(settings1state,1)));
  // alternativ display for better human reading
  printtime(exposuretime);
  lcd.setCursor(9,1);
  lcd.print("|G");
  lcd.write(uint8_t (cursor(settings1state,2)));
  lcd.print(picgoal);
  lcd.print("  ");
  // key handling
  if (keycode < KEY_CODE_NEUTRAL || encoderValue != 0)
    {
    settings1state = settings1state + keyValueH;
    if (settings1state > 2)
      {
      settings1state =   0;
      ++menuescreen;
      }
    if (settings1state < 0)
      {
      settings1state =   0;
      --menuescreen;
      }
    switch(settings1state)
      {
      // case 0 was handled in iso_settings_*** function before
      case 1:
        exposuretime = exposurevary(exposuretime,keyValueV);
        exposuretime = exposurevary(exposuretime,encoderValue);
        // check lower limit
        if (exposuretime < 0.05)
          {
          exposuretime = 0.05;
          }
        // check upper limit
        if (exposuretime > expomax)
          {
          exposuretime = expomax;
          }
        // pause timer
        if (keycode == KEY_CODE_CANCEL)
          {
          timerpause = true;
          }  
        // continue timer
        if (keycode == KEY_CODE_OK)
          {
          timerpause = false;
          }
      break;
      case 2:
        picgoal = picgoal + keyValueV * 10;
        picgoal = picgoal + encoderValue * 10;
        if (picgoal > 1000)
          {
          picgoal = 1000;
          }
        if (picgoal < 0)
          {
          picgoal = 0;
          }
      break;
      }
      keycode = KEY_CODE_NEUTRAL;
      encoderValue = 0;
    }    

  } // end settings screen 1

/*******************************
***** settings screen two ******/
void settings2Srceen()
  {   
  lcd.setCursor(0,0);
  lcd.print("Ir");
  lcd.write(uint8_t (cursor(settings2state,0)));
  lcd.print(intervalramp,1);
  lcd.setCursor(6,0);
  lcd.print("|It");
  lcd.write(uint8_t (cursor(settings2state,1)));
  // for better human reading
  printtime(intervaltime);
  lcd.setCursor(1,1);
  lcd.write(uint8_t (cursor(settings2state,2)));
  lcd.print("S-func");
  lcd.setCursor(8,1);
  lcd.write(uint8_t (cursor(settings2state,3)));
  lcd.print("Config");
  if (keycode < KEY_CODE_NEUTRAL || encoderValue != 0)
    {
    settings2state = settings2state + keyValueH;
    if (settings2state > 3)
      {
      settings2state =   0;
      ++menuescreen;
      if (menuescreen > 2)
        {
        menuescreen = 0;
        }
      }
    if (settings2state < 0)
      {
      settings2state = 0;
      settings1state = 2;
      --menuescreen;
      }  
    switch(settings2state)
      {
      case 0:
        // intervalramp here
        intervalramp = intervalramp + float(keyValueV) / 10;
        intervalramp = intervalramp + float(encoderValue) / 10;     
        if (intervalramp > 4)
          {
          intervalramp = 4;
          }
        if (intervalramp < -4)
          {
          intervalramp = -4;
          }  
      break;
      case 1:
        // intervaltime here
        intervaltime = intervaltime + float(keyValueV) / 2;
        intervaltime = intervaltime + float(encoderValue) / 10;
        // check upper limit
        if (intervaltime > intermax)
          {
          intervaltime = intermax;
          }
        // check lower limit 
        if (intervaltime < 1)
          {
          intervaltime = 1;
          }  
      break;
      case 2:
        if (keycode == KEY_CODE_OK)
          {
          menuescreen = 3;
          keycode = KEY_CODE_NEUTRAL;         
          }
      break;
      case 3:
        if (keycode == KEY_CODE_OK)
          {
          menuescreen = 4;
          keycode = KEY_CODE_NEUTRAL;         
          }
      break;
      }
    keycode = KEY_CODE_NEUTRAL;
    encoderValue = 0;
    }
  } // end settings screen
  
  /*********************************
   *******   config screen   ******/
  void configScreen()
    {
    switch(configstate)
      {
      case 0:
        lcd.setCursor(0,0);
        lcd.print(" Config - Menue\x7E");
        lcd.setCursor(1,1);
        lcd.print("back to setup?");
        if (keycode == KEY_CODE_OK)
          {
          menuescreen = 2;
          keycode = KEY_CODE_NEUTRAL;
          }
      break;
      case 1:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print(" ISO minimum  \x7E");
        lcd.setCursor(1,1);
        lcd.print(" Level:");
        lcd.print(isomin);
        lcd.print("      ");
        isomin = int(float(isomin) * pow(2,float(keyValueV)));
        isomin = int(float(isomin) * pow(2,float(encoderValue)));
        if (isomin < 50)
          {
          isomin = 50;
          }
        if (isomin > 3200)
          {
          isomin = 3200;
          }  
      break;
      case 2:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print(" ISO maximum  \x7E");
        lcd.setCursor(1,1);
        lcd.print(" Level:");
        lcd.print(isomax);
        lcd.print("      ");
        isomax = int(float(isomax) * pow(2,float(keyValueV)));
        isomax = int(float(isomax) * pow(2,float(encoderValue)));
        if (isomax < 50)
          {
          isomax = 50;
          }
        if (isomax > 3200)
          {
          isomax = 3200;
          }  
      break;
      case 3:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print("Flashback? Y/N\x007E");
        lcd.setCursor(1,1);
        if (keyValueV > 0)
          {
          delayactive = true;
          }
        if (keyValueV < 0)
          {
          delayactive = false;
          }  
        if (delayactive == true)
          {
          lcd.print("yes");
          }
        else
          {
          lcd.print("no ");
          }
        lcd.print(" (u=Y/d=N) ");
      break;
      case 4:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print("expos.-offset:\x7E");
        lcd.setCursor(1,1);
        lcd.print(expooffset);
        lcd.print("mS    ");
        expooffset = expooffset + keyValueV;
        if (expooffset > 999)
          {
          expooffset = 999;
          }
        if (expooffset < -99)
          {
          expooffset = -99;
          }         
        lcd.setCursor(6,1);
        lcd.print("|");    
        if (timerstate == 0)
          {
          lcd.print(startflash - startshutter);
          lcd.print("uS    ");
          }
        else
          {
          lcd.print("  ---   ");
          }
        if (keycode == KEY_CODE_OK)
          {
          timerpause = false;  
          }
        if (keycode == KEY_CODE_CANCEL)
          {
          timerpause = true;  
          }  
      break;
      case 5:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print(" camera-busy- \x7E");
        lcd.setCursor(1,1);
        lcd.print(" time:");
        lcd.print(aftershutterdelay);
        lcd.print("mS     ");
        aftershutterdelay = aftershutterdelay + keyValueV * 50;
        aftershutterdelay = aftershutterdelay + encoderValue * 50;
        if (aftershutterdelay < 0)
          {
          aftershutterdelay = 0;
          }
        if (aftershutterdelay > 5000)
          {
          aftershutterdelay = 5000;
          }        
      break;
      case 6:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print("** Display  **\x7E");
        lcd.setCursor(1,1);
        lcd.print(" Contrast:");
        lcd.print(contrastValue);
        lcd.print("   ");
        analogWrite(contrastPin, contrastValue);
        contrastValue = contrastValue + keyValueV;
        contrastValue = contrastValue + encoderValue;
        if (contrastValue < 0)
          {
          contrastValue = 0;
          }
        if (contrastValue > 254)
          {
          contrastValue = 254;
          }  
      break;
      case 7:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print("Battery Limit:\x7E");
        lcd.setCursor(1,1);
        lcd.print("L");
        // old 98.64
        lcd.print(float(lowbattery)/91.2);
        lcd.print("V ");
        lcd.setCursor(8,1);
        lcd.print("/");
        // old 98.64
        lcd.print(float(analogRead(batteryPin))/91.2);
        lcd.print("V ");
        lowbattery = lowbattery + keyValueV;
        lowbattery = lowbattery + encoderValue;
        if (lowbattery < 0)
          {
          lowbattery = 0;
          }
        if (lowbattery > 1023)
          {
          lowbattery = 1023;
          }       
      break;
      case 8:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print(" Configuration ");
        lcd.setCursor(1,1);
        lcd.print(" save and exit?");
        if (keycode == KEY_CODE_OK)
          {
          lcd.setCursor(1,1);
          lcd.print("saving ...      ");
          saveConfig();
          delay(500);
          lcd.setCursor(1,1);
          lcd.print("... done        ");
          menuescreen = 2;
          configstate = 0;
          keycode = KEY_CODE_NEUTRAL;
          delay(500);
          }      
      break;
      } // ebd switch loop
    if (keycode < KEY_CODE_NEUTRAL)
      {
      configstate = configstate + keyValueH;
      keycode = KEY_CODE_NEUTRAL;
      }
    if (configstate < 0)
      {
      configstate = 8;
      }
    if (configstate > 8)
      {
      configstate = 0;
      }  
    keycode = KEY_CODE_NEUTRAL;
    encoderValue = 0;
    } // end config Screen
  
 
 /**********************************
  **** special function screen ****/
  void functionScreen()
    {
    switch(functionscreen)
      {
      case 0:
      lcd.setCursor(0,0);
      lcd.print("spec.-functions\x7E");
      lcd.setCursor(1,1);
      lcd.print("back to setup? ");
      if (keycode == KEY_CODE_OK)
        {
        menuescreen = 2;
        keycode = KEY_CODE_NEUTRAL;
        }
      break;
      case 1:
      lcd.setCursor(0,0);
      lcd.print("\x7F");
      lcd.print("save settings?\x7E");
      lcd.setCursor(1,1);
      lcd.print(" (press O.K.)  ");
      if (keycode == KEY_CODE_OK)
        {
        lcd.setCursor(1,1);
        lcd.print(" saving ...   ");
        saveSettings();
        delay(500);
        lcd.setCursor(1,1);
        lcd.print(" ... done     ");
        keycode = KEY_CODE_NEUTRAL;
        delay(500);
        }
      break;
      case 2:
      lcd.setCursor(0,0);
      lcd.print("\x7F");
      lcd.print("load settings?\x7E");
      lcd.setCursor(1,1);
      lcd.print(" (press O.K.) ");
      if (keycode == KEY_CODE_OK)
        {
        lcd.setCursor(1,1);
        lcd.print(" load ...   ");
        loadSettings();
        delay(500);
        lcd.setCursor(1,1);
        lcd.print(" ... done   ");
        keycode = KEY_CODE_NEUTRAL;
        delay(500);
        }
      break;
      case 3:
      // first check, if timer ist in pause mode
      if (timerpause == false)
        {
        // timer ist not in pause mode, need to switch mode
        timerpause = true;
        // reset picturecount
        piccount = 0;
        }
      lcd.setCursor(0,0);
      lcd.print("\x7F");
      lcd.print(" installation \x7E");
      lcd.setCursor(1,1);
      /* Debuginfo Key repeat function
      lcd.print(getkeyrepeatcount());
      lcd.print(":");
      */
      lcd.print("expot.:");
      //alternativ display for better human reading
      printtime(exposuretime);
      exposuretime = exposurevary(exposuretime,keyValueV);
      exposuretime = exposurevary(exposuretime,encoderValue);
      // check lower limit
      if (exposuretime < 0.05)
        {
        exposuretime = 0.05;
        }
      // check upper limit
      if (exposuretime > expomax)
        {
        exposuretime = expomax;
        }
      if (exposuretime > intervaltime)
        {
        // interval time must be longer than exposure time
        intervaltime = exposuretime + 1;
        }
      if (keycode == KEY_CODE_OK)
        {
        timerpause = false;
        exposureramp = 0;
        intervalramp = 0;
        keycode = KEY_CODE_NEUTRAL;
        }
      break;
      case 4:
      lcd.setCursor(0,0);
      lcd.print("\x7F");
      lcd.print("  checklist   \x7E");
      lcd.setCursor(1,1);
      lcd.print(" ");
      switch(checklist)
        {
        case 0:
          lcd.print("stativ safe?  ");
        break;
        case 1:
          lcd.print("memory card?  ");
        break;
        case 2:
          lcd.print("bulb mode?    ");
        break;
        case 3:
          lcd.print("lens twisted? ");
        break;
        case 4:
          lcd.print("settings made?");
        break;
        case 5:
          lcd.print("let's go!     ");
        break;
        }
      if (keycode == KEY_CODE_OK)
        {
        ++checklist;
        if (checklist > 5)
          {
          // reset all screens
          menuescreen = 0; // main menue screen state
          settings1state = 0; // settings first level menue screen state
          settings2state = 0; // settings second level menue screen state
          configstate = 0; // config menue screen state
          functionscreen = 0; // function menue screen state
          checklist = 0; // counter for checklist function
          }
        keycode = KEY_CODE_NEUTRAL;            
        }
      if (keycode == KEY_CODE_CANCEL)
        {
        checklist = 0;
        keycode = KEY_CODE_NEUTRAL;
        }
      break;
      case 5:
        lcd.setCursor(0,0);
        lcd.print("\x7F chg. ISOspeed\x7E");
        if (keyValueV > 0)
          {
          isolevel = isolevel * 2;
          if (isolevel == 0)
            {
            isolevel = isomin;
            }
          if (isolevel > isomax)
            {
            isolevel = isomax;
            isolvold = isolevel;
            }
          }
        if (keyValueV < 0)
          {
          isolevel = isolevel / 2;
          if (isolevel == 0)
            {
            isolevel = isomax;
            }
          if (isolevel < isomin)
            {
            isolevel = isomin;
            isolvold = isolevel;
            }
          }       
        lcd.setCursor(1,1);
        lcd.print("current:");
        if (isolevel == 0 || isolvold != 0)
          {
          lcd.print("off  ");
          }
        else
          {
          lcd.print(isolevel);
          lcd.print("   ");
          }
        if (keycode == KEY_CODE_OK)
          {
          if (isolevel == 0 && isolvold > 0)
            {
            getisolevel();
            lcd.setCursor(1,1);
            lcd.print("read ISOspeed ");
            delay(1000);
            getanswer(&isolevel);
            isolvold = 0;
            }
          else if (isolevel == isolvold)
            {
            isolevel = 0;
            delay(500);
            }
          else
            {
            setisolevel(isolevel);
            lcd.setCursor(1,1);
            lcd.print(" set ISOspeed ");
            delay(1000);
            getanswer(&isolevel);
            }
          keycode = KEY_CODE_NEUTRAL;
          }        
      break;
      case 6:
        lcd.setCursor(0,0);
        lcd.print("\x7F");
        lcd.print("  power off?   ");
        lcd.setCursor(1,1);
        lcd.print(" Yes = OK    ");     
        if (keycode == KEY_CODE_OK)
          {
          lcd.setCursor(0,0);
          lcd.print("\x7F");
          lcd.print("  ISO client   ");
          lcd.setCursor(1,1);
          lcd.print(" powering off ");    
          #ifdef ISOREMOTE_SERIAL
          clientpoweroff();
          while (isolevel > 0)
            {
            getanswer(&isolevel);
            }
          #endif
          functionscreen = 0;
          menuescreen = 2;
          keycode = KEY_CODE_NEUTRAL;
          }
      break;
      }
    if (keycode < KEY_CODE_NEUTRAL)
      {
      functionscreen = functionscreen + keyValueH;
      keycode = KEY_CODE_NEUTRAL;
      }
    if (functionscreen < 0)
      {
      functionscreen = 6;
      }
    if (functionscreen > 6)
      {
      functionscreen = 0;
      }  
    keycode = KEY_CODE_NEUTRAL;
    encoderValue = 0;
    }// end function Screen

/*******************************************************
 ** manually and automatic ISO switching subfunctions **
 *******************************************************/

// calculate collision time in manual ISO switching mode
void collision_calculation_manually(void)
  {
   // calculate the collision time
   // manage automatic ISO level switch
   collision = 0; // reset collision time
   temp2_float = exposuretime; // store actually exposuretime temporary for calculation
   // calculate collision time via loop
   if (exposureramp > 0)
     {
     // loop until intervaltime is reached
     while (temp2_float < intervaltime)
       {
       temp_float = intervaltime * exposureramp / 600;
       temp2_float = temp2_float * pow(2,temp_float);
       collision = collision + intervaltime;
       }
     }
   if (exposureramp < 0)
     {
     // loop until expomin is reached, expomin = 0.05 Seconds
     while (temp2_float > 0.05)
       {
       temp_float = intervaltime * exposureramp / 600;
       temp2_float = temp2_float * pow(2,temp_float);
       collision = collision + intervaltime;
       }
     }  
  }
// calculate collision time in automatic ISO switching mode
void collision_calculation_automatic(void)
  {
   // calculate the collision time
   collision = 0; // reset collision time
   temp2_float = exposuretime; // store actually exposuretime temporary for calculation
   // calculate collision time via loop and manage ISO switching
   if (exposureramp > 0)
     {
     // check for necessity ISO level up switch
     if (exposuretime > (intervaltime-isotrigger) && isolevel < isomax && timerpause == false)
       {
       iso_switch_up();
       }
     // loop until intervaltime is reached
     // isotrigger can be left unconsidered
     while (temp2_float < intervaltime)
       {
       temp_float = intervaltime * exposureramp / 600;
       temp2_float = temp2_float * pow(2,temp_float);
       collision = collision + intervaltime;
       }
     // loop until end of ISO is reached
     temp_int=isolevel; // transfer actually ISO level into temp
     while(temp_int < isomax)
       {
       // next ISO level
       temp_int = temp_int * 2;
       // add time
       collision = collision + 600 / abs(exposureramp);
       }
     }
   if (exposureramp < 0)
     {
     // check for necessity ISO level down switch
     if (exposuretime *2 < (intervaltime-isotrigger) && isolevel > isomin && timerpause == false)
       {
       iso_switch_down();
       }     
     // loop until expomin is reached, expomin = 0.05 Seconds
     // isotrigger can be left unconsidered
     while (temp2_float > 0.05)
       {
       temp_float = intervaltime * exposureramp / 600;
       temp2_float = temp2_float * pow(2,temp_float);
       collision = collision + intervaltime;
       }
     // loop until end of ISO is reached
     temp_int=isolevel; // transfer actually ISO level into temp
     while(temp_int > isomin)
       {
       // next ISO level
       temp_int = temp_int / 2;
       // add time
       collision = collision + 600 / abs(exposureramp);
       }
     }
  }
// ISO settings screen in manually mode
void iso_settings_manually(void)
  {
  lcd.setCursor(0,0);
  lcd.print("ISO");
  lcd.write(uint8_t (cursor(settings1state,0)));
  lcd.print(temp_input);
  if (temp_input != 0)
    {
    lcd.print(" switch?  ");
    }
  else
    {
    if (isostate !=0 && timerstate == 2)
      {
      lcd.print(" change");
      lcd.print(isostate);
      lcd.print("! ");
      }
    else if (isostate !=0 && timerstate != 2)
      {
      lcd.print(" wait!     ");
      }
    else  
      {
      lcd.print("           ");
      }
    }

  if (keycode < KEY_CODE_NEUTRAL || encoderValue != 0)
    {
    switch(settings1state)
      {
      case 0:
        temp_input = temp_input + keyValueV;
        if (temp_input > 10)
          {
          temp_input = 10;
          }
        if (temp_input < -10)
          {
          temp_input = -10;
          }
        if (keycode == KEY_CODE_OK)
          {
          // if nothing else is to do, timer go on
          if (isostate == 0 && temp_input == 0)
            {
            timerpause = false;
            }
          // if timer is in waiting state, the ISO switch calculation must be done here
          if (isostate != 0 && timerstate == 2)
              {
              exposuretime = exposuretime * pow(2,float(-isostate));
              isolvold = int( float(isolvold) * pow(2,float(isostate)));
              isostate = 0;
              timerpause = false;
              }
          // if temp_input is not zero, there is a ISO-switch wish
          if (temp_input !=0)
            {
            timerpause = true;
            isostate = temp_input;
            temp_input = 0;
            }  
          }
        if (keycode == KEY_CODE_CANCEL)
          {
          if (temp_input == 0)
            {
            // deactivate automatic ISO level switch
            // and store ISO level for reactivating 
            isolevel = isolvold;
            isolvold = 0; 
            }
          else
            {
            temp_input = 0;
            timerpause = false;
            }
          }  
      break;
      // case 1 and 2 will be handled later
      }
    }    
  }
// ISO settings screen in automatic mode
void iso_settings_automatic(void)
  {
  lcd.setCursor(0,0);
  lcd.print("iptt");
  lcd.write(uint8_t (cursor(settings1state,0)));
  lcd.print(isotrigger,1);
  lcd.setCursor(8,0);
  lcd.print("|ISO");
  lcd.print(isolevel);
  lcd.print("  ");
  if (keycode < KEY_CODE_NEUTRAL || encoderValue != 0)
    {
    switch(settings1state)
      {
      case 0:
        isotrigger = isotrigger + float(keyValueV) / 10;
        isotrigger = isotrigger + float(encoderValue) / 10;
        if (isotrigger > 5)
          {
          isotrigger = 5;
          }
        if (isotrigger < 0)
          {
          isotrigger = 0;
          }
        if (keycode == KEY_CODE_CANCEL)
          {
          // deactivate automatic ISO level switch
          // and store ISO level for reactivating 
          isolvold = isolevel;
          isolevel = 0;
          }
      break;
      // case 1 and 2 will be handled later
      }
    }    
  }
// switch ISO automatic up
void iso_switch_up(void)
  {
  // temp counter
  int count = 0;  
  // first pause timer!
  timerpause = true;
  // is there no uncomplete ISO switch?
  timestamp1 = millis();
  if ( isolvold == 0)
    {
    // remember old isolevel
    isolvold = isolevel;
    // give command to new isolevel
    setisolevel(isolvold * 2);
    // wait max. 10S for answer ...
    while (isolvold * 2 != isolevel)
      {
      getanswer(&isolevel);
      delay(10);
      ++count;
      if (count > 1000)
        {
        break;
        }
      }
    }
    timestamp2 = millis();
    #ifdef ISOREMOTE_YUN
        Serial1.print("Debug:");
        Serial1.print(timestamp2-timestamp1,DEC);
        Serial1.print("mS\n");
    #endif    
    // check for success
    if (isolvold * 2 == isolevel)
      {
      // half exposure time with new isolevel
      exposuretime = exposuretime / 2;
      // reset old ISO level
      isolvold = 0;
      }       
  // continue timer!
  timerpause = false;
  }
// switch ISO automatic down  
void iso_switch_down(void)
  {
  // temp counter
  int count = 0;  
  // first pause timer!
  timerpause = true;
  // is there no uncomplete ISO switch?
  timestamp1 = millis();
  if ( isolvold == 0)
    {
    // remember old isolevel
    isolvold = isolevel;
    // give command to new isolevel
    setisolevel(isolvold / 2);
    // wait max. 10S for answer ...
    while (isolvold / 2 != isolevel)
      {
      getanswer(&isolevel);
      delay(10);
      ++count;
      if (count > 1000)
        {
        break;
        }
      }
    }
    timestamp2 = millis();
    #ifdef ISOREMOTE_YUN
      Serial1.print("Debug:");
      Serial1.print(timestamp2-timestamp1,DEC);
      Serial1.print("mS\n");
    #endif
    // check for success
    if (isolvold / 2 == isolevel)
      {
      // half exposure time with new isolevel
      exposuretime = exposuretime * 2;
      // reset old ISO level
      isolvold = 0;
      }       
  // continue timer!
  timerpause = false;
  }

// display time human readable
void printtime(float displaytime)
  {
  // some temporary variables
  int temp_time = 0;
  if (displaytime > 99)
    {
    temp_time = int(displaytime / 60);
    lcd.print(temp_time);
    lcd.print("m");
    lcd.print(displaytime - float(temp_time) * 60,0);
    lcd.print("  ");
    }
  else if (displaytime < 1)
    {
    lcd.print(displaytime*1000,0);
    lcd.print("mS  ");
    }
  else
    {
    lcd.print(displaytime,3);
    lcd.print("S    ");
    }  
  }

   
 /**********************************
  * EEPROM load and save functions *
  **********************************/
void loadConfig()
  {
  // beginn with config @ adress 0
  int address = 0;
  address = address + EEPROM_readAnything(address, delayactive);
  address = address + EEPROM_readAnything(address, expooffset);
  address = address + EEPROM_readAnything(address, contrastValue);
  address = address + EEPROM_readAnything(address, lowbattery);
  address = address + EEPROM_readAnything(address, isomin);
  address = address + EEPROM_readAnything(address, isomax);
  address = address + EEPROM_readAnything(address, aftershutterdelay);
  }
void saveConfig()
  {
  // beginn with config @ adress 0
  int address = 0;
  address = address + EEPROM_writeAnything(address, delayactive);
  address = address + EEPROM_writeAnything(address, expooffset);
  address = address + EEPROM_writeAnything(address, contrastValue);
  address = address + EEPROM_writeAnything(address, lowbattery);
  address = address + EEPROM_writeAnything(address, isomin);
  address = address + EEPROM_writeAnything(address, isomax);
  address = address + EEPROM_writeAnything(address, aftershutterdelay);
  }
void loadSettings()
  {
  // beginn with settings @ adress 512
  int address = 512;
  address = address + EEPROM_readAnything(address, intervaltime);
  address = address + EEPROM_readAnything(address, intervalramp);
  address = address + EEPROM_readAnything(address, exposuretime);
  address = address + EEPROM_readAnything(address, exposureramp);
  address = address + EEPROM_readAnything(address, picgoal);
  address = address + EEPROM_readAnything(address, isotrigger);  
  }
void saveSettings()
  {
  // beginn with settings @ adress 512
  int address = 512;
  address = address + EEPROM_writeAnything(address, intervaltime);
  address = address + EEPROM_writeAnything(address, intervalramp);
  address = address + EEPROM_writeAnything(address, exposuretime);
  address = address + EEPROM_writeAnything(address, exposureramp);
  address = address + EEPROM_writeAnything(address, picgoal);
  address = address + EEPROM_writeAnything(address, isotrigger);  
  }
  


 
