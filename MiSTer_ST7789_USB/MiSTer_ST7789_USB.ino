/*
  By Venice
  Get CORENAME from MiSTer via USB-Serial-TTY Device and show CORENAME related text, Pictures or Logos
  Using Adafruit_ST7789 Library for the ST7789 240x240
  This sketch is made to work only with Arduino Nano without any add-ons (no sensors, no screensaver, etc) to save memory
  Only XBM pictures supported at this time

  -- G R A Y S C A L E  E D I T I O N --

  Needed libraries for the Arduino program:
  - Adafruit GFX (*)
  (*) These Libraries can be installed using Arduino's library manager.
  See also https://github.com/venice1200/MiSTer_tty2oled/wiki/Arduino-HowTo-%28Windows%29

  QuickSelect/Copy&Paste for Arduino IDE v2.x:
  -WEMOS LOLIN32
   
  See changelog.md in Sketch folder for more details
  
  ToDo
  -Everything I forgot
   
  Defines?!
  I use a lot "#ifdef's...endif" to enable code for a specific MCU Type.
  USE_LOLIN32      An Arduino Nano is used with the Arduino IDE Profile "LOLIN32".
 
*/

// Set Version
#define BuildVersion "240911"                    // "T" for Testing

// Include Libraries
#include <Arduino.h>
#include "bitmaps.h"                              // Some needed pictures


// ---------------------------------------------------------------------------------------------------------------------
// ------------------------------------------- Activate your Options ---------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

// How-To...
// Comment (add "//" in front of the Option) to de-activate the Option
// Uncomment (remove "//" in front of the Option) to activate the Option

// Get Debugging Infos over Serial
//#define XDEBUG

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------- Auto-Board-Config via Arduino IDE Board Selection --------------------------------
// ----------------------------------- Make sure the Manual-Board-Config is not active ---------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#ifdef ARDUINO_LOLIN32            // Set Arduino Board to "WEMOS LOLIN32"
  #define USE_LOLIN32             // Wemos LOLIN32, LOLIN32, DevKit_V4
#endif

// ---------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Manual-Board-Config ------------------------------------------------
// ------------------------------------ Make sure the Auto-Board-Config is not active ----------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#define USE_LOLIN32            // Wemos LOLIN32, LOLIN32, DevKit_V4. Set Arduino Board to "WEMOS LOLIN32"

// ---------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Conditional dependencies --------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>             // Arduino SPI library
#include <Wire.h>

// ---------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Hardware-Config --------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------

// WEMOS LOLIN32/Devkit_V4 using VSPI SCLK = 18, MISO = 19, MOSI = 23, SS = 5 and...
#ifdef USE_LOLIN32
  int cDelay = 60;                 // Command Delay in ms for ACK-Handshake
  
  // ST7789 TFT module connections
  #define OLED_CS    10  // define chip select pin
  #define OLED_DC     9  // define data/command pin
  #define OLED_RESET    8  // define reset pin, or set to -1 and connect to Arduino RESET pin

#endif

// Initialize Adafruit ST7789 TFT library
Adafruit_ST7789 oled = Adafruit_ST7789(OLED_CS, OLED_DC, OLED_RESET);
#define OLED_WHITE ST77XX_WHITE
#define OLED_BLACK ST77XX_BLACK

// -------------------------------------------------------------
// ------------------------- Variables -------------------------
// -------------------------------------------------------------

// Strings
String newCommand = "";                // Received Text, from MiSTer without "\n" currently (2021-01-11)
String prevCommand = "";
String actCorename = "No Core loaded"; // Actual Received Corename
uint8_t contrast = 5;                  // Contrast (brightness) of display, range: 0 (no contrast) to 255 (maximum)
int tEffect = 0;                       // Run this Effect

bool updateDisplay = false;
bool startScreenActive = false;        
bool timeIsSet = false;                // Time was set from external ?
bool runsTesting = false;              // Testing ?
bool sendTTYACK = true;                // Sending ttyack back after Command

// Display Vars
uint16_t DispWidth, DispHeight, DispLineBytes1bpp, DispLineBytes4bpp;
int logoBytes1bpp=0;
int logoBytes4bpp=0;
//unsigned int logoBytes1bpp=0;
//unsigned int logoBytes4bpp=0;
const int hwDelay=100;                        // Delay for HWINFO Request
size_t bytesReadCount=0;
//uint8_t logoBin[8192];                        // fixed definition
uint8_t *logoBin;
enum picType {NONE, XBM, GSC, TXT};           // Enum Picture Type
int actPicType=NONE;
int16_t xs, ys;
uint16_t ws, hs;
const uint8_t minEffect=1, maxEffect=23;      // Min/Max Effects for Random
//const uint8_t minEffect=22, maxEffect=23;   // Min/Max Effects for TESTING

// =============================================================================================================
// ========================================== FUNCTION PROTOTYPES ==============================================
// =============================================================================================================

void oled_showStartScreen(void);
void oled_setTime(void);
void oled_setcdelay(void);
void oled_showcdelay(void);
void oled_showSmallCorePicture(int xpos, int ypos);
void oled_showSystemHardware(void);
void oled_sendHardwareInfo(void);
void oled_drawlogo64h(uint16_t w, const uint8_t *bitmap);
void oled_showcorename();
void oled_displayoff(void);
void oled_displayon(void);
void oled_updatedisplay(void);
void oled_readnsetcontrast(void);
void oled_showperror(void);
void oled_showcenterredtext(String text, int font);
void oled_setfont(int font);
void oled_clswithtransition();
void oled_showpic(void);
int oled_readlogo();
void oled_drawlogo(uint8_t e);
void oled_read_and_draw_logo();
void oled_drawEightPixelXY(int x, int y, int dx, int dy);
void oled_readnwritetext(void);
void oled_readndrawgeo(void);
void oled_showtemperature();
void oled_readnsetuserled(void);
void oled_settempzone(void);
void oled_readnsetpowerled(void);
void oled_enableOTA (void);
void oled_setttyack(void);
void oled_cleardisplay(void);
void oled_display(void);
void oled_setcursor(int16_t, int16_t);
void oled_settextcolor(int16_t, int16_t);
void oled_setcontrast(uint8_t);
void oled_printtext(String);
void oled_printftext(String, float);
void oled_printftext(String, int);
int16_t oled_getUTF8Width(const char *);
int8_t oled_getFontAscent(void);
void oled_drawgreyscale(const uint8_t);

// Info about overloading found here
// https://stackoverflow.com/questions/1880866/can-i-set-a-default-argument-from-a-previous-argument
inline void oled_drawEightPixelXY(int x, int y) { oled_drawEightPixelXY(x,y,x,y); };

// =============================================================================================================
// ================================================ SETUP ======================================================
// =============================================================================================================

void setup(void) {
  String bversion = BuildVersion;

  if (bversion.endsWith("T")) runsTesting=true;     // Running Testing Yes/No?

  Serial.begin(115200);                      // Init Serial with 115200 for MiSTer ttyUSBx Device CP2102 Chip
  Serial.flush();                            // Wait for empty Send Buffer
  Serial.setTimeout(500);                    // Set max. Serial "Waiting Time", default = 1000ms

  randomSeed(analogRead(34));                // Init Random Generator with empty Port Analog value
  
// Init ST7789 display 240x240 pixel
oled.init(240, 240, SPI_MODE0);
oled_cleardisplay();
oled.setCursor(0, 0);
oled.setTextColor(ST77XX_WHITE);
oled.setTextWrap(true);

// Get Display Dimensions
/*DispWidth = oled.width();
DispHeight = oled.height();
DispLineBytes1bpp = DispWidth / 8;                       // How many Bytes uses each Display Line at 1bpp (32 byte for width 256 Pixel)
DispLineBytes4bpp = DispWidth / 2;                       // How many Bytes uses each Display Line at 4bpp (128 byte for width 256 Pixel)
logoBytes1bpp = DispWidth * DispHeight / 8;              // SSD1322 = 2048 Bytes
logoBytes4bpp = DispWidth * DispHeight / 2;              // SSD1322 = 8192 Bytes
//logoBin = (uint8_t *) malloc(logoBytes4bpp);             // Create Picture Buffer, better than permanent create (malloc) and destroy (free)
*/
//TODO: do better...
DispWidth = 256;
DispHeight = 64;
DispLineBytes1bpp = DispWidth / 8;
DispLineBytes4bpp = DispWidth / 2;
logoBytes1bpp = 2048;
logoBytes4bpp = 8192;

#ifdef XDEBUG
  Serial.printf("setup: DispWidth=%i DispHeight=%i\n", DispWidth, DispHeight);
#endif  // XDEBUG


// Go...
  oled_showStartScreen();                                  // OLED Startup

  delay(cDelay);                                           // Command Response Delay
  Serial.print(F("ttyrdy;"));                                 // Send "ttyrdy;" after setup is done.
  Serial.print(logoBytes4bpp);

}

// =============================================================================================================
// =============================================== MAIN LOOP ===================================================
// =============================================================================================================
void loop(void) {
  unsigned long currentMillis = millis();

  // Get Serial Data
  if (Serial.available()) {
  	prevCommand = newCommand;                              // Save old Command
    newCommand = Serial.readStringUntil('\n');             // Read string from serial until NewLine "\n" (from MiSTer's echo command) is detected or timeout (1000ms) happens.
    updateDisplay=true;                                    // Set Update-Display Flag

#ifdef XDEBUG
    Serial.printf("\nReceived Corename or Command: %s\n", (char*)newCommand.c_str());
#endif
  }  // end serial available
    
  if (updateDisplay) {                                                                                 // Proceed only if it's allowed because of new data from serial
    if (startScreenActive && newCommand.startsWith("CMD") && !newCommand.startsWith("CMDTZONE")) {     // If any Command is processed the StartScreen isn't shown any more
      startScreenActive=false;                                                                         // This variable should prevent "side effects" with Commands and is used to disable automatic drawings
    }

    if (newCommand.endsWith("QWERTZ")) {                         // Process first Transmission after PowerOn/Reboot.
        // Do nothing, just receive one string to clear the buffer.
    }                    

    // ---------------------------------------------------
    // ---------------- C O M M A N D 's -----------------
    // ---------------------------------------------------

    // -- Test Commands --
    else if (newCommand=="CMDNULL") {                                    // NULL-Command, RunTime Test Command
        // Do nothing
    }

    else if (newCommand=="cls") {                                        // Clear Screen
      oled_cleardisplay();
      oled_display();
    }
    
    else if (newCommand=="sorg") {                                       // Start Screen
      oled_showStartScreen();
    }
    
    else if (newCommand=="bye")  {                                        // Cat Screen
      oled_drawlogo64h(sorgelig_icon64_width, sorgelig_icon64);
    }
    
    else if (newCommand=="CMDCLS") {                                        // Clear Screen with Display Update
      oled_cleardisplay();
      oled_display();
    }
    
    else if (newCommand=="CMDCLSWU") {                                      // Clear Screen without Display Update
      oled_cleardisplay();
    }

    else if (newCommand.startsWith("CMDCLST")) {                            // Clear/Fill Screen with Transition and Color
      oled_clswithtransition();
    }
    
    else if (newCommand=="CMDSORG") {                                       // Show Startscreen
      oled_showStartScreen();
    }
    
    else if (newCommand=="CMDBYE") {                                        // Show Sorgelig's Icon
      oled_drawlogo64h(sorgelig_icon64_width, sorgelig_icon64);
    }

    else if (newCommand.startsWith("CMDSECD")) {                            // Set Command Delay
      oled_setcdelay();
    }

    else if (newCommand=="CMDSHCD") {                                       // Show Command Delay
      oled_showcdelay();
    }

    else if (newCommand=="CMDSHSYSHW") {                                    // Show System HW
      oled_showSystemHardware();
    }

    else if (newCommand=="CMDHWINF") {                                      // Send HW Info
      oled_sendHardwareInfo();
    }

    else if (newCommand=="CMDTEST") {                                       // Show Test-Picture
      oled_printtext("CMDTEST OK");
    }

    else if (newCommand=="CMDSNAM") {                                       // Show actual loaded Corename
      oled_showcorename();
    }

    else if (newCommand.startsWith("CMDSPIC")) {                            // Show actual loaded Picture with(without Transition
      oled_showpic();
      if (tEffect==-1) {                                                    // Send without Effect Parameter or Parameter = -1
        oled_drawlogo(random(minEffect,maxEffect+1));                       // ...and show them on the OLED with Transition Effect 1..MaxEffect
      } 
      else {                                                                // Send with Effect "CMDSPIC,15"
        oled_drawlogo(tEffect);
      }
    }

    else if (newCommand=="CMDSSCP") {                                       // Show actual loaded Core Picture but in 1/4 size
      oled_showSmallCorePicture(64,16);
    }

    else if (newCommand=="CMDDOFF") {                                       // Switch Display Off
      oled_displayoff();
    }

    else if (newCommand=="CMDDON") {                                        // Switch Display On
      oled_displayon();
    }

    else if (newCommand=="CMDDUPD") {                                       // Update Display Content
      oled_updatedisplay();
    }

    else if (newCommand.startsWith("CMDTXT")) {                             // Command from Serial to write Text
      oled_readnwritetext();                                                // Read and Write Text
    }
    
    else if (newCommand.startsWith("CMDGEO")) {                             // Command from Serial to draw geometrics
      oled_readndrawgeo();                                                  // Read and Draw Geometrics
    }
/*
    else if (newCommand.startsWith("CMDAPD")) {                             // Command from Serial to receive Picture Data via USB Serial from the MiSTer
      oled_readlogo();                                                      // Receive Picture Data... 
    }
*/
    else if (newCommand.startsWith("CMDCOR")) {                             // Command from Serial to receive Picture Data via USB Serial from the MiSTer
      oled_read_and_draw_logo();
    }

    else if (newCommand.startsWith("CMDCON")) {                            // Command from Serial to receive Contrast-Level Data from the MiSTer
      oled_readnsetcontrast();                                             // Read and Set contrast                                   
    }

// ---------------------------------------------------
// -- Unidentified Core Name, just write it on screen
// ---------------------------------------------------
    else {
      actCorename=newCommand;
      actPicType=NONE;
      oled_showcorename();
    }  // end ifs

    if (sendTTYACK) {                                 // Send ACK?
      delay(cDelay);                                    // Command Response Delay
      Serial.print(F("ttyack;"));                        // Handshake with delimiter; MiSTer: "read -d ";" ttyresponse < ${TTYDEVICE}"
    }
    // Serial.flush();                                // Wait for sendbuffer is clear

    updateDisplay=false;                              // Clear Update-Display Flag
  } // endif updateDisplay

// ---------------------------------------------------
} // End Main Loop

// =============================================================================================================
// ============================================== Functions ====================================================
// =============================================================================================================

// --------------------------------------------------------------
// -------------------- Show Start-Up Text ----------------------
// --------------------------------------------------------------
void oled_showStartScreen(void) {
  uint8_t color = 0;

#ifdef XDEBUG
  Serial.println(F("Show Startscreen"));
#endif
  oled_cleardisplay();
  oled.drawXBitmap(82, 0, tty2oled_logo, tty2oled_logo_width, tty2oled_logo_height, OLED_WHITE);
  oled_display();
  delay(1000);
  for (int i=0; i<DispWidth; i+=16) {            // Some Animation
    oled.fillRect(i,55,16,8,color);
    color++;
    oled_display();
    delay(20);
  }
  for (int i=0; i<DispWidth; i+=16) {            // Remove Animation Line
    oled.fillRect(i,55,16,8,OLED_BLACK);
    oled_display();
    delay(20);
  }
  delay(500);
  oled_setfont(0);
  oled_setcursor(0,63);
  oled_printtext(BuildVersion);
  if (runsTesting) {
    oled.drawXBitmap(DispWidth-usb_icon_width, DispHeight-usb_icon_height, usb_icon, usb_icon_width, usb_icon_height, OLED_WHITE);
  }

  oled_display();
  startScreenActive=true;
} // end mistertext


// --------------------------------------------------------------
// ------------------ Enable/Disable ttyack ---------------------
// --------------------------------------------------------------
void oled_setttyack(void) {
  String xT="";
  int x;
#ifdef XDEBUG
  Serial.println(F("Called Command CMDSTTYACK"));
#endif
  xT=newCommand.substring(newCommand.indexOf(',')+1);
#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)xT.c_str());
#endif
  
  x=xT.toInt();                                 // Convert Value
  if (x<0) x=0;
  if (x>1) x=1;

  if (x) {
    sendTTYACK = true;
  }
  else {
    sendTTYACK = false;
  }
}

// --------------------------------------------------------------
// ------------ Read and Set Command Delay ----------------------
// --------------------------------------------------------------
void oled_setcdelay(void) {
  String dT="";
#ifdef XDEBUG
  Serial.println(F("Called Command CMDSECD"));
#endif
  
  dT=newCommand.substring(8);           // CMD-Length+1

#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)dT.c_str());
#endif
  cDelay=dT.toInt();                   // Convert Value
#ifdef XDEBUG
  Serial.printf("\nSet cDelay to: %d\n", cDelay);
#endif
}


// --------------------------------------------------------------
// ------------- Show Command Delay on Screen -------------------
// --------------------------------------------------------------
void oled_showcdelay(void) {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDSHCD"));
#endif
  
  oled_cleardisplay();
  oled_setfont(7);
  oled_setcursor(20,32);
  oled_printtext("cDelay: ");
  oled_printtext(cDelay);
  oled_display();
}

// --------------------------------------------------------------
// -------------------- Clear screen ----------------------
// --------------------------------------------------------------
void oled_cleardisplay(void) {
  oled.fillScreen(ST77XX_BLACK);
}

// --------------------------------------------------------------
// -------------------- Display screen ----------------------
// --------------------------------------------------------------
void oled_display(void) {
}

// --------------------------------------------------------------
// ------------- Show 1/4 Core Picture at Position V2 -----------
// --------------------------------------------------------------
// xpos & ypos = Offset
void oled_showSmallCorePicture(int xpos, int ypos) {
  int x=0,y=0,px=0,py=0,i=0;
  unsigned char b1,b2,br;

#ifdef XDEBUG
  Serial.printf("Show 1/4 Pic, ActPicType: %d\n",actPicType);
#endif
  
  oled_cleardisplay();
  switch (actPicType) {
    case XBM:
      x=0;y=0;
      for (py=0; py<DispHeight; py=py+2) {
        for (px=0; px<DispLineBytes1bpp; px++) {
          b1=logoBin[px+py*DispLineBytes1bpp];                       // Get Data Byte for 8 Pixels
          for (i=0; i<8; i=i+2){
            if (bitRead(b1, i)) {
              oled.drawPixel(xpos+x, ypos+y, OLED_WHITE);         // Draw Pixel if "1"
            }
            else {
              oled.drawPixel(xpos+x, ypos+y, OLED_BLACK);         // Clear Pixel if "0"
            }
#ifdef XDEBUG
            Serial.printf("X: %d Y: %d\n",x,y);
#endif
            x++;
          }

        }
        x=0;
        y++;
      }
      oled_display();  
    break;
    case GSC:
      x=0;y=0;
      for (py=0; py<DispHeight; py=py+2) {
        for (px=0; px<DispLineBytes1bpp; px++) {
          for (i=0; i<4; i++) {
            b1=logoBin[(px*4)+i+py*DispLineBytes4bpp];                                                  // Get Data Byte 1 for 2 Pixels
            b2=logoBin[(px*4)+i+(py+1)*DispLineBytes4bpp];                                              // Get Data Byte 2 for 2 Pixels
            //br=(((0xF0 & b1) >> 4) + (0x0F & b1) + ((0xF0 & b2) >> 4) + (0x0F & b2)) / 4;               // cutting
            br=round((((0xF0 & b1) >> 4) + (0x0F & b1) + ((0xF0 & b2) >> 4) + (0x0F & b2)) / 4);        // rounding
            oled.drawPixel(xpos+x, ypos+y, br);   // Draw only Pixel 1, Left Nibble
            //Serial.printf("X: %d Y: %d\n",x,y);
            x++;
          }

        }
      x=0;
      y++;
      }
      oled_display();  
    break;
    case NONE:
      oled_showcorename();
    break;
  }
}


// --------------------------------------------------------------
// ----------- Show System Hardware Info on Screen --------------
// --------------------------------------------------------------
void oled_showSystemHardware(void) {
  int hwinfo=0;

#ifdef XDEBUG
  Serial.println(F("Called Command CMDSHSYSHW"));
#endif

#ifdef USE_LOLIN32                         // Wemos LOLIN32, LOLIN32, DevKit_V4 (Wemos Lolin32)
  hwinfo=2;
#endif

  oled_cleardisplay();
  oled_setfont(2);
  
  oled_setcursor(0,10);
  oled_printtext("SysInfo");

  oled_setfont(1);
  oled_setcursor(0,25);
  oled_printtext("FW Version: " BuildVersion);

  oled_setcursor(0,35);
  oled_printtext("Board Type: ");
  
  switch (hwinfo) {
    case 0:
      oled_printtext("Unknown");                 // Unknown Hardware
    break;
    case 2:
      oled_printtext("WEMOS LOLIN32");           // Lolin Arduino Nano
    break;
    default:
      oled_printtext("Other");                   // Everything else
    break;
  }

  oled_setcursor(0,50);
  oled_printtext("Board Options: ");
  oled_setcursor(0,60);
  oled_display();  
}

// --------------------------------------------------------------
// ----------- Send Hardware Info Back to the MiSTer ------------
// --------------------------------------------------------------
void oled_sendHardwareInfo(void) {
  int hwinfo=0;

#ifdef USE_LOLIN32                         // Wemos LOLIN32, LOLIN32, DevKit_V4 (Wemos Lolin32)
  hwinfo=2;
#endif

  delay(hwDelay);                          // Small Delay

  switch (hwinfo) {
    case 0:
      Serial.println(F("HWNONEXXX;" BuildVersion ";"));              // No known Hardware in use
    break;
    case 1:
      Serial.println(F("HWESP32DE;" BuildVersion ";"));              // ESP32-DEV, TTGO, DTI-Board
    break;
    case 2:
      Serial.println(F("HWLOLIN32;" BuildVersion ";"));              // Wemos,Lolin,DevKit_V4
    break;
    case 3:
      Serial.println(F("HWESP8266;" BuildVersion ";"));              // ESP8266
    break;
    case 4:
      Serial.println(F("HWESP32S3;" BuildVersion ";"));              // ESP32-S3
    break;
    default:
      Serial.println(F("HWNONEXXX;" BuildVersion ";"));              // Default
    break;
  }
}


// --------------------------------------------------------------
// ---- Draw Pictures with an height of 64 Pixel centered -------
// --------------------------------------------------------------
void oled_drawlogo64h(uint16_t w, const uint8_t *bitmap) {
  oled_cleardisplay();
  oled.drawXBitmap(DispWidth/2-w/2, 0, bitmap, w, DispHeight, OLED_WHITE);
  oled_display();
} // end oled_drawlogo64h


// --------------------------------------------------------------
// ----------------- Just show the Corename ---------------------
// --------------------------------------------------------------
void oled_showcorename() {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDSNAM"));
#endif

  oled_setcontrast(contrast);
  oled_showcenterredtext(actCorename,9);
}


// --------------------------------------------------------------
// ------------------ Switch Display off ------------------------
// --------------------------------------------------------------
void oled_displayoff(void) {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDDOFF"));
#endif

}


// --------------------------------------------------------------
// ------------------- Switch Display on ------------------------
// --------------------------------------------------------------
void oled_displayon(void) {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDDOFF"));
#endif
  
}


// --------------------------------------------------------------
// -------------- Update Display Content ------------------------
// --------------------------------------------------------------
void oled_updatedisplay(void) {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDDUPD"));
#endif

  oled_display();                 // Update Display Content
}


// --------------------------------------------------------------
// ----------------- Read an Set Contrast -----------------------
// --------------------------------------------------------------
void oled_readnsetcontrast(void) {
  String cT="";
#ifdef XDEBUG
  Serial.println(F("Called Command CMDCON"));
#endif
  
  cT=newCommand.substring(7);

#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)cT.c_str());
#endif
  contrast=cT.toInt();                   // Convert Value
  oled_setcontrast(contrast);            // Read and Set contrast  
}

// --------------------------------------------------------------
// ---------------- Show "Parameter Error" ----------------------
// --------------------------------------------------------------
void oled_showperror(void) {
  String actText="Parameter Error";
  
  oled_cleardisplay();
  oled_setfont(3);
  oled_setcursor(DispWidth/2-(oled_getUTF8Width(actText.c_str())/2), 20);
  //oled_setcursor(5, 20);
  oled_printtext(actText);
  
  oled_setfont(2);
  oled_setcursor(DispWidth/2-(oled_getUTF8Width(newCommand.c_str())/2), 40);
  //oled_setcursor(5, 40);
  oled_printtext(newCommand);
  oled_display();
}

// --------------------------------------------------------------
// ------------------ Show centered Text ------------------------
// --------------------------------------------------------------
void oled_showcenterredtext(String text, int font) {
  
  oled_cleardisplay();
  oled_setfont(font);
  oled_setcursor(DispWidth/2-(oled_getUTF8Width(text.c_str())/2), DispHeight/2 + (oled_getFontAscent()/2));
  oled_printtext(text);
  oled_display();
}


// --------------------------------------------------------------
// -------------------------- Set Font --------------------------
// --------------------------------------------------------------
void oled_setfont(int font) {

}


// --------------------------------------------------------------
// -------------------------- Set Cursor ------------------------
// --------------------------------------------------------------
void oled_setcursor(int16_t x, int16_t y) {

  oled.setCursor(x, y);

}

// --------------------------------------------------------------
// -------------------------- Set Text Color --------------------
// --------------------------------------------------------------
// for fg and for bg colors, give value -1 to not set the color
void oled_settextcolor(int16_t fg, int16_t bg) {

  oled.setTextColor(fg, bg);

}

// --------------------------------------------------------------
// -------------------------- Set Contrast ----------------------
// --------------------------------------------------------------
void oled_setcontrast(uint8_t c) {
}

// --------------------------------------------------------------
// -------------------------- Print text on screen --------------
// --------------------------------------------------------------
void oled_printtext(String val) {

  oled.print(val);

}

void oled_printtext(byte val) {

  oled.print(val);

}

void oled_printtext(int val) {

  oled.print(val);

}

// --------------------------------------------------------------
// -------------------------- Printf text on screen -------------
// --------------------------------------------------------------
void oled_printftext(String s, float f) {

//TODO do better...
  oled.print(s);
  oled.print(f);

}

void oled_printftext(String s, int i) {

//TODO do better...
  oled.print(s);
  oled.print(i);

}

// --------------------------------------------------------------
// -------------------------- Get a string's width --------------
// --------------------------------------------------------------
int16_t oled_getUTF8Width(const char * s) {

//TODO do better...
  return 0;

}

// --------------------------------------------------------------------------------------------------
// -------------------------- Get the height of the character 'A' in the current font ---------------
// --------------------------------------------------------------------------------------------------
int8_t oled_getFontAscent(void) {

//TODO do better...
  return 0;

}

// -----------------------------------------------------------------------
// -------------------------- Draw a 4-Bit Grayscale Picture--------------
// -----------------------------------------------------------------------
void oled_drawgreyscale(const uint8_t bitmap[]) {

//TODO do a grayscale drawing...
  oled.drawXBitmap(0, 0, bitmap, DispWidth, DispHeight, OLED_WHITE);

}

// --------------------------------------------------------------
// --------------- Clear Display with Transition ----------------
// --------------------------------------------------------------
void oled_clswithtransition() {
  String TextIn,tT="",cT="";
  //uint16_t w,t=0,c=0,d1=0;
  int w,t=0,c=0,d1=0;
  bool pError=false;

#ifdef XDEBUG
  Serial.println(F("Called Command CMDCLST"));
#endif

  TextIn = newCommand.substring(8);         // Get Command Text from "newCommand"

  //Searching for the "," delimiter
  d1 = TextIn.indexOf(',');                 // Find location of first ","

  //Create Substrings
  tT = TextIn.substring(0, d1);             // Get String for Transition
  cT = TextIn.substring(d1+1);              // Get String for Draw Colour
  
#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)TextIn.c_str());
  Serial.printf("Created Strings: T:%s C:%s\n", (char*)tT.c_str(), (char*)cT.c_str());
#endif

  // Enough Parameter given / Parameter Check
  if (d1==-1) {
    pError=true;
  }

  // Convert Strings to Integer
  t = tT.toInt();
  c = cT.toInt();

#ifdef XDEBUG
  Serial.printf("Values: T:%i C:%i\n", t,c);
#endif

  // Parameter check
  if (t<-1) t=-1;
  if (t>maxEffect) t=maxEffect;
  if (c<0) c=0;
  if (c>15) c=15;
  
  if (t==-1) {
    t=random(minEffect,maxEffect+1);
  }

  actPicType=GSC;                        // Set Picture Type to GSC to enable Color Mode
  actCorename="No Core";
  /*for (w=0; w<logoBytes4bpp; w++) {
    logoBin[w]=(c << 4) | c;             // Fill Picture with given Color..
  }*/
  oled.fillScreen(ST77XX_BLACK);

  if (!pError) {
    oled_drawlogo(t);                // ..and draw Picture to Display with Effect
  }
  else {
    oled_showperror();
  }
}  // end oled_clswithtransition


// --------------------------------------------------------------
// ------------------------- Showpic ----------------------------
// --------------------------------------------------------------
void oled_showpic(void) {
#ifdef XDEBUG
  Serial.println(F("Called Command CMDSPIC"));
#endif

  oled_setcontrast(contrast);

  if (newCommand.length()>7) {                       // Parameter added?
    tEffect=newCommand.substring(8).toInt();         // Get Effect from Command String (is set to 0 if not convertable)
    if (tEffect<-1) tEffect=-1;                      // Check Effect minimum
    if (tEffect>maxEffect) tEffect=maxEffect;        // Check Effect maximum
  }
  else {
    tEffect=-1;                                      // Set Parameter to -1 (random)
  }
}

// --------------------------------------------------------------
// ----------------------- Read Logo ----------------------------
// --------------------------------------------------------------
int oled_readlogo() {
  String TextIn="",tT="";
  int d1=0;
  
#ifdef XDEBUG
  Serial.println(F("Called Command CMDCOR"));
#endif

  TextIn=newCommand.substring(7);                    // Get Command String
  d1 = TextIn.indexOf(',');                          // Search String for ","
  if (d1==-1) {                                      // No "," found = no Effect Parameter given
    actCorename=TextIn;                              // Get Corename
    tEffect=-1;                                      // Set Effect to -1 (Random)
#ifdef XDEBUG
    Serial.printf("\nReceived Text: %s, Transition T:None\n", (char*)actCorename.c_str());
#endif
  }
  else {                                             // "," found = Effect Parameter given
    actCorename=TextIn.substring(0, d1);             // Extract Corename from Command String
    tEffect=TextIn.substring(d1+1).toInt();          // Get Effect from Command String (set to 0 if not convertable)
    if (tEffect<-1) tEffect=-1;                      // Check Effect minimum
    if (tEffect>maxEffect) tEffect=maxEffect;        // Check Effect maximum
#ifdef XDEBUG
    Serial.printf("\nReceived Text: %s, Transition T:%i \n", (char*)actCorename.c_str(),tEffect);
#endif
  }

  //logoBin = (uint8_t *) malloc(logoBytes4bpp);             // Create Picture Buffer, better than permanent create (malloc) and destroy (free)
  // 2022-01-05 Add Cast Operator (char*) to the command
  bytesReadCount = Serial.readBytes((char*)logoBin, logoBytes4bpp);  // Read 2048 or 8192 Bytes from Serial

  // Set the Actual Picture Type
  if (bytesReadCount == 2048) actPicType = XBM;
  else if (bytesReadCount == 8192) actPicType = GSC;
  else actPicType = 0;

#ifdef XDEBUG
    Serial.printf("\nactPicType: %d, bytesReadCount T:%i \n", actPicType, bytesReadCount);
#endif

#ifdef XDEBUG
  oled_cleardisplay();
  oled.setCursor(0,0);
  oled.print(bytesReadCount);
  oled_display();
  delay(1000);
#endif

  // Check if 2048 or 8192 Bytes read
  if (((int)bytesReadCount != logoBytes1bpp) && ((int)bytesReadCount != logoBytes4bpp)) {
    oled_printtext("Transfer error");
    oled.setCursor(0,0);
    oled.print(bytesReadCount);
    oled_display();
    //free(logoBin);
    return 0;
  }
  else {
    free(logoBin);
    return 1;
  }
}  //end oled_readlogo


// --------------------------------------------------------------
// ----------------------- Draw Logo ----------------------------
// --------------------------------------------------------------
void oled_drawlogo(uint8_t e) {
  int w,x,y,x2=0,y2=0;
  //unsigned char logoByteValue;
  //int logoByte;
  uint8_t vArray[DispLineBytes1bpp]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
  
#ifdef XDEBUG
  Serial.printf("Draw Logo with Transition Effect No. %d\n",e);
#endif

  oled_setcontrast(contrast);

  switch (e) {
    case 1:                                  // Left to Right
      for (x=0; x<DispLineBytes1bpp; x++) {
        for (y=0; y<DispHeight; y++) {
          oled_drawEightPixelXY(x, y);
        }
        oled_display();
      }
    break;  // 1

    case 2:                                  // Top to Bottom
      for (y=0; y<DispHeight; y++) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          oled_drawEightPixelXY(x, y);
        }
        if (y%2==1) oled_display();
      }
    break;  // 2

    case 3:                                  // Right to left
      for (x=DispLineBytes1bpp-1; x>=0; x--) {
        for (y=0; y<DispHeight; y++) {
          oled_drawEightPixelXY(x, y);
        }
        oled_display();
      }
    break;  // 3

    case 4:                                  // Bottom to Top
      for (y=DispHeight-1; y>=0; y--) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          oled_drawEightPixelXY(x, y);
        }
        if (y%2==0) oled_display();
      }
    break;  // 4

    case 5:                                  // Even Line Left to Right / Odd Line Right to Left
      for (x=0; x<DispLineBytes1bpp; x++) {
        for (y=0; y<DispHeight; y++) {
          if ((y % 2) == 0) {
            x2 = x;
          }
          else {
            x2 = x*-1 + DispLineBytes1bpp -1;
          }
          oled_drawEightPixelXY(x2, y);
        }  // end for y
        oled_display();
      }  // end for x
    break;  // 5

    case 6:                                     // Top Part Left to Right / Bottom Part Right to Left
      for (x=0; x<DispLineBytes1bpp; x++) {
        for (y=0; y<DispHeight; y++) {
          if (y < DispHeight/2) {
            x2 = x;
          }
          else {
            x2 = x*-1 + DispLineBytes1bpp -1;
          }
          oled_drawEightPixelXY(x2, y);
        }  // end for y
        oled_display();
      }  // end for x
    break;  // 6

    case 7:                                     // Four Parts Left to Right to Left to Right...
      for (w=0; w<4; w++) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          for (y=0; y<DispHeight/4; y++) {
            if (w%2==0) {
              x2 = x;
            }
            else {
              x2 = x*-1 + DispLineBytes1bpp -1;
            }
            oled_drawEightPixelXY(x2, y+w*16);
          }  // end for y
          oled_display();
        }  // end for x
      }
    break;  // 7

    case 8:                                     // 4 Parts, Top-Left => Bottom-Right => Top-Right => Bottom-Left
      // Part 1 Top Left
      for (x=0; x<DispLineBytes1bpp/2; x++) {
        for (y=0; y<DispHeight/2; y++) {
          oled_drawEightPixelXY(x, y);
        }  // end for y
        oled_display();
      }  // end for x
      // Part 2 Bottom Right
      for (x=DispLineBytes1bpp/2; x<DispLineBytes1bpp; x++) {
        for (y=DispHeight/2; y<DispHeight; y++) {
          oled_drawEightPixelXY(x, y);
        }  // end for y
        oled_display();
      }  // end for x
      // Part 3 Top Right
      for (x=DispLineBytes1bpp-1; x>=DispLineBytes1bpp/2; x--) {
        for (y=0; y<DispHeight/2; y++) {
          oled_drawEightPixelXY(x, y);
        }  // end for y
        oled_display();
      }  // end for x
      // Part 4 Bottom Left
      for (x=DispLineBytes1bpp/2-1; x>=0; x--) {
        for (y=DispHeight/2; y<DispHeight; y++) {
          oled_drawEightPixelXY(x, y);
        }  // end for y
        oled_display();
      }  // end for x
    break; // 8
    
    case 9:                                      // Particle Effect NEW and faster
      for (w=0; w<1000; w++) {
        x = random(DispLineBytes1bpp);
        y = random(DispHeight/8);
        for (int offset=0; offset<8; offset++) {
          oled_drawEightPixelXY(x, y*8+offset);
        }
        // Different speed
        if (w<=250) {
          if ((w % 5)==0) oled_display();
        }
        if (w>250 && w<=500) {
          if ((w % 10)==0) oled_display();
        }
        if (w>500) {
          if ((w % 20)==0) oled_display();
        }
      }
      // Finally overwrite the Screen with full Size Picture
      oled_cleardisplay();
      if (actPicType==XBM) oled.drawXBitmap(0, 0, logoBin, DispWidth, DispHeight, OLED_WHITE);
      if (actPicType==GSC) oled_drawgreyscale(logoBin);
      oled_display();
    break;  // 9

    case 10:                                       // Left to Right Diagonally
      for (x=0; x<DispLineBytes1bpp+DispHeight; x++) {
        for (y=0; y<DispHeight; y++) {
          // x2 calculation = Angle
          //x2=x-y;                                // Long Diagonal
          //x2=x-y/2;                              // Middle Diagonal
          x2=x-y/4;                                // Short Diagonal
          if ((x2>=0) && (x2<DispLineBytes1bpp)) {
            oled_drawEightPixelXY(x2, y);
          }  // end for x2
        }  // end for y
        oled_display();
      }  // end for x
    break;  // 10
    
    case 11:                                      // Slide in left to right
      for (x=0; x<DispLineBytes1bpp; x++) {
        for (x2=DispLineBytes1bpp-1-x; x2<DispLineBytes1bpp; x2++) {
          for (y=0; y<DispHeight; y++) {
            oled_drawEightPixelXY(x+x2-(DispLineBytes1bpp-1), y, x2, y);
          }
        }
        oled_display();
      }
    break;  // 11

    case 12:                                     // Slide in Top to Bottom
      for (y=0; y<DispHeight; y++) {
        for (y2=DispHeight-1-y; y2<DispHeight; y2++) {
          for (x=0; x<DispLineBytes1bpp; x++) {
            oled_drawEightPixelXY(x, y+y2-(DispHeight-1), x, y2);
          }
        }
        if (y%2==1) oled_display();
      }
    break;  // 12

    case 13:                                  // Slide in Right to left
      for (x=DispLineBytes1bpp-1; x>=0; x--) {
        for (x2=DispLineBytes1bpp-1-x; x2>=0; x2--) {
          for (y=0; y<DispHeight; y++) {
            oled_drawEightPixelXY(x+x2, y, x2, y);
          }
        }
        oled_display();
      }
    break;  // 13

    case 14:                                  // Slide in Bottom to Top
      for (y=DispHeight-1; y>=0; y--) {
        for (y2=DispHeight-1-y; y2>=0; y2--) {
          for (x=0; x<DispLineBytes1bpp; x++) {
            oled_drawEightPixelXY(x, y+y2, x, y2); 
          }
        }
        if (y%2==0) oled_display();
      }
    break;  // 14

    case 15:                                  // Top and Bottom to Middle
      for (y=0; y<DispHeight/2; y++) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          oled_drawEightPixelXY(x, y);
          oled_drawEightPixelXY(x, DispHeight-y-1);
        }
      oled_display();
      }
    break;  // 15

    case 16:                                  // Left and Right to Middle
      for (x=0; x<DispLineBytes1bpp/2; x++) {
        for (y=0; y<DispHeight; y++) {
          oled_drawEightPixelXY(x, y);
          oled_drawEightPixelXY(DispLineBytes1bpp-x-1, y);
        }
        oled_display();
      }
    break;  // 16

    case 17:                                  // Middle to Top and Bottom
      for (y=0; y<DispHeight/2; y++) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          oled_drawEightPixelXY(x, DispHeight/2-1-y);
          oled_drawEightPixelXY(x, DispHeight/2+y);
        }
      oled_display();
      }
    break;  // 17

    case 18:                                  // Middle to Left and Right
      for (x=0; x<DispLineBytes1bpp/2; x++) {
        for (y=0; y<DispHeight; y++) {
          oled_drawEightPixelXY(DispLineBytes1bpp/2-1-x, y);
          oled_drawEightPixelXY(DispLineBytes1bpp/2+x, y);
        }
        oled_display();
      }
    break;  // 18
    
    case 19:                                  // Warp, Middle to Left, Right, Top and Bottom
      for (w=0; w<DispLineBytes1bpp/2; w++) {
        for (y=DispHeight/2-2-(w*2); y<=DispHeight/2+(w*2)+1; y++) {
          for (x=DispLineBytes1bpp/2-1-w; x<=DispLineBytes1bpp/2+w; x++) {
            oled_drawEightPixelXY(x, y);
          }
        }
        oled_display();
      }
    break;  // 19

    case 20:                                  // Slightly Clockwise
      for (y=0; y<DispHeight/2;y++) {
        for (x=DispLineBytes1bpp-DispHeight/16;x<DispLineBytes1bpp;x++) {
          oled_drawEightPixelXY(x, y);
        }
      //oled_display();  
      if (y%2==1) oled_display();             // Update only each uneven (second) round = faster 
      }
      for (x=DispLineBytes1bpp-1;x>=DispHeight/16;x--) {
        for (y=DispHeight/2; y<DispHeight;y++) {
          oled_drawEightPixelXY(x, y);
        }
      oled_display();  
      }
      for (y=DispHeight-1; y>=DispHeight/2;y--) {
        for (x=0;x<DispHeight/16;x++) {
          oled_drawEightPixelXY(x, y);
        }
      //oled_display();  
      if (y%2==0) oled_display();             // Update only each even (second) round = faster 
      }
      for (x=0;x<DispLineBytes1bpp-DispHeight/16;x++) {
        for (y=0; y<DispHeight/2;y++) {
          oled_drawEightPixelXY(x, y);
        }
      oled_display();  
      }
    break;  // 20

    case 21:                                  // Shaft
      for (y=0; y<DispHeight; y++) {
        for (x=0; x<DispLineBytes1bpp; x++) {
          if ((x>=0 && x<DispLineBytes1bpp/4*1) || (x>=DispLineBytes1bpp/2 && x<DispLineBytes1bpp/4*3)) oled_drawEightPixelXY(x, y);
          if ((x>=DispLineBytes1bpp/4*1 && x<DispLineBytes1bpp/2) || (x>=DispLineBytes1bpp/4*3 && x<DispLineBytes1bpp)) oled_drawEightPixelXY(x, DispHeight-y-1);
        }
        //oled_display();
        if (y%2==1) oled_display();
      }
    break;
    
    case 22:                                  // Waterfall
      for (w=0; w<DispLineBytes1bpp; w++) {   // Shuffle the Array
        x2=random(DispLineBytes1bpp);
        x=vArray[w];
        vArray[w]=vArray[x2];
        vArray[x2]=x;
      }
      for (x=0; x<DispLineBytes1bpp/2;x++) {
        for (y=0; y<DispHeight; y++) {
          oled_drawEightPixelXY(vArray[x*2], y);
          oled_drawEightPixelXY(vArray[x*2+1], y);
          //if (y%8==7) oled_display();       // Waterfall Speed
          if (y%16==15) oled_display();       // Waterfall Speed
        }
      }
    break;  // 22

    case 23:                                  // Chess Field with 8 squares
      for (w=0; w<DispLineBytes1bpp/4; w++) { // Shuffle the Array
        x2=random(DispLineBytes1bpp/4);
        x=vArray[w];
        vArray[w]=vArray[x2];
        vArray[x2]=x;
      }
      for (w=0; w<DispLineBytes1bpp/4; w++) { // Set x,y for the fieled
        switch(vArray[w]) {
          case 0:
            x2=0; y2=0;
          break;
          case 1:
            x2=8; y2=0;
          break;
          case 2:
            x2=16; y2=0;
          break;
          case 3:
            x2=24; y2=0;
          break;
          case 4:
            x2=0; y2=32;
          break;
          case 5:
            x2=8; y2=32;
          break;
          case 6:
            x2=16; y2=32;
          break;
          case 7:
            x2=24; y2=32;
          break;
        }
        for (x=x2;x<x2+DispLineBytes1bpp/4;x++) {
          for (y=y2;y<y2+DispHeight/2;y++) {
            oled_drawEightPixelXY(x, y);
          }
        }
        oled_display();
        delay(100);                           // Delay
      }
    break;  // 23

    default:
      if (actPicType == XBM) {
#ifdef XDEBUG
        oled_cleardisplay();
        oled.setCursor(0,0);
        oled.print("drawXBitmap");
        oled_display();
        delay(1000);
#endif
        oled_cleardisplay();
        oled.drawXBitmap(0, 0, logoBin, DispWidth, DispHeight, OLED_WHITE);
        oled_display();
      }
      if (actPicType == GSC) {
#ifdef XDEBUG
        oled_cleardisplay();
        oled.setCursor(0,0);
        oled.print("draw4bppBitmap");
        oled_display();
        delay(1000);
#endif
        oled_cleardisplay();
        oled_drawgreyscale(logoBin);
        oled_display();
      }    
    break;
  } // end switch (e)
}  // end oled_drawlogo


// ----------------------------------------------------------------
// ----------------------- Read and Draw Logo (streaming) ---------
// ----------------------------------------------------------------
void oled_read_and_draw_logo() {

/*
      if (oled_readlogo()==1) {                                             // Receive Picture Data... 
        if (tEffect==-1) {                                                  // Send without Effect Parameter or with Effect Parameter -1
          oled_drawlogo(random(minEffect,maxEffect+1));                     // ...and show them on the OLED with Transition Effect 1..MaxEffect
        } 
        else {                                                              // Send with Effect "CMDCOR,llander,15"
          oled_drawlogo(tEffect);
        }
      }
*/

/*
      if (actPicType == XBM) {
#ifdef XDEBUG
        oled_cleardisplay();
        oled.setCursor(0,0);
        oled.print("drawXBitmap");
        oled_display();
        delay(1000);
#endif
        oled_cleardisplay();
        oled.drawXBitmap(0, 0, logoBin, DispWidth, DispHeight, OLED_WHITE);
        oled_display();
      }
      if (actPicType == GSC) {
#ifdef XDEBUG
        oled_cleardisplay();
        oled.setCursor(0,0);
        oled.print("draw4bppBitmap");
        oled_display();
        delay(1000);
#endif
        oled_cleardisplay();
        oled_drawgreyscale(logoBin);
        oled_display();
      }    
*/


  // Read Picture
  uint16_t block_size = DispLineBytes1bpp;
  uint8_t b;
  uint16_t x = 0;
  uint16_t y = 0;

  uint8_t logoBin[block_size];   // Create Picture Buffer
  int serialBytesCount = 0;

  unsigned char d;
  int i, j;




  // Read Corename & effect
  String TextIn="",tT="";
  int d1=0;
  
#ifdef XDEBUG
  Serial.println(F("Called Command CMDCOR"));
#endif

  TextIn=newCommand.substring(7);                    // Get Command String
  d1 = TextIn.indexOf(',');                          // Search String for ","
  if (d1==-1) {                                      // No "," found = no Effect Parameter given
    actCorename=TextIn;                              // Get Corename
    tEffect=-1;                                      // Set Effect to -1 (Random)
#ifdef XDEBUG
    Serial.printf("\nReceived Text: %s, Transition T:None\n", (char*)actCorename.c_str());
#endif
  }
  else {                                             // "," found = Effect Parameter given
    actCorename=TextIn.substring(0, d1);             // Extract Corename from Command String
    tEffect=TextIn.substring(d1+1).toInt();          // Get Effect from Command String (set to 0 if not convertable)
    if (tEffect<-1) tEffect=-1;                      // Check Effect minimum
    if (tEffect>maxEffect) tEffect=maxEffect;        // Check Effect maximum
#ifdef XDEBUG
    Serial.printf("\nReceived Text: %s, Transition T:%i \n", (char*)actCorename.c_str(),tEffect);
#endif
  }




  oled_cleardisplay();

/*
   // drawxbitmap
  for(b = 0; b < logoBytes4bpp ; b += block_size) {
    bytesReadCount = Serial.readBytes((char*)logoBin, block_size);  // Read one block (from 2048 or 8192 Bytes image) from Serial
    if (bytesReadCount != block_size) {
      break;
    }

    oled.drawXBitmap(x, y, logoBin, block_size, 1, OLED_WHITE);        
    
    x += block_size;
    if (x >= DispWidth) {
      x = 0;
      y++;
    }
    if (y >= DispHeight) {
      break;
    }
  }
*/

/* DEBUG
y=1;
  for (i = 0; i < 250; i++) {
    oled.drawPixel(x+i,y,OLED_WHITE);
    x++;
  }*/

  // KO: Arduino Nano only got a buffer of 64 bytes for serial, but the whole picture is sent on the serial by the tty2oled service
  // So here in the code, we only read 32 bytes then 31 bytes (the remaining is lost).
  
  // drawpixel
  for(b = 0; b < logoBytes4bpp ; b += block_size) {
    serialBytesCount = Serial.readBytes((char*)logoBin, block_size);  // Read one block (from 2048 or 8192 Bytes image) from Serial
    /*Serial.print("CMDCOR: bytesReadCount\n");
    Serial.print(serialBytesCount);
    Serial.print("\nEND bytesReadCount\n\n");*/
    /*oled_printtext("\nserialBytesCount=\n");
    oled_printtext(serialBytesCount);*/
    if (serialBytesCount <= 0) {
      //Serial.println("serialBytesCount <= 0\n");
      break;
    }

    // draw a line of 256 pixels
    for (j = 0; j < block_size; j++) {
      d=logoBin[j];                // Get Data Byte for 8 Pixels
      for (i = 0; i < 8; i++){
        //Serial.println("oled.drawPixel(x*8+i,y,?);");
        if (bitRead(d, i)) {
          oled.drawPixel(j*8+i,y,OLED_WHITE);         // Draw Pixel if "1"
          //Serial.println("white");
          //oled_printtext("w");
        }
        else {
          oled.drawPixel(j*8+i,y,OLED_BLACK);         // Clear Pixel if "0"
          //Serial.println("black");
          //oled_printtext("b");
        }
      }
      /*Serial.println(x);
      Serial.println(y);
      Serial.println("end drawpixel");*/
    }
    y++;
    
    if (serialBytesCount != block_size) {
      //Serial.println("serialBytesCount != block_size\n");
      break;
    }

  /*oled_printtext("\nb=\n");
  oled_printtext(b);*/
  }
      
}  // end oled_read_and_draw_logo

// --------------- Draw 8 Pixel to Display Buffer ----------------------
// x,y: Data Coordinates of the Pixels on the Display
// dx,dy: Data Coordinates of the Pixels in the Array
// Normaly x=dx and y=dy but for the slide effects it's different.
// 8 Pixels are written, Data Byte(s) are taken from Array
// Display Positions are calculated from x,y and Type of Pic (XBM/GSX)
// ---------------------------------------------------------------------
void oled_drawEightPixelXY(int x, int y, int dx, int dy) {
  unsigned char b;
  int i;
  switch (actPicType) {
    case XBM:
      b=logoBin[dx+dy*DispLineBytes1bpp];                // Get Data Byte for 8 Pixels
      for (i=0; i<8; i++){
        if (bitRead(b, i)) {
          oled.drawPixel(x*8+i,y,OLED_WHITE);         // Draw Pixel if "1"
        }
        else {
          oled.drawPixel(x*8+i,y,OLED_BLACK);         // Clear Pixel if "0"
        }
      }
    break;
    case GSC:
      for (i=0; i<4; i++) {
        b=logoBin[(dx*4)+i+dy*DispLineBytes4bpp];        // Get Data Byte for 2 Pixels
        oled.drawPixel(x*8+i*2+0, y, (0xF0 & b) >> 4);   // Draw Pixel 1, Left Nibble
        oled.drawPixel(x*8+i*2+1, y, 0x0F & b);          // Draw Pixel 2, Right Nibble
      }
    break;
  }
}


// ----------------------------------------------------------------------
// ----------------------- Read and Write Text --------------------------
// ----------------------------------------------------------------------
void oled_readnwritetext(void) {
  int f=0,c=0,b=0,x=0,y=0,d1=0,d2=0,d3=0,d4=0,d5=0;
  //int16_t x1,y1;
  //uint16_t w1,h1;
  String TextIn="", fT="", cT="", bT="", xT="", yT="", TextOut="";
  bool bufferMode=false;
  bool pError=false;
  
#ifdef XDEBUG
  Serial.println(F("Called Command CMDTEXT"));
#endif
 
  TextIn = newCommand.substring(7);            // Get Command Text from "newCommand"
  
#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)TextIn.c_str());
#endif

  //Searching for the "," delimiter
  d1 = TextIn.indexOf(',');                 // Find location of first ","
  d2 = TextIn.indexOf(',', d1+1 );          // Find location of second ","
  d3 = TextIn.indexOf(',', d2+1 );          // Find location of third ","
  d4 = TextIn.indexOf(',', d3+1 );          // Find location of fourth ","
  d5 = TextIn.indexOf(',', d4+1 );          // Find location of fifth ","

  //Create Substrings
  fT = TextIn.substring(0, d1);             // Get String for Font-Type
  cT = TextIn.substring(d1+1, d2);          // Get String for Draw Color
  bT = TextIn.substring(d2+1, d3);          // Get String for Background Color
  xT = TextIn.substring(d3+1, d4);          // Get String for X-Position
  yT = TextIn.substring(d4+1, d5);          // Get String for Y-Position
  TextOut = TextIn.substring(d5+1);         // Get String for Text
  
#ifdef XDEBUG
  Serial.printf("\nCreated Strings: F:%s C%s B%s X:%s Y:%s T:%s\n", (char*)fT.c_str(), (char*)cT.c_str(), (char*)bT.c_str(), (char*)xT.c_str(), (char*)yT.c_str(), (char*)TextOut.c_str());
#endif

  // Convert Strings to Integer
  f = fT.toInt();
  c = cT.toInt();
  b = bT.toInt();
  x = xT.toInt();
  y = yT.toInt();
  
  // Parameter check
  if (f<0 || c<0 || c>15 || b<0 || b>15 || x<0 || x>DispWidth-1 || y<0 || y>DispHeight-1 || d1==-1 || d2==-1 || d3==-1 || d4==-1 || d5==-1) {
    pError=true;
  }

  if (f>=100) {                  // Do not run oled_display() after printing
    bufferMode=true;
    f=f-100;
  }

  oled_setfont(f);
  
  if (!pError) {
    // Write or Clear Text
    oled_settextcolor(c, b);
    oled_setcursor(x,y);                                  // Set Cursor Position
    oled_printtext(TextOut);                                  // Write Text to Buffer
    if (!bufferMode) oled_display();                      // Update Screen only if not "Clear Mode" (Font>100)
    oled_settextcolor(OLED_WHITE, OLED_BLACK);
  }
  else { 
    oled_showperror();
  }
}


// --------------------------------------------------------------
// ------------------ Read and Draw Geometrics ------------------
// --------------------------------------------------------------
void oled_readndrawgeo(void) {
  int g=0,c=0,x=0,y=0,i=0,j=0,k=0,l=0,d1=0,d2=0,d3=0,d4=0,d5=0,d6=0,d7=0;
  String TextIn="",gT="",cT="",xT="",yT="",iT="",jT="",kT="",lT="";
  bool pError=false;
  bool bufferMode=false;
  
#ifdef XDEBUG
  Serial.println(F("Called Command CMDGEO"));
#endif

  TextIn = newCommand.substring(7);             // Get Command Text from "newCommand"
  
#ifdef XDEBUG
  Serial.printf("\nReceived Text: %s\n", (char*)TextIn.c_str());
#endif
  
  //Searching for the "," delimiter
  d1 = TextIn.indexOf(',');                 // Find location of first ","
  d2 = TextIn.indexOf(',', d1+1 );          // Find location of second ","
  d3 = TextIn.indexOf(',', d2+1 );          // Find location of third ","
  d4 = TextIn.indexOf(',', d3+1 );          // Find location of fourth ","
  d5 = TextIn.indexOf(',', d4+1 );          // Find location of fifth ","
  d6 = TextIn.indexOf(',', d5+1 );          // Find location of sixt ","
  d7 = TextIn.indexOf(',', d6+1 );          // Find location of seventh ","

  //Create Substrings
  gT = TextIn.substring(0, d1);           // Get String for Geometric-Type
  cT = TextIn.substring(d1+1, d2);        // Get String for Clear Flag
  xT = TextIn.substring(d2+1, d3);        // Get String for X-Position
  yT = TextIn.substring(d3+1, d4);        // Get String for Y-Position
  iT = TextIn.substring(d4+1, d5);        // Get String for Parameter i
  jT = TextIn.substring(d5+1, d6);        // Get String for Parameter j
  kT = TextIn.substring(d6+1, d7);        // Get String for Parameter k
  lT = TextIn.substring(d7+1);            // Get String for Parameter l

#ifdef XDEBUG
  Serial.printf("\nPart-Strings: G:%s C:%s X:%s Y:%s I:%s J:%s K:%s L:%s\n", (char*)gT.c_str(), (char*)cT.c_str(), (char*)xT.c_str(), (char*)yT.c_str(), (char*)iT.c_str(), (char*)jT.c_str(), (char*)kT.c_str(), (char*)lT.c_str());
#endif

  // Convert Strings to Integer
  g = gT.toInt();
  c = cT.toInt();
  x = xT.toInt();
  y = yT.toInt();
  i = iT.toInt();
  j = jT.toInt();
  k = kT.toInt();
  l = lT.toInt();

#ifdef XDEBUG
  Serial.printf("\nValues: G:%i C:%i X:%i Y:%i I:%i J:%i K:%i L:%i\n", g,c,x,y,i,j,k,l);
#endif

  // Enough Parameter given / Parameter Check
  if (g<1 || c<0 || c>15 || x<0 || x>DispWidth-1 || y<0 || y>DispHeight-1 || d1==-1 || d2==-1 || d3==-1  || d4==-1 || d5==-1  || d6==-1 || d7==-1) {
    pError=true;
  }
  
  if (g>100) {                  // Do not run oled_display() after drawing
    bufferMode=true;
    g=g-100;
  }
  
  if (!pError) {
    switch (g) {
      case 1:  // Pixel x,y
        oled.drawPixel(x,y,c);
      break;
      case 2:  // Line x0,y0,x1,y1,c
        oled.drawLine(x,y,i,j,c);
      break;
      case 3:  // Rectangle x,y,w,h,c
        oled.drawRect(x,y,i,j,c);
      break;
      case 4:  // Filled Rectangle/Box x,y,w,h,c
        oled.fillRect(x,y,i,j,c);
      break;
      case 5:  // Circle x,y,r,c
        oled.drawCircle(x,y,i,c);
      break;
      case 6:  // Filled Circle x,y,r,c
        oled.fillCircle(x,y,i,c);
      break;
      case 7:  // Rounded Rectangle x,y,w,h,r,c
        oled.drawRoundRect(x,y,i,j,k,c);
      break;
      case 8: // Filled Rounded Rectangle x,y,w,h,r,c
        oled.fillRoundRect(x,y,i,j,k,c);
      break;
      case 9: // Triangle x0,y1,x1,y1,x2,y2,c
        oled.drawTriangle(x,y,i,j,k,l,c);
      break;
      case 10: // Filled Triangle x0,y1,x1,y1,x2,y2,c
        oled.fillTriangle(x,y,i,j,k,l,c);
      break;
      default:  // Just something :-)
        oled.drawCircle(128,32,32,15);
        oled.fillCircle(128,32,8,8);
        break;
    }
    if (!bufferMode) oled_display();                       // Update Screen only if not Buffer Mode (Geo>100)
  }
  else { 
    oled_showperror();
  }
}


//========================== The end ================================
