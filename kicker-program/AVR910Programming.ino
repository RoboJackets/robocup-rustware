

////////////////////////////////
// USING C++ HEADER FILE INSTEAD OF SD CARD
////////////////////////////////

// ADDED AT LEAST 100 MS BETWEEN EACH COMMAND
// Kicker docs on github are useful
// https://github.com/RoboJackets/robocup-firmware/blob/master/doc/Kicker.md

// VERY IMPORTANT:
// as of 3/10/2024, data was correctly read on rising edge, in SPI_MODE0.
// When writing serial data to the ATmega32A, data is clocked on the rising edge of SCK.
// When reading data from the ATmega32A, data is clocked on the falling edge of SCK
// FROM HERE: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmega32A-DataSheet-Complete-DS40002072A.pdf


// immediately after reset goes active (low), SCK should be driven to 0
// During this first phase of the programming cycle, keeping the SCK Line free
//      from pulses is critical, as pulses will cause the target AVR to lose synchronization with the programmer.
// MISO line only becomes an output on the AVR after the enable programming command

// The VCC between Teensy and AVR can be used to power the Teensy to make sure voltages are correct.
// As an alternative, the target system can have its power supplied from the programmer through the same
// connector used for the communication. This would allow the target to be programmed without applying
// power to the target externally.



/////////////////////////////
// COMMAND FORMAT FOR AVR PROGRAMMING
/////////////////////////////
// - all commands consist of four bytes
//     - first byte contains command code, selecting operation, and target memory
//     - second and third byte contain the address of the selected memory area
//     - fourth byte contains the data
// - data returned is usually the data sent in the previous byte
// - MSB first



// kicker_bin.h is the file with the array of bytes
// array is called KICKER_BYTES
// length is called KCIKER_BYTES_LEN

#include "kicker_bin.h"
#include <SPI.h>
// pgmspace used for accessing array in program memory
#include <avr/pgmspace.h>
//#include <SD.h>

//char KICKER_BYTES[500];
//int KICKER_BYTES_LEN = 500;


#define ATMEL_VENDOR_CODE 0x1E
#define DEVICE_LOCKED 0x00
#define WRITE_HIGH_BYTE 0x48
#define WRITE_LOW_BYTE 0x40
#define READ_HIGH_BYTE 0x28
#define READ_LOW_BYTE 0x20
#define WRITE_HIGH_FLASH_BYTE 0x68
#define WRITE_LOW_FLASH_BYTE 0x60

#define AVR_FAMILY_MASK 0xF0
#define AVR_FAMILY_ID 0x95
#define ATMEGA_DEVICE_ID 0x02
#define ATMEGA_PAGESIZE 64  // Size in words (word = 2 bytes)
#define ATMEGA_NUM_PAGES 256




// SPISettings takes (maxClockSpeed, dataOrder, dataMode)
// Hard to find what clock speed the AVR processor has
// looking at following website for speeds, not sure what bus the actual avr processor is on
// https://github.com/RoboJackets/robocup-firmware/blob/master/control/mtrain/tests/cpp/spi.cpp
// trying the lowest speed, since the AVR only requires a minimum low and high period
//      - means that there is a maximum clock speed, but I can't find where what this is for the specific AVR on the kicker board
// DEFAULT CLOCK SPEED: 4MHZ

SPISettings w_settings(100000, MSBFIRST, SPI_MODE0);

// unclear if SPI_MODE1 or 2 is used for reading
SPISettings r_settings(100000, MSBFIRST, SPI_MODE0);

// following three are already declared in the SD card library
// only CS pin is needed
const int MOSI_PIN = 11; // default
const int MISO_PIN = 12; // default
const int SCK_PIN = 13;  // default
const int CS_PIN = 10;
//const int MOSI_PIN = 26;   //alternate
//const int MISO_PIN = 1;    //alternate
//const int SCK_PIN = 27;    //alternate
//const int CS_PIN = 10;      //alternate


// Arduino Uno doesn't have a pin 14, so when using one for in-system programming use pin 9, plugging into 14 on kicker testing board
const int RST_PIN = 9;
// reset pin is normally high.9



bool programmed = false;
bool programmingEnabled = false;
bool erased = false;
int programmingAttempts = 0;

// determines if we're writing the high or low byte
bool highLow = 0;

// open binary **** Switching to using the header file ****

//const int SDchipSelect = BUILTIN_SDCARD;
//const char* filename = "binary.txt"; // get the actual name
//File binary;

// index in the header file array
int byteIndex = 0;


void setup() {
  Serial.begin(9600);

  // set CS_PIN and RST_PIN as output
  pinMode(CS_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  //pinMode(SCK_PIN, OUTPUT);

  // set chip select to high
  digitalWrite(CS_PIN, 1);
  
  digitalWrite(RST_PIN, 1);

  delay(50);

  // SCK has to be low for at least 20 ms after RST becomes active (low)
  digitalWrite(SCK_PIN, LOW);

  delay(500);


  // reset pin to low to allow programming to be enabled
  digitalWrite(RST_PIN, 0); 

  delay(500);


  // initialize SPI
  SPI.begin();



  // not necessary anymore, using header file instead of SD card
  /*
  // only attempt to access the SD card 5 times
  int initializeAttempts = 0;

  // loop attempting to initialize the SD card, give up after 5 attempts
  
  while (!SD.begin(SDchipSelect)) {
    initializeAttempts++;
    Serial.println("SD card failed to initialize, attempt " + String(initializeAttempts));

    // makes the programming procedure not run
    programmingAttempts = 5;
    delay(2000);
  
    if (initializeAttempts > 5) {
      Serial.println("SD card failed to initalize, giving up");
      break;
    }
  }

  if (initializeAttempts <= 5) {
    // allows the programming procedure to run
    programmingAttempts = 0;
    Serial.println("SD card initialized");
    delay(500);

    // initialize the file to be used
    binary = SD.open(filename);
  }
  */

  Serial.println("STARTING");
}


////////////////////////////////////////////////////////////////////
// functions to use in loop
////////////////////////////////////////////////////////////////////

// sends signals to enable programming for the AVR
bool enableProgramming() {
  // Programming Enable Command: 0xAC, 0x53, 0x00, 0x00
  byte received0, received1, received2, received3;

  Serial.println("sending enable command");

  // begin transaction determines settings for the SPI transfers
  // not necessary if using default settings
  SPI.beginTransaction(w_settings);

  digitalWrite(CS_PIN, 0);



  received0 = SPI.transfer(0xAC);

  received1 = SPI.transfer(0x53);
  
  received2 = SPI.transfer(0x00);
  
  received3 = SPI.transfer(0x00);
  
  Serial.println(received0, HEX);
  Serial.println(received1, HEX);
  Serial.println(received2, HEX);
  Serial.println(received3, HEX);
  
  digitalWrite(CS_PIN, 1);

  SPI.endTransaction();

  Serial.println("sent enable command");
  
  delay(100);




  return (received2 == 0x53);
}


bool chipErase() {
  // chip erase command is 0xAC, 0x80, 0x00, 0x00
  
  // variable to check if the chip erase is working
  // will be assigned to erased
  byte returned;

  // enable proper settings
  SPI.beginTransaction(w_settings);

  // set chip select pin to low (active)
  digitalWrite(CS_PIN, 0);

  SPI.transfer(0xAC);
  SPI.transfer(0x80);
  returned = SPI.transfer(0x00);
  SPI.transfer(0x00);

  Serial.print("erase return was ");
  Serial.println(returned);

  // return chip select to high
  digitalWrite(CS_PIN, 1);

  SPI.endTransaction();

  delay(20);

  // need to release the reset line to finish chip erase
  /*
  digitalWrite(RST_PIN, 1);
  delay(10);
  digitalWrite(RST_PIN, 0);
  */
  return (returned == 0x80);
  
}


// stop the programming cycle
void exitProgramming() {
  digitalWrite(RST_PIN, 1);
  delay(1000);
  digitalWrite(RST_PIN, 0);
}


// reading a register
byte readRegister(byte reg) {
  delay(20);
  byte val;
  byte received;

  // might have to change SPI mode for reading from a register - data clocked on falling edge when reading data
  // enable settings
  SPI.beginTransaction(r_settings);

  digitalWrite(CS_PIN, 0);


  received = SPI.transfer(0x30);
  Serial.println(received, HEX);
  received = SPI.transfer(0x00);
  Serial.println(received, HEX);
  received = SPI.transfer(reg);
  Serial.println(received, HEX);
  val = SPI.transfer(0x00);

  digitalWrite(CS_PIN, 1);

  SPI.endTransaction();

  delay(20);


  Serial.println(val, HEX);
  Serial.println("");
  return val;
}



byte readVendorCode() {
  // address 0x00 is the vendor code
  return readRegister(0x00);
}




byte readPartFamilyAndFlashSize() {
  return readRegister(0x01);
}




byte readPartNumber() {
  return readRegister(0x02);
}



bool verify_param(String name, byte expected, byte received, char mask) {
  Serial.println("Checking " + name + "...");
  
  // checks if what we expect and received are the same
  //bool success = ((received & mask) == expected);
  bool success = (received == expected);

  if (success) {
    Serial.println("done");
  }
  else {
    Serial.println("Got unexpected value: 0x" + String(received));
    Serial.println("Expected: " + String(expected));
  }

  return success;
}






bool performChecks() {
  
  // everything we need to check {
  // Can't make tuples in Arduino, so using struct
  struct Test {
    String name; char expected; int received; char mask;
  };
  
  delay(20);

  Serial.println("");
  Serial.println("Checking params, reading codes");


  byte vendorCode = readVendorCode();
  byte family = readPartFamilyAndFlashSize();
  byte device = readPartNumber();


  Test vendorTest = {"Vendor ID", ATMEL_VENDOR_CODE, vendorCode, 0xFF};
  Test familyTest = {"Part Family", AVR_FAMILY_ID, 
    family, AVR_FAMILY_MASK};
  Test deviceTest = {"Device ID", ATMEGA_DEVICE_ID, device, 0xFF};

  Test tests[] = {vendorTest, familyTest, deviceTest};

  Serial.println("verifying codes");
  // checking each one
  for (int i = 0; i<3; i++) {
    if (!verify_param(tests[i].name, tests[i].expected, tests[i].received, tests[i].mask)) {
      return false;
    }
  }


  // if it gets here, every verify_param was true for all
  return true;
}










bool programBinary(int pageSize, int numPages) {
  char pageOffset = 0;
  int pageNumber = 0;
  int c = 0;
  int highLow = 0;

  char lc_offset = 0xFF;
  int lc_highlow = 0xFF;

  byteIndex = 0;

  // paged memory
  if (numPages > 1) {
    // loops through header file array while there is binary available
    while (byteIndex < KICKER_BYTES_LEN) {
      
      //read the next byte
      // KICKER_BYTES stored in program memory, so has to be accessed with this function
      // KICKER_BYTES gives the location in memory, adding byteIndex to iterate through array
      c = pgm_read_byte_near(KICKER_BYTES + byteIndex);
      byteIndex++;

      //Serial.print("current byte index: ");
      //Serial.print(byteIndex);
      //Serial.print("/" + String(KICKER_BYTES_LEN) + ": ");
      //Serial.println(c, HEX);
      if (byteIndex % 150 == 0) {
        Serial.print("Current Index : ");
        Serial.println(byteIndex);
      }

      // page is fully loaded, time to write it to flash
      if (pageOffset == (pageSize)) {
        writeFlashMemoryPage(pageNumber, lc_offset, lc_highlow);
        lc_offset = 0xFF;
        lc_highlow = 0xFF;

        pageNumber++;
        
        // if more pages are written than there are available in chip
        if (pageNumber > numPages) {
          Serial.println( "ERROR: AVR910 binary exceeds chip memory capacity");
          return false;
        }
        pageOffset = 0;
      }

      if (lc_offset == 0xFF && c != 0xFF) {
        lc_offset = pageOffset;
        if (!highLow) {
          lc_highlow = READ_LOW_BYTE;
        }
        else {
          lc_highlow = READ_HIGH_BYTE;
        }
      }

      // Write low byte.
      if (highLow == 0) {
        loadMemoryPage(WRITE_LOW_BYTE, pageOffset, c);
        highLow = 1;
      }

      // Write high byte.
      else {
        loadMemoryPage(WRITE_HIGH_BYTE, pageOffset, c);
        highLow = 0;
        pageOffset++;
      }
  }

  // We might have partially filled up a page
  writeFlashMemoryPage(pageNumber, lc_offset, lc_highlow);

  delay(2000);

  // make sure that the memory was programmed completely and successfully
  bool success = checkMemory(pageSize, pageNumber+1, true);
    return success;
  }
  return false;
}

void writeFlashMemoryPage(char pageNumber, char pageOffset, int highLow) {
  // Write program memory page command
  // writing at address a:b
  delay(20);

  // enable settings
  SPI.beginTransaction(w_settings);

  digitalWrite(CS_PIN, 0);
  SPI.transfer(0x4C); // 0100 1100
  // 11 bits total, 5 for page offset, 6 for page number
  SPI.transfer(pageNumber >> 2);
  SPI.transfer(pageNumber << 6);
  SPI.transfer(0x00); // xxxx xxxx
  digitalWrite(CS_PIN, 1);

  //SPI.endTransaction();

  delay(20);
  // might have to look into the poll function in AVR910.cpp
}


void loadMemoryPage(int highLow, char address, char data) {
  // load program memory page command
  // write H (high or low) data i to Program
  // Memory page at word address b. Data
  // low byte must be loaded before Data
  // high byte is applied within the same
  // address.

  //enable settings
  SPI.beginTransaction(w_settings);

  delay(20);
  digitalWrite(CS_PIN, 0);
  SPI.transfer(highLow); // 0100 H000
  SPI.transfer(0x00);
  SPI.transfer(address & 0x3F); // xxxb bbbb
  SPI.transfer(data); // iiii iiii
  digitalWrite(CS_PIN, 1);
  delay(20);

  SPI.endTransaction();
}


char readProgramMemory(int highLow, byte pageNumber, byte pageOffset) {
  // read program memory command
  // Read H (hi or lo) data o from
  // Program memory at word address a:b

  // enable SPI settings
  SPI.beginTransaction(r_settings);

  digitalWrite(CS_PIN, 0);
  SPI.transfer(highLow); // 0100 0H00
  SPI.transfer(pageNumber >> 2); // 00aa aaaa
  SPI.transfer((pageNumber <<6) | (pageOffset & 0x3F)); // aabb bbbb
  byte response = SPI.transfer(0x00); // oooo oooo
  digitalWrite(CS_PIN, 1);

  SPI.endTransaction();

  return response;
}


bool checkMemory(int pageSize, int numPages, bool verbose) {
  bool success = true;

  Serial.println("[INFO] Checking memory? (pagesize: " + String(pageSize) + ", numpages: " + String(numPages));

  // Go back to the beginning of the binary file.
  byteIndex = 0;

  for (int page = 0; page < numPages; page++) {
    // leave loop if something did match or we've checked everything
    if (!success || byteIndex >= KICKER_BYTES_LEN) {
      break;
    }

      for (int offset = 0; offset < pageSize; offset++) {
          byte response;


          // break if we've read all of the valid bytes before we read the low byte
          if (byteIndex >= KICKER_BYTES_LEN) {
            break;
          }

          // read a byte from the expected array
          byte c = pgm_read_byte_near(KICKER_BYTES + byteIndex);
          byteIndex++;



          // Read program memory low byte from the AVR.
          response = readProgramMemory(READ_LOW_BYTE, page, offset);

          //Serial.println("expected " + String(c, HEX) + ", received " + String(response, HEX));

          // check if received equals expected
          if (c != response) {
               success = false;
               break;
          }

          // break if we've read all of the valid bytes before we read the high byte
          if (byteIndex >= KICKER_BYTES_LEN) {
            break;
          }

          // get the next expected byte from Uno flash memory
          c = pgm_read_byte_near(KICKER_BYTES + byteIndex);
            byteIndex++;

           // Read program memory high byte.
          response = readProgramMemory(READ_HIGH_BYTE, page, offset);

          Serial.println("expected " + String(c, HEX) + ", received " + String(response, HEX));

          if (c != response) {
            success = false;
          }
      }
   }

  // GOES TOO FAR ON LAST PAGE
  if (verbose) {
       if (success) {
          Serial.println("[INFO] Kicker Memory Contents: OK.");
       } else {
          Serial.println("[ERROR] Kicker Memory Contents: FAILED.");
      }
  }

  return success;
}




////////////////////////////////////////
// main loop
////////////////////////////////////////

void loop() {
  // wait 5 seconds before starting
  delay(5000);


  // don't want to program the AVR multiple times
  if (!programmed && programmingAttempts < 5) {
    programmingAttempts++;
    
    //Serial.println("flashing reset");

    // to enable programming, give reset a positive pulse for at least two clock cycles
    // moved this reset pulse to setup
    /*
    digitalWrite(RST_PIN, 1);
    delay(1000);
    digitalWrite(RST_PIN, 0);
    */
    
    // wait another 100 ms
    delay(100);

    //attempting to enable programming
    // enabling programming (returns true if it worked, false otherwise)
    Serial.println("attempting to enable programming");
    programmingEnabled = enableProgramming();
    //Serial.println("attempted to enable programming");

    delay(100);

    // proceeds with other programming commands
    if (programmingEnabled) {

      Serial.println("Programming successfully enabled");
      

      // attempts to perform the chip erase function
      erased = chipErase();

      
      delay(100);
      
      // continue with programming
      if (erased) {
          Serial.println("\nerased successfully\n");

          // checks if we're targeting the correct chip
          // make performChecks() work
          if (performChecks()) {

            delay(100);
            Serial.println("about to program binary\n");
            Serial.println("");

            // START ACTUALLY PROGRAMMING
            programmed = programBinary(ATMEGA_PAGESIZE, ATMEGA_NUM_PAGES);

            delay(100);

            // done programming
            exitProgramming();
            Serial.println("DONE");

          }
          else {
            Serial.println("ERROR: TARGET NOT CORRECT");
            
            delay(100);

            exitProgramming();
          }
      }
    
      else {
        delay(100);
        Serial.println("ERROR: FAILED TO ERASE CHIP");
        exitProgramming();
      }
    } 
    // if programming is not enabled
    else {
      Serial.println("programming not enabled successfully");
      
      // flash the reset pin
      
      digitalWrite(RST_PIN, 1);
      delay(100);
      digitalWrite(RST_PIN, 0);
      delay(100);
      

      if (programmingAttempts == 5) {
        Serial.println("Giving up");
        digitalWrite(RST_PIN, 1);
        delay(100);
        SPI.end();
      }
    }
    
  }
}

