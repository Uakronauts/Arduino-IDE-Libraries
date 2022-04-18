// Adafruit Internal Flash FatFs Simple Datalogging Example
// Author: Tony DiCola, Ha Thach
//
// This is a simple dataloging example using the SPI Flash
// FatFs library.  The example will open a file on the SPI
// flash and append new lines of data every minute. Note that
// you MUST have a flash chip that's formatted with a flash
// filesystem before running.  See the fatfs_format example
// to perform this formatting.
//
// Usage:
// - Modify the pins and type of fatfs object in the config
//   section below if necessary (usually not necessary).
// - Upload this sketch to your M0 express board.
// - Open the serial monitor at 115200 baud.  You should see the
//   example print a message every minute when it writes a new
//   value to the data logging file.

#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_InternalFlash.h"

// Start address and size should matches value in the CircuitPython (INTERNAL_FLASH_FILESYSTEM = 1)
// to make it easier to switch between Arduino and CircuitPython
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (64*1024)

Adafruit_InternalFlash flash(INTERNAL_FLASH_FILESYSTEM_START_ADDR, INTERNAL_FLASH_FILESYSTEM_SIZE);

// file system object from SdFat
FatFileSystem fatfs;

// Configuration for the datalogging file:
#define FILE_NAME      "data.csv"


void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Adafruit Internal Flash FatFs Simple Datalogging Example");

  // Initialize flash library
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1) delay(1);
  }

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1) delay(1);
  }
  Serial.println("Mounted filesystem!");

  Serial.println("Logging data every 60 seconds...");
}

void loop() {
  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
    // Take a new data reading from a sensor, etc.  For this example just
    // make up a random number.
    int reading = random(0,100);
    // Write a line to the file.  You can use all the same print functions
    // as if you're writing to the serial monitor.  For example to write
    // two CSV (commas separated) values:
    dataFile.print("Sensor #1");
    dataFile.print(",");
    dataFile.print(reading, DEC);
    dataFile.println();
    // Finally close the file when done writing.  This is smart to do to make
    // sure all the data is written to the file.
    dataFile.close();
    Serial.println("Wrote new measurement to data file!");
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }

  Serial.println("Trying again in 60 seconds...");

  // Wait 60 seconds.
  delay(60000L);
}
