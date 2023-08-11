/** =========================================================================
 * @file sonar_simple_logging.ino
 * @brief Example for Maxbotix Sonar, based on simple logging example.
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @author Anthony Aufdenkampe <aaufdenkampe@limno.com>
 * @copyright (c) 2017-2022 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 * Firmware Library: ModularSensors v0.34.0, released 2023-03-16
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Include the libraries required for any data logger
// ==========================================================================
/** Start [includes] */
// The Arduino library is needed for every Arduino program.
#include <Arduino.h>

// EnableInterrupt is used by ModularSensors for external and pin change
// interrupts and must be explicitly included in the main program.
#include <EnableInterrupt.h>

// Include the main header for ModularSensors
#include <ModularSensors.h>
/** End [includes] */


// ==========================================================================
//  Creating Additional Serial Ports
// ==========================================================================

// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you need.
// Not all AVR boards are supported by NeoSWSerial.
/** Start [neoswserial] */
#include <NeoSWSerial.h>          // for the stream communication
const int8_t neoSSerial1Rx = 7;  // data in pin
const int8_t neoSSerial1Tx = -1;  // data out pin
NeoSWSerial  neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR() {
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}
/** End [neoswserial] */


// ==========================================================================
//  Assigning Serial Port Functionality
// ==========================================================================

// The Maxbotix sonar is the only sensor that communicates over a serial port
// but does not use modbus
// Since the Maxbotix only needs one-way communication and sends a simple text
// string repeatedly, almost any software serial port will do for it.
#define sonarSerial neoSSerial1     // For Neo software serial


// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "sonar_simple_logging.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "sonar_Tester";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = -6;  // Central Standard Time
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const int32_t serialBaud = 115200;  // Baud rate for debugging
const int8_t  greenLED   = 8;       // Pin for the green LED
const int8_t  redLED     = 9;       // Pin for the red LED
const int8_t  buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t  wakePin    = A7;  // MCU interrupt/alarm pin to wake from sleep
// Mayfly 0.x D31 = A7
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */


// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_stats] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5";
ProcessorStats mcuBoard(mcuBoardVersion);

// Create sample number, battery voltage, and free RAM variable pointers for the
// processor
// Variable* mcuBoardBatt = new ProcessorStats_Battery(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable* mcuBoardAvailableRAM = new ProcessorStats_FreeRam(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable* mcuBoardSampNo = new ProcessorStats_SampleNumber(
//     &mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
/** End [processor_stats] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [maxim_ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
// Variable* ds3231Temp =
//     new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");
/** End [maxim_ds3231] */


// ==========================================================================
//    Settings for Additional Sensors
// ==========================================================================

// ==========================================================================
//  Maxbotix HRXL Ultrasonic Range Finder
// ==========================================================================
/** Start [max_botix_sonar] */
#include <sensors/MaxBotixSonar.h>

// A Maxbotix sonar with the trigger pin disconnect CANNOT share the serial port
// A Maxbotix sonar using the trigger may be able to share but YMMV

// NOTE: Extra hardware and software serial ports are created in the "Settings
// for Additional Serial Ports" section

// NOTE: Use -1 for any pins that don't apply or aren't being used.
const int8_t SonarPower    = sensorPowerPin;  // Excite (power) pin
const int8_t Sonar1Trigger = -1;              // Trigger pin
// Trigger should be a *unique* negative number if unconnected
const uint8_t sonar1NumberReadings = 3;  // The number of readings to average

// Create a MaxBotix Sonar sensor object
MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger,
                     sonar1NumberReadings);

// Create an ultrasonic range variable pointer
Variable* sonar1Range =
    new MaxBotixSonar_Range(&sonar1, "12345678-abcd-1234-ef00-1234567890ab");
/** End [max_botix_sonar] */


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
//  NOTE:  There are three different ways of creating the same variable array
//         and filling it with variables. Here we blend methods.
// ==========================================================================
/** Start [variables_create_in_array_pre_named] */
Variable* variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard,
                                    "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_FreeRam(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    new ProcessorStats_Battery(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    // new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
    // Add variable using method 3, pre_named
    sonar1Range,
    //  ... Add more variables as needed!
    // new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_Temp(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Variable(calculateVariableValue, calculatedVarResolution,
    //              calculatedVarName, calculatedVarUnit, calculatedVarCode,
    //              calculatedVarUUID),
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);
// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
/** End [variables_create_in_array_pre_named] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  Working Functions
// ==========================================================================
/** Start [working_functions] */
// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75) {
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
    /** Start [setup_prints] */
    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("\n\nNow running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    /** Start [setup_softserial] */
    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt,
                        CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif
    /** End [setup_softserial] */

    /** Start [setup_serial_begins] */

    // Start the SoftwareSerial stream for the sonar; it will always be at 9600
    // baud
    sonarSerial.begin(9600);
    /** End [setup_serial_begins] */

    /** Start [setup_flashing_led] */
    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();
    /** End [setup_flashing_led] */

    /** Start [setup_logger] */
    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Set information pins
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the logger
    dataLogger.begin();
    /** End [setup_logger] */

    /** Start [setup_sensors] */
    Serial.println(F("Setting up sensors..."));
    varArray.setupSensors();
    /** End [setup_sensors] */

    /** Start [setup_file] */
    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    dataLogger.createLogFile(true);  // true = write a new header
    /** End [setup_file] */

    /** Start [setup_sleep] */
    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
    /** End [setup_sleep] */
}


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [simple_loop] */
void loop() {
    dataLogger.logData();
}

/** End [simple_loop] */
