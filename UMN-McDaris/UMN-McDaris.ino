/** =========================================================================
 * @file CLFL-stream-1.ino
 * @brief From ModularSensors examples DRWI_LTE.ino + menu_a_la_carte.ino
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @author Anthony Aufdenkampe <aaufdenkampe@limno.com>
 * @copyright (c) 2017-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: PlatformIO in Atom or VSCode
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 * Software System: ModularSensors v0.25.1
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

// To get all of the base classes for ModularSensors, include LoggerBase.
// NOTE:  Individual sensor definitions must be included separately.
#include <LoggerBase.h>
/** End [includes] */


// ==========================================================================
//  Settings for Additional Serial Ports
// ==========================================================================
// The modem and a number of sensors communicate over UART/TTL - often called
// "serial". "Hardware" serial ports (automatically controlled by the MCU) are
// generally the most accurate and should be configured and used for as many
// peripherals as possible.  In some cases (ie, modbus communication) many
// sensors can share the same serial port.

// Unfortunately, most AVR boards have only one or two hardware serial ports,
// so we'll set up three types of extra software serial ports to use

// AltSoftSerial by Paul Stoffregen
// (https://github.com/PaulStoffregen/AltSoftSerial) is the most accurate
// software serial port for AVR boards. AltSoftSerial can only be used on one
// set of pins on each board so only one AltSoftSerial port can be used. Not all
// AVR boards are supported by AltSoftSerial.
/** Start [altsoftserial] */
#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;
/** End [altsoftserial] */

// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you want.
// Not all AVR boards are supported by NeoSWSerial.
/** Start [neoswserial] */
#include <NeoSWSerial.h>          // for the stream communication
const int8_t neoSSerial1Rx = 11;  // data in pin
const int8_t neoSSerial1Tx = -1;  // data out pin
NeoSWSerial  neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR() {
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}
/** End [neoswserial] */



// ==========================================================================
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "CLFL-stream-1.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "CLFL-stream-1";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 10;
// Your logger's timezone.
const int8_t timeZone = -6;  // Central Standard Time (CST=-6)
// NOTE:  Daylight savings time will not be applied!  Please use standard time!

// Set the input and output pins for the logger
// NOTE:  Use -1 for pins that do not apply
const long   serialBaud = 115200;  // Baud rate for debugging
const int8_t greenLED   = 8;       // Pin for the green LED
const int8_t redLED     = 9;       // Pin for the red LED
const int8_t buttonPin  = 21;      // Pin for debugging mode (ie, button pin)
const int8_t wakePin    = A7;      // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */


// // ==========================================================================
// //  Wifi/Cellular Modem Options
// // ==========================================================================
// /** Start [xbee_cell_transparent] */
// // For any Digi Cellular XBee's
// // NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global)
// // are more stable used in bypass mode (below)
// // The Telit based Digi XBees (LTE Cat1) can only use this mode.
// #include <modems/DigiXBeeCellularTransparent.h>
//
// // Create a reference to the serial port for the modem
// HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
// const long      modemBaud   = 9600;     // All XBee's use 9600 by default
//
// // Modem Pins - Describe the physical pin connection of your modem to your board
// // NOTE:  Use -1 for pins that do not apply
// const int8_t modemVccPin    = -2;    // MCU pin controlling modem power
// const int8_t modemStatusPin = 19;    // MCU pin used to read modem status
// const bool useCTSforStatus = false;  // Flag to use the modem CTS pin for status
// const int8_t modemResetPin = 20;     // MCU pin connected to modem reset pin
// const int8_t modemSleepRqPin = 23;   // MCU pin for modem sleep/wake request
// const int8_t modemLEDPin = redLED;   // MCU pin connected an LED to show modem
//                                      // status (-1 if unconnected)
//
// // Network connection information
// const char* apn = "hologram";  // The APN for the gprs connection
//
// DigiXBeeCellularTransparent modemXBCT(&modemSerial, modemVccPin, modemStatusPin,
//                                       useCTSforStatus, modemResetPin,
//                                       modemSleepRqPin, apn);
// // Create an extra reference to the modem by a generic name
// DigiXBeeCellularTransparent modem = modemXBCT;
// /** End [xbee_cell_transparent] */


// ==========================================================================
//  Using the Processor as a Sensor
// ==========================================================================
/** Start [processor_sensor] */
#include <sensors/ProcessorStats.h>

// Create the main processor chip "sensor" - for general metadata
const char*    mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);
/** End [processor_sensor] */


// ==========================================================================
//  Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
/** Start [ds3231] */
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);
/** End [ds3231] */


// ==========================================================================
//  Meter Hydros 21 Conductivity, Temperature, and Depth Sensor
// ==========================================================================
/** Start [hydros21] */
#include <sensors/DecagonCTD.h>

const char*   CTDSDI12address   = "1";    // The SDI-12 Address of the CTD
const uint8_t CTDNumberReadings = 6;      // The number of readings to average
const int8_t  CTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
const int8_t  CTDData  = 7;               // The SDI12 data pin

// Create a Decagon CTD sensor object
DecagonCTD ctd(*CTDSDI12address, CTDPower, CTDData, CTDNumberReadings);

// Create conductivity, temperature, and depth variable pointers for the CTD
Variable* ctdCond = new DecagonCTD_Cond(&ctd,
                                        "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdTemp = new DecagonCTD_Temp(&ctd,
                                        "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdDepth =
    new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-ef00-1234567890ab");
/** End [hydros21] */

// // ==========================================================================
// //    Maxbotix HRXL Ultrasonic Range Finder
// // ==========================================================================
// #include <sensors/MaxBotixSonar.h>
//
// // Create a reference to the serial port for the sonar
// // A Maxbotix sonar with the trigger pin disconnect CANNOT share the serial port
// // A Maxbotix sonar using the trigger may be able to share but YMMV
// // Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
// // AltSoftSerial &sonarSerial = altSoftSerial;  // For software serial if needed
// NeoSWSerial &sonarSerial = neoSSerial1;  // For software serial if needed
// // SoftwareSerial_ExtInts &sonarSerial = softSerial1;  // For software serial if needed
//
// const int8_t SonarPower = sensorPowerPin;  // Excite (power) pin (-1 if unconnected)
// const int8_t Sonar1Trigger = -1;  // Trigger pin (a unique negative number if unconnected) (D25 = A1)
// const uint8_t sonar1NumberReadings = 5;  // The number of readings to average
//
// // Create a MaxBotix Sonar sensor object
// MaxBotixSonar sonar1(sonarSerial, SonarPower, Sonar1Trigger, sonar1NumberReadings);
//
// // Create an ultrasonic range variable pointer
// Variable *sonar1Range = new MaxBotixSonar_Range(&sonar1, "9805e438-9c06-405e-b937-fd88aeda96af");
//

// // ==========================================================================
// //    Yosemitech Y520 Conductivity Sensor
// // ==========================================================================
// #include <sensors/YosemitechY520.h>
//
// // Create a reference to the serial port for modbus
// AltSoftSerial &modbusSerial = altSoftSerial;  // For software serial if needed
//
// byte y520ModbusAddress = 0x01;  // The modbus address of the Y520
// const int8_t rs485AdapterPower = sensorPowerPin;  // Pin to switch RS485 adapter power on and off (-1 if unconnected)
// const int8_t modbusSensorPower = sensorPowerPin;  // Pin to switch sensor power on and off (-1 if unconnected)
// const int8_t max485EnablePin = -1;  // Pin connected to the RE/DE on the 485 chip (-1 if unconnected)
// const uint8_t y520NumberReadings = 5;  // The manufacturer recommends averaging 10 readings, but we take 5 to minimize power consumption
//
// // Create a Y520 conductivity sensor object
// YosemitechY520 y520(y520ModbusAddress, modbusSerial, rs485AdapterPower, modbusSensorPower, max485EnablePin, y520NumberReadings);
//

// ==========================================================================
//    Calculated Variables
// ==========================================================================

// // Create the function to give your calculated result.
// // The function should take no input (void) and return a float.
// // You can use any named variable pointers to access values by way of variable->getValue()
//
// // Create the function to calculate water level / gage height variable
// float calculateSonarGageHeight(void)
// {
//     float sonarGageHeight = -9999;  // Always safest to start with a bad value
//     float sonarGageHeight_mm = -9999;  // Always safest to start with a bad value
//     const float minimumRange = 300;    // in millimeters
//     const float maximumRange = 5000;    // in millimeters
//     const float sonarDistanceToZeroStage = 304.8*(5); // in millimeters, where 304.8 mm = 1.00 ft
//     float sonarDistanceMeasured = sonar1Range->getValue();
//     if (sonarDistanceMeasured != -9999)  // make sure all inputs are good
//     {
//         sonarGageHeight_mm = sonarDistanceToZeroStage - sonarDistanceMeasured;
//         sonarGageHeight = sonarGageHeight_mm / 304.8; // to convert to feet, divide by 304.8, or divide by 1 to remain in mm.
//     }
//     return sonarGageHeight;
// }
//
// // Properties of the calculated water level / gage height variable
// const uint8_t sonarGageHeightVarResolution = 3;  // The number of digits after the decimal place
// const char *sonarGageHeightVarName = "gageHeight";  // This must be a value from http://vocabulary.odm2.org/variablename/
// const char *sonarGageHeightVarUnit = "Foot";  // This must be a value from http://vocabulary.odm2.org/units/
// const char *sonarGageHeightVarCode = "SonarGageHeight";  // A short code for the variable
// const char *sonarGageHeightVarUUID = "47dc1ffe-e092-4af7-ab26-140629b1cb60";
//
// // Create the calculated water pressure variable objects and return a variable pointer to it
// Variable *calculatedSonarGageHeight = new Variable(calculateSonarGageHeight, sonarGageHeightVarResolution,
//                                         sonarGageHeightVarName, sonarGageHeightVarUnit,
//                                         sonarGageHeightVarCode, sonarGageHeightVarUUID);


// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variables_create_in_array] */
// Version 1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
Variable* variableList[] = {
    // sonar1Range,
    // calculatedSonarGageHeight,
    // new YosemitechY520_Temp(&y520, "9e932e27-a3d6-499d-8106-bcd42b6d9085"),
    // new YosemitechY520_Cond(&y520, "09f14e53-b1ac-4aa5-a993-d3f0e8366250"),
    new ProcessorStats_Battery(&mcuBoard, "7a7e70d0-8b6d-44e8-a97f-dabb61c868a7"),
    new DecagonCTD_Cond(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    new DecagonCTD_Temp(&ctd,  "12345678-abcd-1234-ef00-1234567890ab"),
    new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    // new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    // new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
};
// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);
// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
/** End [variables_create_in_array] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// ==========================================================================
/** Start [monitormw] */
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char* registrationToken =
    "0144a65b-abdb-4cd2-a6c4-da1c244f891c";   // Device registration token
const char* samplingFeature =
    "683e9661-7c50-4bde-865f-7cef4efddc73";     // Sampling feature UUID

// Create a data publisher for the Monitor My Watershed/EnviroDIY POST endpoint
#include <publishers/EnviroDIYPublisher.h>
// EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient,
//                                  registrationToken, samplingFeature);
EnviroDIYPublisher EnviroDIYPOST(dataLogger,
                                registrationToken, samplingFeature);
/** End [monitormw] */


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

// Uses the processor sensor object to read the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
void setup() {
    /** Start [setup_wait] */
    // Wait for USB connection to be established by PC
    // NOTE:  Only use this when debugging - if not connected to a PC, this
    // could prevent the script from starting
    #if defined SERIAL_PORT_USBVIRTUAL
        while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000L)) {}
    #endif
    /** End [setup_wait] */

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
    // Serial.print(F("TinyGSM Library version "));
    // Serial.println(TINYGSM_VERSION);
    Serial.println();
    /** End [setup_prints] */

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
    // Start the serial connection with the modem
    // modemSerial.begin(modemBaud);

    // Start the stream for the modbus sensors;
    // all currently supported modbus sensors use 9600 baud
    // altSoftSerial.begin(9600);  // For all modbus serial streams

    // Start the SoftwareSerial stream for the sonar; it will always be at 9600
    // baud
    // sonarSerial.begin(9600);
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

    // Attach the modem and information pins to the logger
    // dataLogger.attachModem(modem);
    // modem.setModemLED(modemLEDPin);
    // dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
    //                          greenLED);

    // Begin the logger
    dataLogger.begin();
    /** End [setup_logger] */

    /** Start [setup_sesors] */
    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }
    /** End [setup_sesors] */

    // #if defined MS_BUILD_TEST_XBEE_CELLULAR
    //     /** Start [setup_xbeec_carrier] */
    //     // Extra modem set-up
    //     Serial.println(F("Waking modem and setting Cellular Carrier Options..."));
    //     modem.modemWake();  // NOTE:  This will also set up the modem
    //     // Go back to command mode to set carrier options
    //     modem.gsmModem.commandMode();
    //     // Carrier Profile - 0 = Automatic selection
    //     //                 - 1 = No profile/SIM ICCID selected
    //     //                 - 2 = AT&T
    //     //                 - 3 = Verizon
    //     // NOTE:  To select T-Mobile, you must enter bypass mode!
    //     modem.gsmModem.sendAT(GF("CP"), 2);
    //     modem.gsmModem.waitResponse();
    //     // Cellular network technology - 0 = LTE-M with NB-IoT fallback
    //     //                             - 1 = NB-IoT with LTE-M fallback
    //     //                             - 2 = LTE-M only
    //     //                             - 3 = NB-IoT only
    //     // NOTE:  As of 2020 in the USA, AT&T and Verizon only use LTE-M
    //     // T-Mobile uses NB-IOT
    //     modem.gsmModem.sendAT(GF("N#"), 2);
    //     modem.gsmModem.waitResponse();
    // // Write changes to flash and apply them
    //     Serial.println(F("Wait while applying changes..."));
    //     // Write changes to flash
    //     modem.gsmModem.writeChanges();
    //     // Reset the cellular component to ensure network settings are changed
    //     modem.gsmModem.sendAT(GF("!R"));
    //     modem.gsmModem.waitResponse(30000L);
    //     // Force reset of the Digi component as well
    //     // This effectively exits command mode
    //     modem.gsmModem.sendAT(GF("FR"));
    //     modem.gsmModem.waitResponse(5000L);
    //     /** End [setup_xbeec_carrier] */
    // #endif


    /** Start [setup_clock] */
    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.55 || !dataLogger.isRTCSane()) {
        // Synchronize the RTC with NIST
        // This will also set up the modem
        dataLogger.syncRTC();
    }
    /** End [setup_clock] */

    /** Start [setup_file] */
    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);
            // true = wait for card to settle after power up
        dataLogger.createLogFile(true); // true = write a new header
        dataLogger.turnOffSDcard(true);
            // true = wait for internal housekeeping after write
    }
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

/** Start [complex_loop] */
// Use this long loop when you want to do something special
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.

// We use this long to reset AltSoftSerial pins to LOW,
// to reduce RS485 adapter power bleed on sleep

void loop() {
    // Reset the watchdog
    dataLogger.watchDogTimer.resetWatchDog();

    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    // We're only doing anything at all if the battery is above 3.4V
    if (dataLogger.checkInterval() && getBatteryVoltage() > 3.4) {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;
        dataLogger.watchDogTimer.resetWatchDog();

        // Print a line to show new reading
        Serial.println(F("------------------------------------------"));
        // Turn on the LED to show we're taking a reading
        dataLogger.alertOn();
        // Power up the SD Card, but skip any waits after power up
        dataLogger.turnOnSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();

        // Turn on the modem to let it start searching for the network
        // Only turn the modem on if the battery at the last interval was high
        // enough
        // NOTE:  if the modemPowerUp function is not run before the
        // completeUpdate
        // function is run, the modem will not be powered and will not
        // return a signal strength reading.
        // if (getBatteryVoltage() > 3.6) modem.modemPowerUp();

        // Start the stream for the modbus sensors, if your
        // RS485 adapter "steals" current from data pins when powered off,
        // & you stop modbus serial connection with digitalWrite(5, LOW), below.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        // altSoftSerial.begin(9600);

        // Do a complete update on the variable array.
        // This this includes powering all of the sensors, getting updated
        // values, and turing them back off.
        // NOTE:  The wake function for each sensor should force sensor setup
        // to run if the sensor was not previously set up.
        varArray.completeUpdate();

        dataLogger.watchDogTimer.resetWatchDog();

        // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
        // on sleep, because Modbus Stop bit leaves these pins HIGH.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
        digitalWrite(5, LOW);   // Reset AltSoftSerial Tx pin to LOW
        digitalWrite(6, LOW);   // Reset AltSoftSerial Rx pin to LOW

        // Create a csv data record and save it to the log file
        dataLogger.logToSD();
        dataLogger.watchDogTimer.resetWatchDog();

        // Connect to the network
        // Again, we're only doing this if the battery is doing well
        // if (getBatteryVoltage() > 3.55) {
        //     dataLogger.watchDogTimer.resetWatchDog();
        //     if (modem.connectInternet()) {
        //         dataLogger.watchDogTimer.resetWatchDog();
        //         // Publish data to remotes
        //         Serial.println(F("Modem connected to internet."));
        //         dataLogger.publishDataToRemotes();
        //
        //         // Sync the clock at midnight
        //         dataLogger.watchDogTimer.resetWatchDog();
        //         if (Logger::markedEpochTime != 0 &&
        //             Logger::markedEpochTime % 86400 == 0) {
        //             Serial.println(F("Running a daily clock sync..."));
        //             dataLogger.setRTClock(modem.getNISTTime());
        //             dataLogger.watchDogTimer.resetWatchDog();
        //             modem.updateModemMetadata();
        //             dataLogger.watchDogTimer.resetWatchDog();
        //         }
        //
        //         // Disconnect from the network
        //         modem.disconnectInternet();
        //         dataLogger.watchDogTimer.resetWatchDog();
        //     }
        //     // Turn the modem off
        //     modem.modemSleepPowerDown();
        //     dataLogger.watchDogTimer.resetWatchDog();
        // }

        // Cut power from the SD card - without additional housekeeping wait
        dataLogger.turnOffSDcard(false);
        dataLogger.watchDogTimer.resetWatchDog();
        // Turn off the LED
        dataLogger.alertOff();
        // Print a line to show reading ended
        Serial.println(F("------------------------------------------\n"));

        // Unset flag
        Logger::isLoggingNow = false;
    }

    // Start the stream for the modbus sensors, if your
    // RS485 adapter "steals" current from data pins when powered off,
    // & you stop modbus serial connection with digitalWrite(5, LOW), below.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
    // altSoftSerial.begin(9600);

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting) dataLogger.testingMode();

    // Reset modbus serial pins to LOW, if your RS485 adapter bleeds power
    // on sleep, because Modbus Stop bit leaves these pins HIGH.
// https://github.com/EnviroDIY/ModularSensors/issues/140#issuecomment-389380833
    digitalWrite(5, LOW);   // Reset AltSoftSerial Tx pin to LOW
    digitalWrite(6, LOW);   // Reset AltSoftSerial Rx pin to LOW

    // Call the processor sleep
    dataLogger.systemSleep();
}
/** End [complex_loop] */
