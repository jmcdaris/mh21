/** =========================================================================
 * @file logging_to_MMW.ino
 * @brief Example logging data and publishing to Monitor My Watershed.
 *
 * @author Sara Geleskie Damiano <sdamiano@stroudcenter.org>
 * @copyright (c) 2017-2020 Stroud Water Research Center (SWRC)
 *                          and the EnviroDIY Development Team
 *            This example is published under the BSD-3 license.
 *
 * Build Environment: Visual Studios Code with PlatformIO
 * Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
 *
 * DISCLAIMER:
 * THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
 * ======================================================================= */

// ==========================================================================
//  Defines for the Arduino IDE
//  NOTE:  These are ONLY needed to compile with the Arduino IDE.
//         If you use PlatformIO, you should set these build flags in your
//         platformio.ini
// ==========================================================================
/** Start [defines] */
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif
/** End [defines] */

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
//  Data Logging Options
// ==========================================================================
/** Start [logging_options] */
// The name of this program file
const char* sketchName = "logging_to_MMW_calib.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char* LoggerID = "20351";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 5;
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
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin   = -1;  // MCU SD card power pin
const int8_t sdCardSSPin    = 12;  // SD card chip select/slave select pin
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power
/** End [logging_options] */


// ==========================================================================
//  Wifi/Cellular Modem Options
// ==========================================================================
/** Start [xbee_cell_transparent] */
// For any Digi Cellular XBee's
// NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global) can be used
// in either bypass or transparent mode, each with pros and cons
// The Telit based Digi XBees (LTE Cat1) can only use this mode.
#include <modems/DigiXBeeCellularTransparent.h>

// Create a reference to the serial port for the modem
HardwareSerial& modemSerial = Serial1;  // Use hardware serial if possible
const int32_t   modemBaud   = 9600;     // All XBee's use 9600 by default

// Modem Pins - Describe the physical pin connection of your modem to your board
// NOTE:  Use -1 for pins that do not apply
const int8_t modemVccPin    = -2;     // MCU pin controlling modem power
const int8_t modemStatusPin = 19;     // MCU pin used to read modem status
const bool useCTSforStatus  = false;  // Flag to use the XBee CTS pin for status
const int8_t modemResetPin  = 20;     // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;    // MCU pin for modem sleep/wake request
const int8_t modemLEDPin = redLED;    // MCU pin connected an LED to show modem
                                      // status (-1 if unconnected)

// Network connection information
const char* apn = "hologram";  // The APN for the gprs connection

// NOTE:  If possible, use the `STATUS/SLEEP_not` (XBee pin 13) for status, but
// the `CTS` pin can also be used if necessary
DigiXBeeCellularTransparent modemXBCT(&modemSerial, modemVccPin, modemStatusPin,
                                      useCTSforStatus, modemResetPin,
                                      modemSleepRqPin, apn);
// Create an extra reference to the modem by a generic name
DigiXBeeCellularTransparent modem = modemXBCT;
/** End [xbee_cell_transparent] */


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

const char*   CTDSDI12address_1   = "1";    // The SDI-12 Address of the QWTA CTD
const char*   CTDSDI12address_7   = "7";    // The SDI-12 Address of the OPVL CTD
const uint8_t CTDNumberReadings = 6;      // The number of readings to average
const int8_t  CTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
const int8_t  CTDData_1  = 7;               // The SDI12 data pin
const int8_t  CTDData_7  = 11;               // The SDI12 data pin

// const char*   CTDSDI12address   = "1";    // The SDI-12 Address of the CTD
// const uint8_t CTDNumberReadings = 6;      // The number of readings to average
// const int8_t  CTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
// const int8_t  CTDData  = 7;               // The SDI12 data pin

// Create a Decagon CTD sensor object
// DecagonCTD ctd(*CTDSDI12address, CTDPower, CTDData, CTDNumberReadings);
DecagonCTD ctd_1(*CTDSDI12address_1, CTDPower, CTDData_1, CTDNumberReadings);
DecagonCTD ctd_7(*CTDSDI12address_7, CTDPower, CTDData_7, CTDNumberReadings);

// Create conductivity, temperature, and depth variable pointers for the CTD
// Create calculated variable pointer
Variable *cond1 = new DecagonCTD_Cond(&ctd_1,"8223ee2b-69dc-4b8d-ba0a-618a0c1be623");
Variable *cond2 = new DecagonCTD_Cond(&ctd_7, "cab179bc-0e05-4bae-a2db-0a9bd20fb8e1");


/** End [hydros21] */

// ==========================================================================
//    Calculated Variables
// ==========================================================================

// Create the function to give your calculated result.
// The function should take no input (void) and return a float.
// You can use any named variable pointers to access values by way of variable->getValue()

// Create the function to calculate water level / gage height variable
float calibrateConductivity1(void) //QWTA sensor
{
    float calibConductivity1 = -9999;  // Always safest to start with a bad value
    const float ConductivityK1 = 0.9566;   // slope from conductivity calibration spreadsheet
    const float ConductivityB1 = -2.048;   // intercept from conductivity calibration spreadsheet (mS/cm)
    float condMeasured1 = cond1->getValue();
    if (condMeasured1 != -9999)  // make sure all inputs are good
    {
        calibConductivity1 = ConductivityK1*condMeasured1+ConductivityB1; // applying equation from calibration spreadsheet
    }
    return calibConductivity1;
}

// Properties of the calculated water level / gage height variable
const uint8_t conductivityVarResolution = 1;  // The number of digits after the decimal place
const char *conductivityVarName = "specificConductance";  // This must be a value from http://vocabulary.odm2.org/variablename/
const char *conductivityVarUnit = "microsiemenPerCentimeter";  // This must be a value from http://vocabulary.odm2.org/units/
const char *conductivityVarCode1 = "calibConductivity1";  // A short code for the variable
const char *conductivityVarUUID1 = "39e2c734-2462-4015-9541-4d3d42cdfb03"; //Calibarted SC for CTD1

// Create the calculated water pressure variable objects and return a variable pointer to it
Variable *calibcond1 = new Variable(calibrateConductivity1, conductivityVarResolution,
                                        conductivityVarName, conductivityVarUnit,
                                        conductivityVarCode1, conductivityVarUUID1);

float calibrateConductivity2(void) //OPVL sensor
{
    float calibConductivity2 = -9999;  // Always safest to start with a bad value
    const float ConductivityK2 = 1.00412;   // slope from conductivity calibration spreadsheet
    const float ConductivityB2 = 8.365;   // intercept from conductivity calibration spreadsheet (mS/cm)
    float condMeasured2 = cond2->getValue();
    if (condMeasured2 != -9999)  // make sure all inputs are good
    {
        calibConductivity2 = ConductivityK2*condMeasured2+ConductivityB2; // applying equation from calibration spreadsheet
    }
    return calibConductivity2;
}

// Properties of the calculated water level / gage height variable
// const uint8_t conductivityVarResolution = 1;  // The number of digits after the decimal place
// const char *conductivityVarName = "specificConductance";  // This must be a value from http://vocabulary.odm2.org/variablename/
// const char *conductivityVarUnit = "microsiemenPerCentimeter";  // This must be a value from http://vocabulary.odm2.org/units/
const char *conductivityVarCode2 = "calibConductivity2";  // A short code for the variable
const char *conductivityVarUUID2 = "3d7cb05a-6505-4a84-b927-9277bef4ed51";

// Create the calculated water pressure variable objects and return a variable pointer to it
Variable *calibcond2 = new Variable(calibrateConductivity2, conductivityVarResolution,
                                        conductivityVarName, conductivityVarUnit,
                                        conductivityVarCode2, conductivityVarUUID2);



// ==========================================================================
//  Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================
/** Start [variable_arrays] */
Variable* variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard,
                                    "12345678-abcd-1234-ef00-1234567890ab"),
    // new DecagonCTD_Depth(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    // new DecagonCTD_Temp(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    // new DecagonCTD_Cond(&ctd, "12345678-abcd-1234-ef00-1234567890ab"),
    cond1,
    calibcond1,
    new DecagonCTD_Temp(&ctd_1, "2fdcdd5e-37c0-4520-ac7e-dcd00043e7e7"),
    new DecagonCTD_Depth(&ctd_1, "2c01b840-3247-4503-a594-88cfd0780b8d"),
    cond2,
    calibcond2,
    new DecagonCTD_Temp(&ctd_7, "9441ff2b-a5cd-4e02-9e55-6ee5d8248eec"),
    new DecagonCTD_Depth(&ctd_7, "7d6b7963-decb-43c9-b098-21401e4ccae5"),
    new ProcessorStats_Battery(&mcuBoard,
                               "12345678-abcd-1234-ef00-1234567890ab"),
    new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab"),
    new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
    new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab"),
};


// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
/** End [variable_arrays] */


// ==========================================================================
//  The Logger Object[s]
// ==========================================================================
/** Start [loggers] */
// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);
/** End [loggers] */


// ==========================================================================
//  Creating Data Publisher[s]
// ==========================================================================
/** Start [publishers] */
// A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char* registrationToken =
    "02233e84-30a4-4037-aa92-b81512ed9e89";  // Device registration token
const char* samplingFeature =
    "1f88b364-b06c-400e-bc1d-14f0d3e724a9";  // Sampling feature UUID

// Create a data publisher for the Monitor My Watershed/EnviroDIY POST endpoint
#include <publishers/EnviroDIYPublisher.h>
EnviroDIYPublisher EnviroDIYPOST(dataLogger, &modem.gsmClient,
                                 registrationToken, samplingFeature);
/** End [publishers] */


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

// Reads the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage() {
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}
/** End [working_functions] */


// ==========================================================================
//  Arduino Setup Function
// ==========================================================================
/** Start [setup] */
void setup() {
// Wait for USB connection to be established by PC
// NOTE:  Only use this when debugging - if not connected to a PC, this
// could prevent the script from starting
#if defined SERIAL_PORT_USBVIRTUAL
    while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)) {}
#endif

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);
    Serial.print(F("TinyGSM Library version "));
    Serial.println(TINYGSM_VERSION);
    Serial.println();

// Allow interrupts for software serial
#if defined SoftwareSerial_ExtInts_h
    enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt,
                    CHANGE);
#endif
#if defined NeoSWSerial_h
    enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
#endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin,
                             greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }

    // Sync the clock if it isn't valid or we have battery to spare
    if (getBatteryVoltage() > 3.55 || !dataLogger.isRTCSane()) {
        // Synchronize the RTC with NIST
        // This will also set up the modem
        dataLogger.syncRTC();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4) {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(
            true);  // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(
            true);  // true = wait for internal housekeeping after write
    }

    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
}
/** End [setup] */


// ==========================================================================
//  Arduino Loop Function
// ==========================================================================
/** Start [loop] */
// Use this short loop for simple data logging and sending
void loop() {
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < 3.4) {
        dataLogger.systemSleep();
    }
    // At moderate voltage, log data but don't send it over the modem
    else if (getBatteryVoltage() < 3.55) {
        dataLogger.logData();
    }
    // If the battery is good, send the data to the world
    else {
        dataLogger.logDataAndPublish();
    }
}
/** End [loop] */
