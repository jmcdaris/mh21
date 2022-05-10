[//]: # ( @page example_mmw Monitor My Watershed Example )
# Using ModularSensors to log data to Monitor My Watershed/EnviroDIY

This sketch reduces menu_a_la_carte.ino to provide an example of how to log to https://monitormywatershed.org/ from two sensors, the BME280 and DS18. To complete the set up for logging to the web portal, the UUIDs for the site and each variable would need to be added to the sketch.

The settings for other data portals were removed from the example.

The modem settings were left unchanged because the sketch will test successfully without modem connection (wait patiently, it takes a few minutes).

This is the example you should use to deploy a logger with a modem to stream live data to the Monitor My Watershed data portal.

_______

[//]: # ( @tableofcontents )

[//]: # ( Start GitHub Only )
- [Using ModularSensors to log data to Monitor My Watershed/EnviroDIY](#using-modularsensors-to-log-data-to-monitor-my-watershedenvirodiy)
- [Unique Features of the Monitor My Watershed Example](#unique-features-of-the-monitor-my-watershed-example)
- [To Use this Example:](#to-use-this-example)
  - [Prepare and set up PlatformIO](#prepare-and-set-up-platformio)
  - [Set the logger ID](#set-the-logger-id)
  - [Set the universally universal identifiers (UUID) for each variable](#set-the-universally-universal-identifiers-uuid-for-each-variable)
  - [Upload!](#upload)

[//]: # ( End GitHub Only )

_______

[//]: # ( @section example_mmw_unique Unique Features of the Monitor My Watershed Example )
# Unique Features of the Monitor My Watershed Example
- A single logger publishes data to the Monitor My Watershed data portal.
- Uses a cellular Digi XBee or XBee3

[//]: # ( @section example_mmw_using To Use this Example: )
# To Use this Example:

[//]: # ( @subsection example_mmw_pio Prepare and set up PlatformIO )
## Prepare and set up PlatformIO
- Register a site and sensors at the Monitor My Watershed/EnviroDIY data portal (http://monitormywatershed.org/)
- Create a new PlatformIO project
- Replace the contents of the platformio.ini for your new project with the [platformio.ini](https://raw.githubusercontent.com/EnviroDIY/ModularSensors/master/examples/logging_to_MMW/platformio.ini) file in the examples/logging_to_MMW folder on GitHub.
    - It is important that your PlatformIO configuration has the lib_ldf_mode and build flags set as they are in the example.
    - Without this, the program won't compile.
- Open [logging_to_MMW.ino](https://raw.githubusercontent.com/EnviroDIY/ModularSensors/master/examples/logging_to_MMW/logging_to_MMW.ino) and save it to your computer.
    - After opening the link, you should be able to right click anywhere on the page and select "Save Page As".
    - Move it into the src directory of your project.
    - Delete main.cpp in that folder.

[//]: # ( @subsection example_mmw_logger_id Set the logger ID )
## Set the logger ID
- Change the "XXXX" in this section of code to the loggerID assigned by Stroud:

```cpp
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "XXXX";
```

[//]: # ( @subsection example_mmw_uuids Set the universally universal identifiers (UUID) for each variable )
## Set the universally universal identifiers (UUID) for each variable
- Go back to the web page for your site at the Monitor My Watershed/EnviroDIY data portal (http://monitormywatershed.org/)
- For each variable, find the dummy UUID (`"12345678-abcd-1234-ef00-1234567890ab"`) and replace it with the real UUID for the variable.

[//]: # ( @subsection example_mmw_upload Upload! )
## Upload!
- Test everything at home **before** deploying out in the wild!

_______

[//]: # ( @section example_mmw_pio PlatformIO Configuration )

[//]: # ( @include{lineno} logging_to_MMW/platformio.ini )

[//]: # ( @section example_mmw_code The Complete Code )




Here's what the .ini file should look like:
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; http://docs.platformio.org/page/projectconf.html

[platformio]
;src_dir = logging_to_MMW_calib
src_dir = logging_to_MMW_calib_MeterYosi

[env:mayfly]
monitor_speed = 115200
board = mayfly
platform = atmelavr
framework = arduino
lib_ldf_mode = deep+
lib_ignore =
    RTCZero
    Adafruit NeoPixel
    Adafruit GFX Library
    Adafruit SSD1306
    Adafruit ADXL343
    Adafruit STMPE610
    Adafruit TouchScreen
    Adafruit ILI9341
build_flags =
    -DSDI12_EXTERNAL_PCINT
    -DNEOSWSERIAL_EXTERNAL_PCINT
    -DMQTT_MAX_PACKET_SIZE=240
    -DTINY_GSM_RX_BUFFER=64
    -DTINY_GSM_YIELD_MS=2
    -DENABLE_SERIAL2
    -DENABLE_SERIAL3
    ; -D MS_BUILD_TEST_XBEE_CELLULAR  ; Turn on first time w/ a Digi LTE-M module
    ; -D MS_LOGGERBASE_DEBUG
    ; -D MS_DATAPUBLISHERBASE_DEBUG
    ; -D MS_ENVIRODIYPUBLISHER_DEBUG
lib_deps =
    envirodiy/EnviroDIY_ModularSensors@=0.32.2
;  ^^ Use this when working from a tagged release of the library
;     See tags at https://platformio.org/lib/show/1648/EnviroDIY_ModularSensors

;    https://github.com/EnviroDIY/ModularSensors.git#develop
;  ^^ Use this when if you want to pull from the develop branch

    https://github.com/PaulStoffregen/AltSoftSerial.git
    ; https://github.com/SRGDamia1/NeoSWSerial.git
    https://github.com/EnviroDIY/SoftwareSerial_ExternalInts.git
;  ^^ These are software serial port emulator libraries, you may not need them
