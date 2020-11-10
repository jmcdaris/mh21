# Deployment Notes

#### SiteCode: CLFL-stream-1
https://monitormywatershed.org/sites/CLFL-stream-1/


#### Library Dependencies (in PlatformIO.ini)

```.ini
build_flags =
    -DSDI12_EXTERNAL_PCINT
    -DNEOSWSERIAL_EXTERNAL_PCINT
    -DMQTT_MAX_PACKET_SIZE=240
    -D MS_BUILD_TEST_XBEE_CELLULAR
lib_deps =
    EnviroDIY_ModularSensors@=0.25.0
    https://github.com/PaulStoffregen/AltSoftSerial.git
    https://github.com/SRGDamia1/NeoSWSerial.git
    https://github.com/EnviroDIY/SoftwareSerial_ExternalInts.git
```

#### Logger Settings

```C++
```

#### Device Addresses/Pins:

```C++
```


#### Registration tokens:

```C++
const char *REGISTRATION_TOKEN = "0144a65b-abdb-4cd2-a6c4-da1c244f891c";   // Device registration token
const char *SAMPLING_FEATURE = "683e9661-7c50-4bde-865f-7cef4efddc73";     // Sampling feature UUID
const char *UUIDs[] =                                                      // UUID array for device sensors
{
    "47dc1ffe-e092-4af7-ab26-140629b1cb60",   // Gage height (All_Calc_gageHeight)
    "9805e438-9c06-405e-b937-fd88aeda96af",   // Distance (MaxBotix_MB7389_Distance)
    "09f14e53-b1ac-4aa5-a993-d3f0e8366250",   // Electrical conductivity (YosemiTech_Y520-A_Cond)
    "9e932e27-a3d6-499d-8106-bcd42b6d9085",   // Temperature (YosemiTech_Y520-A_Temp)
    "7a7e70d0-8b6d-44e8-a97f-dabb61c868a7"    // Battery voltage (EnviroDIY_Mayfly_Batt)
};
```
