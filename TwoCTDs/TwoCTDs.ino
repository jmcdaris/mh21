// ==========================================================================
//  Meter Hydros 21 Conductivity, Temperature, and Depth Sensor
// ==========================================================================
/** Start [hydros21] */
#include <sensors/DecagonCTD.h>

const char*   CTDSDI12address_1   = "1";    // The SDI-12 Address of the CTD
const char*   CTDSDI12address_10   = "10";    // The SDI-12 Address of the CTD
const uint8_t CTDNumberReadings = 6;      // The number of readings to average
const int8_t  CTDPower = sensorPowerPin;  // Power pin (-1 if unconnected)
const int8_t  CTDData_1  = 7;               // The SDI12 data pin
const int8_t  CTDData_10  = 11;               // The SDI12 data pin

// Create a Decagon CTD sensor object
DecagonCTD ctd_1(*CTDSDI12address_1, CTDPower, CTDData_1, CTDNumberReadings);
DecagonCTD ctd_10(*CTDSDI12address_10, CTDPower, CTDData_10, CTDNumberReadings);

// Create conductivity, temperature, and depth variable pointers for the CTD
Variable* ctdCond_1 = new DecagonCTD_Cond(&ctd_1,"12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdTemp_1 = new DecagonCTD_Temp(&ctd_1, "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdDepth_1 = new DecagonCTD_Depth(&ctd_1, "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdCond_10 = new DecagonCTD_Cond(&ctd_10, "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdTemp_10 = new DecagonCTD_Temp(&ctd_10, "12345678-abcd-1234-ef00-1234567890ab");
Variable* ctdDepth_10 = new DecagonCTD_Depth(&ctd_10, "12345678-abcd-1234-ef00-1234567890ab");
        /** End [hydros21] */
