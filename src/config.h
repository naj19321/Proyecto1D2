/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "Naj19321"
#define IO_KEY       "aio_sPPe43op33JLMDwsHnrgKd5YjPlN"

/******************************* WIFI **************************************/

/*#define WIFI_SSID "CLARO1_F525A9"
#define WIFI_PASS "L5928miEvq"*/

#define WIFI_SSID "Saenz"
#define WIFI_PASS "saen1999"

#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

