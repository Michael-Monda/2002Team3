#include <Arduino.h>

#if defined __WPI_OPEN__
//WPI Open
const char* ssid = "WPI-Open";
const char* password = "";

#elif defined __RBE_NETWORK__
//RBE
const char* ssid = "RBE";
const char* password = "elm69wisest16poisoned";

#endif

// put your robomqtt credientials here
const char* mqtt_server = "robomqtt.cs.wpi.edu";
#define mqtt_port 1883
#define MQTT_USER team3
#define MQTT_PASSWORD aero2344 