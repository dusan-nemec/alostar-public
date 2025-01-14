#ifndef COMM_H_
#define COMM_H_

#include <WiFi.h>
#include <NetworkUdp.h>

// device access point configuration
#define DEFAULT_DEVICE_ID     1
#define DEFAULT_AP_SSID       "WH_IMU1"
#define DEFAULT_AP_PWD        "12345678"

// default device configuration
#define DEFAULT_DEV_SSID      "KRIS_AB205"
#define DEFAULT_DEV_PWD       "robot205"

// default UDP configuration
#define DEFAULT_UDP_HOST      "158.193.224.133"
#define DEFAULT_UDP_PORT      1234

// default NTP configuration
#define DEFAULT_NTP_HOST      "pool.ntp.org"
#define DEFAULT_NTP_OFFSET    1 // GMT+1
#define DEFAULT_NTP_DAYLIGHT  1 // use daylight

// Size of one record is exactly 32 + 64 = 96 bytes including null termination
#define SSID_LENGTH_MAX     31
#define PWD_LENGTH_MAX      63
#define HOST_LENGTH_MAX     31

struct CommSettings
{
  uint32_t deviceID;

  char apSSID[SSID_LENGTH_MAX + 1];
  char apPWD[PWD_LENGTH_MAX + 1];

  char devSSID[SSID_LENGTH_MAX + 1];
  char devPWD[PWD_LENGTH_MAX + 1];

  char udpHost[HOST_LENGTH_MAX + 1];
  uint16_t udpPort;

  char ntpHost[HOST_LENGTH_MAX + 1];
  uint32_t ntpGmtOffset;
  uint32_t ntpDaylightOffset;
  // TODO
};

CommSettings commSettings = {
  DEFAULT_DEVICE_ID, 
  DEFAULT_AP_SSID, DEFAULT_AP_PWD, 
  DEFAULT_DEV_SSID, DEFAULT_DEV_PWD, 
  DEFAULT_UDP_HOST, DEFAULT_UDP_PORT,
  DEFAULT_NTP_HOST, DEFAULT_NTP_OFFSET, DEFAULT_NTP_DAYLIGHT
};

#define AP_CAPACITY 20

// Access point descriptor
struct AccessPoint
{
  String ssid;
  int32_t rssi;
};

// Array of discovered access points
AccessPoint accessPoints[AP_CAPACITY];
int accessPointsCount = 0;
bool searchAccessPoints = true;
uint32_t lastConnectAttempt;
volatile uint8_t attemptsCount = 0;
volatile bool connected = false;
volatile bool apMode = false;

// TCP/IP socket opened on port 80
NetworkServer server(80);
NetworkClient client;
HttpRequest httpRequest;

//The udp library class
NetworkUDP udp;
IPAddress ip;

void initNTP()
{
  // set NTP server
  configTime(commSettings.ntpGmtOffset * 3600, commSettings.ntpDaylightOffset * 3600, commSettings.ntpHost);
}

void WiFiEvent(WiFiEvent_t event) 
{
  switch (event) 
  {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      ip = WiFi.localIP();
      if(connected)
        break;

      Serial.print("WiFi connected! IP address: ");
      Serial.println(ip);
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(ip, commSettings.udpPort);
      server.begin();

      connected = true;
      attemptsCount = 0;
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.");
      connected = false;
      break;

    default: 
      break;
  }
}

// connects / reconnects to the best available WiFi in DEV mode (return true)
// when it fails, return false
bool connectWiFi()
{
  Serial.println("Connecting to WiFi network: " + String(commSettings.devSSID));

  udp.stop();
  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.

  //Initiate connection
  WiFi.setHostname(commSettings.apSSID);
  WiFi.begin(commSettings.devSSID, commSettings.devPWD);
  apMode = false;

  Serial.println("Waiting for WIFI connection...");
  lastConnectAttempt = millis();
  attemptsCount = attemptsCount + 1;
  return true;  
}

void scanWiFi()
{
  // searches available networks
  Serial.println("Searching for networks ...");
  int n = WiFi.scanNetworks();
  Serial.println("Found networks:");
  accessPointsCount = 0;
  for(int i = 0; i < n && accessPointsCount < AP_CAPACITY; ++i)
  {
    String ssid = WiFi.SSID(i);
    Serial.print(ssid);
    Serial.print(" (");
    Serial.print(WiFi.RSSI(i));
    Serial.println(" dBm)");
    
    // save found AP
    accessPoints[accessPointsCount].ssid = ssid;
    accessPoints[accessPointsCount].rssi = WiFi.RSSI(i);
    accessPointsCount++;
  }
}

bool startAccessPoint()
{
  Serial.println("Starting own access point...");

  // delete old config
  WiFi.disconnect(true);

  bool res = WiFi.softAP(commSettings.apSSID, commSettings.apPWD);
  
  if(res)
  {
    Serial.println("... done.");
    server.begin();
    apMode = true;
  }
    
  return res;
}


#endif