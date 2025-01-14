#include <Preferences.h>
#include "HttpRequest.h"
#include "Board.h"
#include "Comm.h"
#include "Sensors.h"

/*// EEPROM addresses
#define EEA_FIRST_RUN         0
#define EEA_COMM_SETTINGS     (EEA_FIRST_RUN + 4)
#define EEA_SENSOR_SETTINGS   (EEA_COMM_SETTINGS  + sizeof(CommSettings))*/

volatile uint8_t ledMode = 0; // 0 = OFF, 1 = ON, 2 = BLINK
uint32_t lastLedBlink = 0;
Preferences config;

void setup() 
{
  Serial.begin(115200);
  initConfig();
  scanWiFi();
  connectWiFi();
  initNTP();

  xTaskCreatePinnedToCore(
    runSensors,    // Function to be called
    "runSensors",  // Name of task
    4096,           // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,           // Parameter to pass
    1,              // Task priority
    NULL,           // Task handle
    1);             // Run on one core 1

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  ledMode = 0;
  lastLedBlink = millis();
}

void loop() 
{
  if(connected)
  {
    ledMode = 1;
    if(!fifo.isEmpty())
    {
      // sample(s) available
      udp.beginPacket(commSettings.udpHost, commSettings.udpPort);
      while(!fifo.isEmpty())
      {
        Sample s = fifo.get();
        
        //char buff[256];
        //sprintf(buff, "%d;%s;%lld;%f;%f;%f;%f;%f;%f;%f\r", commSettings.deviceID, ip.toString().c_str(), s.tim, s.temperature, s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]); // debug
        //Serial.println(buff); // debug
        
        udp.printf("%d;%lld.%lld;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\r\n", 
          commSettings.deviceID, s.tim / 1000000L, s.tim % 1000000L, s.temperature, 
          s.gyro[0], s.gyro[1], s.gyro[2], 
          s.accel[0], s.accel[1], s.accel[2], 
          s.distance[0], s.distance[1], s.distance[2],
          s.distanceErr[0], s.distanceErr[1], s.distanceErr[2]
        );
      }
      udp.endPacket();
    }
  }

  else if(attemptsCount >= 5)
  {
    // failed to connect 5-times, go to AP mode
    if(!apMode)
      startAccessPoint();

    ledMode = 2;
  }

  else if(millis() > lastConnectAttempt + 5000)
  {
    // try to reconnect
    connectWiFi();
    ledMode = 0;
  }

  if (client.connected())
  {
    while(client.available())
    {
      byte b = client.read();
      if(httpRequest.parse(b))
        handleHttpRequest();
    }
  }
    
  // Listenning for new clients
  NetworkClient newClient = server.accept();
  if(newClient)
  {
    // close old client, connect new client
    client.flush();
    client.stop();

    client = newClient;
  }

  // LED visualisation
  if(ledMode == 0)
  {
    // off
    digitalWrite(LED, LOW);
  }
  else if(ledMode == 1)
  {
    // on
    digitalWrite(LED, HIGH);
  }
  else if(ledMode == 2)
  {
    // blink
    if(millis() - lastLedBlink > 500)
    {
      digitalWrite(LED, digitalRead(LED) ? LOW : HIGH);
      lastLedBlink = millis();
    }
  }
}

void handleHttpRequest()
{
  String body;
  bool doRefresh = false;
  bool doConnect = false;
  bool doSave = false;

  body.reserve(4096);
  
  body += "<html>\n"
    "<head>\n"
      "<title>Setup</title>\n"
      "<link rel=\"icon\" href=\"data:;base64,iVBORw0KGgo=\">\n"
      "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
    "</head>"
    "<body>";
   
  if(httpRequest.method() == "GET")
  {
    // write basic config page
    body += "<h1>Setup</h1>\n"
      "<form method=\"post\" action=\"\">\n"
        "<h2>Client setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"dev_id\">Device ID:</label>\n"
          "<input type=\"number\" id=\"dev_id\" name=\"dev_id\" min=\"1\" max=\"65535\" value=\"" + String(commSettings.deviceID) + "\" required>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"dev_ssid\">Network:</label>\n"
          "<select id=\"dev_ssid\" name=\"dev_ssid\" required>\n";

    for(int i=0; i < accessPointsCount; i++)
    {
      const AccessPoint& ap = accessPoints[i];
      body += "<option value=\"" + ap.ssid + "\"" + (ap.ssid == WiFi.SSID() ? " selected" : "") + ">" + accessPoints[i].ssid + " (" + accessPoints[i].rssi + "dBm)</option>\n";
    }
   
    body += "</select>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"current_ip\">Current IP:</label>\n"
          "<input type=\"text\" id=\"current_ip\" name=\"current_ip\" value=\"" + ip.toString() + "\" disabled>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<input type=\"checkbox\" id=\"set_dev_pwd\" name=\"set_dev_pwd\" value=\"1\" onchange=\"document.getElementById('dev_pwd').disabled = !this.checked;\">\n"
          "<label for=\"set_dev_pwd\">Set password</label>\n"
          "<input type=\"text\" id=\"dev_pwd\" name=\"dev_pwd\" minlength=\"8\" placeholder=\"new password...\" maxlength=\"63\" disabled>\n"
        "</div>\n"
        "<h2>Access point setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"ap_ssid\">SSID:</label>\n"
          "<input type=\"text\" id=\"ap_ssid\" name=\"ap_ssid\" value=\"" + commSettings.apSSID + "\" maxlength=\"31\">\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<input type=\"checkbox\" id=\"set_ap_pwd\" name=\"set_ap_pwd\" value=\"1\" onchange=\"document.getElementById('ap_pwd').disabled = !this.checked;\">\n"
          "<label for=\"set_ap_pwd\">Set password</label>\n"
          "<input type=\"text\" id=\"ap_pwd\" name=\"ap_pwd\" minlength=\"8\" placeholder=\"new password...\" maxlength=\"63\" disabled>\n"
        "</div>\n"
        "<h2>UDP target setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"udp_host\">Host:</label>\n"
          "<input type=\"text\" id=\"udp_host\" name=\"udp_host\" value=\"" + commSettings.udpHost + "\" maxlength=\"31\" required>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"udp_port\">Port:</label>\n"
          "<input type=\"number\" id=\"udp_port\" name=\"udp_port\" min=\"1\" max=\"65535\" value=\"" + String(commSettings.udpPort) + "\" required>\n"
        "</div>\n"
        "<h2>NTP setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"ntp_host\">Host:</label>\n"
          "<input type=\"text\" id=\"ntp_host\" name=\"ntp_host\" value=\"" + commSettings.ntpHost + "\" maxlength=\"31\" required>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"ntp_offset\">GMT offset:</label>\n"
          "<input type=\"number\" id=\"ntp_offset\" name=\"ntp_offset\" min=\"-12\" max=\"12\" value=\"" + String(commSettings.ntpGmtOffset) + "\">\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"ntp_daylight\">Daylight offset:</label>\n"
          "<input type=\"number\" id=\"ntp_daylight\" name=\"ntp_daylight\" min=\"-1\" max=\"1\" value=\"" + String(commSettings.ntpDaylightOffset) + "\">\n"
        "</div>\n"
        "<h2>IMU setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"gyro_fs\">Gyroscope full-scale (deg/s):</label>\n"
          "<input type=\"number\" id=\"gyro_fs\" name=\"gyro_fs\" min=\"125\" max=\"4000\" value=\"" + String(sensorSettings.gyroFS) + "\" required>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"accel_fs\">Accelerometer full-scale (g):</label>\n"
          "<input type=\"number\" id=\"accel_fs\" name=\"accel_fs\" min=\"2\" max=\"16\" value=\"" + String(sensorSettings.accelFS) + "\" required>\n"
        "</div>\n"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"imu_odr\">IMU data rate (Hz):</label>\n"
          "<input type=\"number\" id=\"imu_odr\" name=\"imu_odr\" min=\"12.5\" max=\"6667.0\" step=\"0.5\" value=\"" + String(sensorSettings.imuODR) + "\">\n"
        "</div>\n"
        "<h2>LiDAR setup</h2>"
        "<div style=\"padding: 6px;\">\n"
          "<label for=\"lidar_period\">Sampling period (ms):</label>\n"
          "<input type=\"number\" id=\"lidar_period\" name=\"lidar_period\" min=\"10\" max=\"200\" value=\"" + String(sensorSettings.lidarPeriod) + "\" required>\n"
        "</div>\n"
        "<p style=\"color: red;\">Warning: after submitting, the sensor will disconnect.</p>\n"
        "<input type=\"submit\" name=\"save\" value=\"Save changes\">\n"
        "<input type=\"submit\" name=\"refresh\" value=\"Refresh list\">\n"
        "<input type=\"submit\" name=\"connect\" value=\"Connect to selected network\">\n"
      "</form>\n";
  }
  else if(httpRequest.method() == "POST")
  {
    // obtain selected SSID and optionally change password
    if(httpRequest.arg("save") != "")
    {
      commSettings.deviceID = httpRequest.arg("dev_id").toInt();
      strcpy(commSettings.devSSID, httpRequest.arg("dev_ssid").c_str());

      if(httpRequest.arg("set_dev_pwd") != "")
        strcpy(commSettings.devPWD, httpRequest.arg("dev_pwd").c_str());

      strcpy(commSettings.apSSID, httpRequest.arg("ap_ssid").c_str());

      if(httpRequest.arg("set_ap_pwd") != "")
        strcpy(commSettings.apPWD, httpRequest.arg("ap_pwd").c_str());

      strcpy(commSettings.udpHost, httpRequest.arg("udp_host").c_str());
      commSettings.udpPort = httpRequest.arg("udp_port").toInt();

      strcpy(commSettings.ntpHost, httpRequest.arg("ntp_host").c_str());
      commSettings.ntpGmtOffset = httpRequest.arg("ntp_offset").toInt();
      commSettings.ntpDaylightOffset = httpRequest.arg("ntp_daylight").toInt();
      
      initNTP();

      sensorSettings.gyroFS = httpRequest.arg("gyro_fs").toInt();
      sensorSettings.accelFS = httpRequest.arg("accel_fs").toInt();
      sensorSettings.imuODR = httpRequest.arg("imu_odr").toFloat();
      sensorSettings.lidarPeriod = httpRequest.arg("lidar_period").toInt();
      sensorSettingsChanged = true;

      body += "<h1 style=\"color: orange;\">Saving the changes ...</h1>\n";
      doSave = true;
    }

    else if(httpRequest.arg("refresh") != "")
    {
      // refreshing the list of networks - disconnects first
      body += "<h1 style=\"color: red;\">Refreshing the list of networks ...</h1>\n";
      doRefresh = true;
    }
    else if(httpRequest.arg("connect") != "")
    {
      body += "<h1 style=\"color: red;\">Connecting to " + String(commSettings.devSSID) + "</h1>\n";
      doConnect = true;
    }
  }

  body += "</body>\n"
    "</html>";

  client.println("HTTP/1.1 200 OK");
  
  if(httpRequest.method() == "POST")
    client.println("Refresh: 10;url=/");

  client.println("Content-Type: text/html");
  client.println("Connection: keep-alive");  
  client.print("Content-Length: ");
  client.println(body.length());
  client.println();
  
  client.print(body);
  client.flush();

  if(doSave)
  {
    saveConfig();
  }
  else if(doRefresh)
  {
    client.stop();
    scanWiFi();
  }
  else if(doConnect)
  {
    client.stop();
    connectWiFi();
  }
}

void initConfig()
{
  config.begin("conf", false);
  if(config.isKey("ready"))
  {
    // use previously stored values
    Serial.println("Loading config from EEPROM...");
    loadConfig();
  }
  else
  {
    // nothing has been saved into EEPROM yet
    Serial.println("Initializing EEPROM...");
    saveConfig();
  }
}

void saveConfig()
{
  config.putBytes("comm", &commSettings, sizeof(CommSettings));
  config.putBytes("sens", &sensorSettings, sizeof(SensorSettings));
  config.putBool("ready", true);
}

void loadConfig()
{
  config.getBytes("comm", &commSettings, sizeof(CommSettings));
  config.getBytes("sens", &sensorSettings, sizeof(SensorSettings));
}

