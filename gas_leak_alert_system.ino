/* 
  Author: 	Aditya Khot
  Date: 	14/3/2023
  Version: 	1.0
  Purpose: 	This code will send an alert to the user if the load cell detects 
  a strain greater than the threshold or if the MQ sensor detects a gas leak. 
  The code will also display the strain and gas level on the LCD display. 
  The code will also send the strain and gas level to Thingspeak. The code 
  will also send the strain and gas level to the user via SMS.
  
 */


// include libraries
// include wire library to communicate with the LCD display
#include <Wire.h>
// include liquid crystal library to display data on the LCD display
#include <LiquidCrystal_I2C.h>
// include HX711 library to communicate with the load cell
#include "HX711.h"

// define variables
LiquidCrystal_I2C lcd(0x27, 16, 2);

// define pinouts for load cell
#define DOUT 15
#define CLK 18

// define pinouts for MQ sensor
#define MQ 2

// define variables for load cell
// calibration factor is the ratio of the measured value at known load to the actual load in kilograms
float calibration_factor = -234560;
// zero factor is the measured value when the load cell is not under any load
float zero_factor = 8295405;
// strain threshold is the amount of strain that will trigger an alert
float strainThreshold = 1.50;

// define variables for MQ sensor
int initialMQValue = 0;
int MQValueInByte = 0;
// MQ threshold is the amount of gas that will trigger an alert
int MQThreshold = 1000;

bool thresholdReached = false;

// define variables for HX711 scale
HX711 scale(DOUT, CLK);

// if sim lock is enabled, enter the sim pin here
const char simPIN[] = "";

// SMS target number (must be in international format) e.g. +00 000 000 0000
#define TARGET_NUMBER "+918956707945"


// get the API key from ThingSpeak.com.
String apiKeyValue = "CRK1BLP31PCG2RHE";

// Server details
// The server variable can be just a domain name or it can have a subdomain. It depends on the service you are using
const char server[] = "thingspeak.com";  // domain name: example.com, maker.ifttt.com, etc
const char resource[] = "http://api.thingspeak.com/update?api_key=CRK1BLP31PCG2RHE";
const int port = 80;  // server port number

// GPRS credentials (leave empty, if not needed)
const char apn[] = "WWW";    // APN (example: airtelgprs.com)
const char gprsUser[] = "";  // GPRS User
const char gprsPass[] = "";  // GPRS Password

#define TINY_GSM_MODEM_SIM800    // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>

// define pinouts for TTGO T-Call SIM800L module
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

// define serial monitor and serial AT
#define SerialMon Serial
#define SerialAT Serial1

// define baud rate for serial monitor
#define BAUD_RATE 115200

// define debug mode
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// TinyGSM Client for Internet connection
TinyGsmClient client(modem);

// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);


#define uS_TO_S_FACTOR 1000000UL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 3600       /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */


// define TinyGsmClient
#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

// function to send data to Thingspeak
bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);

  if (en) {
    Wire.write(0x37);
  } else {
    Wire.write(0x35);
  }
  return Wire.endTransmission() == 0;
}

class IntervalTimer {
private:
  unsigned long previousMillis;
  unsigned long interval;

public:
  IntervalTimer(unsigned long _interval) {
    interval = _interval;
    previousMillis = millis();
  }

  template<typename Function, typename... Args>
  void run(Function&& func, Args&&... args) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      (*func)(std::forward<Args>(args)...);
      previousMillis = currentMillis;
    }
  }
};



IntervalTimer sendDatatoCloud(60000);  // run every half minute

void setup() {
  // begin serial monitor.
  SerialMon.begin(BAUD_RATE);

  // begin lcd display and turn backlight on
  lcd.begin();
  lcd.backlight();
  lcd.home();

  // initiate scale to default and then set defined offset
  scale.set_scale();
  scale.set_offset(zero_factor);

  // initiate wire library to start I2C and communicate with LCD display
  Wire.begin(I2C_SDA, I2C_SCL);

  // start by clearing lcd display
  lcd.clear();

  // if the IP5306 is not ok, display an error message
  bool isOk = setPowerBoostKeepOn(1);
  lcd.print(String("IP5306: ") + (isOk ? "OK" : "FAIL"));

  // wait 1 second before running the next line of code
  delay(1000);

  // pinouts and modes defined
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // clear lcd display
  lcd.clear();

  // print initializing message on lcd display
  lcd.print("Initializing...");

  // start TTGO T-Call SIM800L module and it's baud rate
  SerialAT.begin(BAUD_RATE, SERIAL_8N1, MODEM_RX, MODEM_TX);

  // wait 3 seconds before running the next line of code
  delay(3000);

  lcd.clear();
  lcd.print("Restarting Modem");

  // restart TTGO T-Call SIM800L module
  modem.restart();

  // if sim lock is enabled, unlock the sim card with the sim pin defined above
  // if sim lock is disabled, skip this step
  if (strlen(simPIN) && modem.getSimStatus() != 3) {
    modem.simUnlock(simPIN);
  }

  while (!SerialMon) { ; }

  // print initializing message on lcd display
  lcd.clear();
  lcd.print("Starting...");

  // wait 1 seconds before running the next line of code
  delay(1000);
}

// This function will send the strain and MQ sensor values to ThingSpeak
void sendDataToThingSpeak(String str1, String str2) {
  // lcd.clear();

  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);

  // lcd.setCursor(0, 0);
  // lcd.print("Connecting...");

  // lcd.setCursor(0, 1);
  // lcd.print("APN: ");

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("fail");

    // lcd.setCursor(0, 1);
    // lcd.print("APN: ");
    // lcd.print("fail");
  } else {
    SerialMon.println(" OK");

    // lcd.setCursor(0, 1);
    // lcd.print("APN: ");
    // lcd.print("OK");

    // lcd.clear();

    // lcd.setCursor(0, 0);
    // lcd.print("Connecting...");

    // lcd.setCursor(0, 1);
    // lcd.print("Host: ");

    SerialMon.print("Connecting to ");
    SerialMon.print(server);

    if (!client.connect(server, port)) {
      SerialMon.println(" fail");

      // lcd.setCursor(0, 1);
      // lcd.print("Host: ");
      // lcd.print("fail");
    } else {
      SerialMon.println(" OK");
      SerialMon.println("Performing HTTP POST request...");

      // lcd.setCursor(0, 1);
      // lcd.print("Host: ");
      // lcd.print(" OK");

      String field1 = "&field1=";
      field1 += str1;
      String field2 = "&field2=";
      field2 += str2;
      String field3 = "&field3=";
      if (thresholdReached) {
        field3 += "1";
      } else {
        field3 += "0";
      }

      SerialMon.print(field1);
      SerialMon.print(field2);

      // Prepare your HTTP POST request data
      String httpRequestData =
        "api_key=" + apiKeyValue + field1 + field2 + field3;

      client.print(String("POST ") + resource + " HTTP/1.1\r\n");
      client.print(String("Host: ") + server + "\r\n");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(httpRequestData.length());
      client.println();
      client.println(httpRequestData);

      unsigned long timeout = millis();
      while (client.connected() && millis() - timeout < 10000L) {
        // Print available data (HTTP response from server)
        while (client.available()) {
          char c = client.read();
          SerialMon.print(c);
          timeout = millis();
        }
      }
      SerialMon.println();

      // Close client and disconnect
      client.stop();
      SerialMon.println(F("Server disconnected"));
      modem.gprsDisconnect();
      SerialMon.println(F("GPRS disconnected"));
    }
  }
}

void loop() {

  int MQValue = 0;

  // adjust calibration factor
  scale.set_scale(calibration_factor);

  // run strain function
  float strainValue = displayStrain();

  // run strain threshold alert function
  strainThresholdAlert();

  // if there is no data in the serial buffer, run the following functions
  if (Serial.available() <= 0) {
    MQValue = displayMQValue();
    alertMQSensor();
  }

  // function to update Strain and MQ sensor values to ThingSpeak
  sendDatatoCloud.run(sendDataToThingSpeak, String(strainValue), String(MQValue));

  delay(500);
}

// This function will alert the user if the MQ sensor detects a gas leak
void alertMQSensor() {
  // check to see if the MQ sensor value is greater than or equal to the threshold
  if ((analogRead(MQ) / 4) >= MQThreshold) {

    thresholdReached = true;

    // if the MQ sensor value is greater than or equal to the threshold, send an alert
    String smsMessage = "MQ Sensor Gas Alert: Sensor detected Smoke or Gas particles.";

    // send the alert using the TinyGSM library
    modem.sendSMS(TARGET_NUMBER, smsMessage);

    // make phone call to
    modem.callNumber(TARGET_NUMBER);

  }
}
// This function will display the MQ sensor value on the LCD display
int displayMQValue() {
  // set cursor to the first row and first column
  lcd.setCursor(0, 1);
  lcd.print("MQ Value: ");

  // MQ sensor value is stored in the variable MQValueInByte
  MQValueInByte = Serial.read();
  initialMQValue = analogRead(MQ) / 4;

  // if the MQ sensor value is greater than or equal to the threshold, display the value
  lcd.print(initialMQValue);

  return initialMQValue;
}
// This function will alert the user if the strain threshold is exceeded
void strainThresholdAlert() {

  // get data from scale
  float strain = scale.get_units();

  // check to see if strain is greater than or equal to the threshold
  float alert = strain <= strainThreshold && strain >= 0.5; //if greater than 0.5kg 

  // if strain is greater than or equal to the threshold, send an alert
  if (alert) {
    String smsMessage = "Strains Alert: Weight is below "+String(strainThreshold)+"kg";
    modem.sendSMS(TARGET_NUMBER, smsMessage);
  }
}


// This function will display the strain value on the LCD display
float displayStrain() {
  // get data from scale
  float strain = scale.get_units();

  // set lcd display to print strain
  lcd.setCursor(0, 0);
  lcd.print("Strain: ");
  lcd.print(strain <= 0.00 ? 0 : strain);
  lcd.print(" kg ");

  return strain;
}
