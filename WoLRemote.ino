
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_ZeroTimer.h>

#include "arduino_secrets.h"

const bool DEBUG = false;
const unsigned int baudRate = 19200;

const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

byte targetMAC1[6];
byte targetMAC2[6];

int status = WL_IDLE_STATUS;
const int statusPollingInterval = 10000;
int statusPollingCounter = 0;

const byte button1Pin = 4;
const byte button2Pin = 0;
const byte statusLEDPin = 6;
volatile byte button1State = LOW;
volatile byte button2State = LOW;
volatile byte statusLEDState = LOW;
int statusLEDCounter = 0;
const int buttonPollingInterval = 100;

Adafruit_ZeroTimer zt4 = Adafruit_ZeroTimer(4);

enum PROGRAM_STATE {
  INITIALIZING,
  CONNECTED,
  DISCONNECTED,
  SENDING,
  ERROR0
} programState;


const unsigned int portListen = 5612;
const unsigned int portBroadcast = 5611;
WiFiUDP udp;

void setup() {
  programState = INITIALIZING;
  //Initialize Serial and wait for port to open:
  if (DEBUG) {
    Serial.begin(9600);
    while (!Serial);
  }
  
  pinMode(statusLEDPin, OUTPUT);
  digitalWrite(statusLEDPin, LOW);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  initializeTimer();
  
  parseTargetMAC();
  if (DEBUG) printTargetMAC();

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    if (DEBUG) Serial.println("WiFi shield not present");
    programState = ERROR0;
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    if (DEBUG) {
      Serial.print("Attempting to connect to WPA SSID: ");
      Serial.println(ssid);
    }
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  if (DEBUG) {
    Serial.println("You're connected to the network");

    Serial.println("Opening UDP socket...");
  }
  
  udp.begin(portListen);
  if (DEBUG) {
    Serial.print("Listening for UDP at port ");
    Serial.println(portListen, DEC);
  }
  
  attachInterrupt(digitalPinToInterrupt(button1Pin), button1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(button2Pin), button2Interrupt, RISING);

  programState = CONNECTED;

  if (DEBUG) {
    Serial.println();
    printWiFiInfo();
  }
}

void initializeTimer() {
  const static int period_us = 10000; // 100 Hz
  const static int basePeriodCounter = F_CPU / 1000000 * period_us;
  int periodCounter = basePeriodCounter;
  int divisions = 0;
  while (periodCounter > 0xFFFF) {
    periodCounter >>= 1;
    divisions++;
  }
  int pulseCounter = periodCounter / 2;
  enum tc_clock_prescaler prescaler = (enum tc_clock_prescaler)TC_CTRLA_PRESCALER(divisions);

  zt4.configure(prescaler, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_PWM);
  zt4.setPeriodMatch(periodCounter, pulseCounter, 1);
  zt4.setCallback(true, TC_CALLBACK_CC_CHANNEL1, Timer4Callback1);
  zt4.enable(true);
}

void TC4_Handler() {
  Adafruit_ZeroTimer::timerHandler(4);
}

void Timer4Callback1() {
  statusLEDCounter++;

  const int interval = getStatusLEDInterval();
  if (interval == 0) {
    statusLEDCounter = 0;
    statusLEDState = HIGH;
    digitalWrite(statusLEDPin, statusLEDState);
  }
  else if (statusLEDCounter >= interval) {
    statusLEDCounter = 0;
    statusLEDState ^= 1;
    digitalWrite(statusLEDPin, statusLEDState);
  }
}


// Interval is in 100th of seconds
int getStatusLEDInterval() {
  switch (programState) {
    case INITIALIZING: return 10;
    case CONNECTED:    return 0;
    case DISCONNECTED: return 200;
    case SENDING:      return 25;
    case ERROR0:       return 50;
  }
}

void loop() {
  if (button1State == HIGH) {
    if (DEBUG) Serial.println();
    sendMagicPacket(1);
  }
  if (button2State == HIGH) {
    if (DEBUG) Serial.println();
    sendMagicPacket(2);
  }

  if (button1State == HIGH || button2State == HIGH) {
    delay(1000);
    checkWiFiConnection();
    button1State = LOW;
    button2State = LOW;
  }
    
  delay(buttonPollingInterval);
  statusPollingCounter += buttonPollingInterval;
  if (statusPollingCounter >= statusPollingInterval) {
    statusPollingCounter = 0;
    checkWiFiConnection();
  }
}

void checkWiFiConnection() {
  status = WiFi.status();
  if (status == WL_CONNECTED) {
    programState = CONNECTED;
  }
  else {
    programState = DISCONNECTED;
    status = WiFi.begin(ssid, pass);
    if (status == WL_CONNECTED) {
      udp.begin(portListen);
      programState = CONNECTED;
    }
  }
  if (DEBUG) {
    Serial.println();
    printWiFiInfo();
  }
}

void button1Interrupt() {
  button1State = HIGH;
}

void button2Interrupt() {
  button2State = HIGH;
}


void sendMagicPacket(int target) {
  programState = SENDING;
  const byte header[6] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  char strBuf[6*3];
  
  if (DEBUG) {
    Serial.print("Sending magic packet to target ");
    Serial.print(target, DEC);
    Serial.println("...");
  }
  udp.beginPacket(getSubnetBroadcastAddress(), portBroadcast);
  if (DEBUG) {
    Serial.println("Raw data:");

    bytesToStr(strBuf, header, 6);
    Serial.print(strBuf);
  }
  udp.write(header, 6);
  
  if (DEBUG) {
    if (target == 1) {
      bytesToStr(strBuf, targetMAC1, 6);
    }
    else {
      bytesToStr(strBuf, targetMAC2, 6);
    }
  }
  for (int i = 0; i < 16; i++) {
    if (DEBUG) {
      Serial.print(":");
      Serial.print(strBuf);
    }
    if (target == 1) {
      udp.write(targetMAC1, 6);
    }
    else {
      udp.write(targetMAC2, 6);
    }
    
  }
  if (DEBUG) Serial.println();

  udp.endPacket();
  if (DEBUG) Serial.println("Magic packet sent!");
}

IPAddress getSubnetBroadcastAddress() {
  IPAddress mask = WiFi.subnetMask();
  IPAddress ip = WiFi.localIP();
  IPAddress broadcast;
  for (int i = 0; i < 4; i++) {
    broadcast[i] = (mask[i] & ip[i]) | (~(mask[i]));
  }
  return broadcast;
}


void printTargetMAC() {
  char targetMACstr[6*3];
  bytesToStr(targetMACstr, targetMAC1, 6);
  Serial.print("Target 1 MAC: ");
  Serial.println(targetMACstr);
  bytesToStr(targetMACstr, targetMAC2, 6);
  Serial.print("Target 2 MAC: ");
  Serial.println(targetMACstr);
}

void parseTargetMAC() {
  {
    const char macStr[] = TARGET_MAC_1;
    for (int i = 0; i < 6; i++) {
      targetMAC1[i] = strToByte(macStr + i * 3);
    }
  }
  {
    const char macStr[] = TARGET_MAC_2;
    for (int i = 0; i < 6; i++) {
      targetMAC2[i] = strToByte(macStr + i * 3);
    }
  }
}

byte strToByte(const char* str) {
  return 0x10 * charToByte(str[0]) + charToByte(str[1]);
}

byte charToByte(char c) {
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 0xA;
  }
  else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 0xA;
  }
  else if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else {
    return 0;
  }
}

void printWiFiInfo() {
  printSSID();
  printBSSID();
  printSignalStrength();
  printEncryptionType();
  printIP();
  printSubnetMask();
  printBroadcastAddress();
  printMacAddress();
}


void printIP() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void printSubnetMask() {
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("Subnet mask: ");
  Serial.println(subnet);
}

void printBroadcastAddress() {
  IPAddress broadcast = getSubnetBroadcastAddress();
  Serial.print("Broadcast address: ");
  Serial.println(broadcast);
}

void printSSID() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
}

void printBSSID() {
  byte bssid[6];
  char bssidStr[6*3];
  WiFi.BSSID(bssid);
  reverseArray(bssid, 6);
  bytesToStr(bssidStr, bssid, 6);
  Serial.print("BSSID: ");
  Serial.println(bssidStr);
}

void printMacAddress() {
  byte mac[6];
  char macStr[6*3];
  WiFi.macAddress(mac);
  reverseArray(mac, 6);
  bytesToStr(macStr, mac, 6);
  Serial.print("MAC address: ");
  Serial.println(macStr);
}

void reverseArray(byte* arr, int count) {
  for (int i = 0; i < count / 2; i++) {
    byte tmp = arr[i];
    arr[i] = arr[count - 1 - i];
    arr[count - 1 - i] = tmp;
  }
}

void bytesToStr(char* out, const byte bytes[], int count) {
  for (int i = 0; i < count; i++) {
    if (i > 0) {
      *out++ = ':';
    }
    const byte current = bytes[i];
    *out++ = lowerByteToHex(current >> 4);
    *out++ = lowerByteToHex(current & 0x0F);
    
  }
  *out++ = '\0';
}

char lowerByteToHex(byte b) {
  if (b < 0xA) {
    return (char)b + '0';
  }
  return (char)b - 0xA + 'A';
}

void printSignalStrength() {
  const long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.print("dBm");
  Serial.println();
}

void printEncryptionType() {
  const byte encryption = WiFi.encryptionType();
  Serial.print("Encryption type: ");
  Serial.print(getEncryptionTypeString(encryption));
  Serial.println();
}

const char* getEncryptionTypeString(byte encryptionType) {
  static const char TKIP[]  = "TKIP (WPA)";
  static const char WEP[]   = "WEP";
  static const char CCMP[]  = "CCMP (WPA)";
  static const char NONE[]  = "NONE";
  static const char AUTO[]  = "AUTO";
  static const char UNKWN[] = "UNKNOWN";

  static const byte type_TKIP = 0x2;
  static const byte type_WEP  = 0x5;
  static const byte type_CCMP = 0x4;
  static const byte type_NONE = 0x7;
  static const byte type_AUTO = 0x8;

  switch(encryptionType) {
    case type_TKIP: return TKIP;
    case type_WEP:  return WEP;
    case type_CCMP: return CCMP;
    case type_NONE: return NONE;
    case type_AUTO: return AUTO;
    default:        return UNKWN;
  }
}
