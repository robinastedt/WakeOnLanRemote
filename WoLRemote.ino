
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <avr/interrupt.h>

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

enum PROGRAM_STATE {
  INITIALIZING,
  CONNECTED,
  DISCONNECTED,
  SENDING,
  ERROR0
} programState;


const unsigned int portListen = 5612;
const unsigned int portBroadcast = 5611;
const IPAddress ipBroadcast(10, 0, 0, 255);
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


  // initialize timer
  initializeTimer4();
  
  parseTargetMAC();
  printTargetMAC();

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

void initializeTimer4() {
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;           // Set the counter to 8-bit mode
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  REG_TC4_COUNT8_CC0 = 0x55;                      // Set the TC4 CC0 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_COUNT8_CC1 = 0xAA;                      // Set the TC4 CC1 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_COUNT8_PER = 0xFF;                      // Set the PER (period) register to its maximum value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  //NVIC_DisableIRQ(TC4_IRQn);
  //NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_MC1 | TC_INTENSET_MC0 | TC_INTENSET_OVF;     // Enable TC4 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_MC1 | TC_INTENCLR_MC0 | TC_INTENCLR_OVF;     // Disable TC4 interrupts
 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV64 |     // Set prescaler to 64, 16MHz/64 = 256kHz
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY);        // Wait for synchronization
}

void TC4_Handler()                              // Interrupt Service Routine (ISR) for timer TC4
{     
  // Check for overflow (OVF) interrupt
  if (TC4->COUNT8.INTFLAG.bit.OVF && TC4->COUNT8.INTENSET.bit.OVF)             
  {
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
  }

  // Check for match counter 0 (MC0) interrupt
  if (TC4->COUNT8.INTFLAG.bit.MC0 && TC4->COUNT8.INTENSET.bit.MC0)             
  {
    REG_TC4_INTFLAG = TC_INTFLAG_MC0;         // Clear the MC0 interrupt flag
  }

  // Check for match counter 1 (MC1) interrupt
  if (TC4->COUNT8.INTFLAG.bit.MC1 && TC4->COUNT8.INTENSET.bit.MC1)           
  {
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
   
    REG_TC4_INTFLAG = TC_INTFLAG_MC1;        // Clear the MC1 interrupt flag
  }
}

int getStatusLEDInterval() {
  switch (programState) {
    case INITIALIZING: return 500;
    case CONNECTED:    return 0;
    case DISCONNECTED: return 2000;
    case SENDING:      return 100;
    case ERROR0:       return 200;
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
  udp.beginPacket(ipBroadcast, portBroadcast);
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
  printMacAddress();
}


void printIP() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
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
