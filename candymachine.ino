/*                               _____                   _                                 _      _              
 *   ___    .-"""-.    ___      / ____|                 | |                               | |    (_)             
 *   \  \  /\ \ \ \\  /  /     | |      __ _  _ __    __| | _   _  _ __ ___    __ _   ___ | |__   _  _ __    ___ 
 *    }  \/\ \ \ \ \\/  {      | |     / _` || '_ \  / _` || | | || '_ ` _ \  / _` | / __|| '_ \ | || '_ \  / _ \
 *    }  /\ \ \ \ \ /\  {      | |____| (_| || | | || (_| || |_| || | | | | || (_| || (__ | | | || || | | ||  __/
 *   /__/  \ \ \ \ /  \__\      \_____|\__,_||_| |_| \__,_| \__, ||_| |_| |_| \__,_| \___||_| |_||_||_| |_| \___|
 *          '-...-'                                          __/ |                   
 *                                                          |___/                              Renze Nicolai 2016
 */

#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include "sha256.h"

/*** Configuration ***/
#define FW_VERSION 6
#define EE_AUTHSTR_LEN 50
#define TIMEOUT_TIME 99
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };

/*** Pins ***/
#define ONEWIRE_PIN A0
#define ONEWIRE_LED A1
#define KEYPAD_R1 49
#define KEYPAD_R2 48
#define KEYPAD_R3 47
#define KEYPAD_R4 46
#define KEYPAD_C1 43
#define KEYPAD_C2 44
#define KEYPAD_C3 45

/* OLD */
#define KPOUT_R1 29
#define KPOUT_R2 24
#define KPOUT_R3 25
#define KPOUT_R4 22
#define KPOUT_C1 23
#define KPOUT_C2 27
#define KPOUT_C3 26
/* --- */

#define KPOUT_DAT A2
#define KPOUT_CLK A3
#define KPOUT_LAT A4

/*** Variables ***/
IPAddress ip(0,0,0,0);
IPAddress server(0,0,0,0);
uint32_t port = 0;
String username = "";
String password = "";
String devicename = "";
String mqtttopic = "";
String user_name = "";
String user_saldo = "";
String lcd_message = "";
String lcd_welcome_message = "";
String sha256string = ""; //Used as ID
uint16_t timeout = 0;

uint8_t product = 0; //When logged in
uint16_t code = 0; //For use without iButton

bool isVending = false;
bool isCodeCheck  = false;

struct EESettings {
  uint32_t magic;
  uint8_t server[4];
  uint32_t port;
  byte mac[6];
  char username[EE_AUTHSTR_LEN];
  char password[EE_AUTHSTR_LEN];
  char topic[EE_AUTHSTR_LEN];
  char devicename[EE_AUTHSTR_LEN];
  char welcome[EE_AUTHSTR_LEN];
  uint32_t chksum;
};

/*** Objects ***/
EthernetClient ethClient;
PubSubClient client(ethClient);
OneWire ds(ONEWIRE_PIN);
LiquidCrystal lcd(A10, A11, A12, A13, A14, A15);

/*** Function prototypes ***/
bool loadConfig(void);
void saveConfig(void);
void deviceSetup(void);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void updateLcd(void);
void readKeys(void);
void logout(void);
bool loggedin(void);
void lcdUserHome(void);
void lcdWelcome(void);
void lcdCodeCheck(void);
void lcdVending(void);
void lcdError(void);
void doActualVend(uint8_t to_vend);
void pressKeyOnController(uint8_t key);

/*** Main program ***/
void setup() {
  Serial.begin(115200); //Start serial port for configuration and debugging
  pinMode(ONEWIRE_LED, OUTPUT);
  pinMode(KEYPAD_R1, INPUT);
  pinMode(KEYPAD_R2, INPUT);
  pinMode(KEYPAD_R3, INPUT);
  pinMode(KEYPAD_R4, INPUT);
  pinMode(KEYPAD_C1, OUTPUT);
  pinMode(KEYPAD_C2, OUTPUT);
  pinMode(KEYPAD_C3, OUTPUT);
  pinMode(KPOUT_DAT, OUTPUT);
  pinMode(KPOUT_CLK, OUTPUT);
  pinMode(KPOUT_LAT, OUTPUT);

  /*while(1) {
    shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0b01010101);
  }*/
  
  /*while (1) {
    for (uint8_t key = 0; key<9; key++) {
      pressKeyOnController(key);
      delay(500);
    }
  }*/
  
  devicename = "unknown";
  mqtttopic = "test";
  lcd_welcome_message = "ERROR";

  //---
  //devicename = "candymachine_"+String(mac[3], HEX)+String(mac[4], HEX)+String(mac[5], HEX);
  //mqtttopic = "test";
  //lcd_welcome_message = "Welkom!";
  timeout = 0;
  //---
  
  delay(200); //Give the powersupply time to stabilize  
  lcd.begin(16, 2);

  Serial.println();
  Serial.println();
  Serial.println("  ___    .-\"\"\"-.    ___    ");
  Serial.println("  \\  \\  /\\ \\ \\ \\\\  /  /    ");
  Serial.println("   }  \\/\\ \\ \\ \\ \\\\/  {     ");
  Serial.println("   }  /\\ \\ \\ \\ \\ /\\  {     ");
  Serial.println("  /__/  \\ \\ \\ \\ /  \\__\\    ");
  Serial.println("         '-...-'           ");
  Serial.println();
  Serial.println("CandyMachine");
  lcd.clear();
  lcd.print("CandyMachine  v"+String(FW_VERSION));
  Serial.println("Firmware version "+String(FW_VERSION)+", built on "+__DATE__+" "+__TIME__+".");
  lcd.setCursor(0,1);
  lcd.print("R. Nicolai  2016");
  delay(2000);
  
  bool configured = loadConfig();

  if (!configured) {
    Serial.println("This device has not been configured. Starting setup!");
    deviceSetup();
  } else {
    while (Serial.available()) {delay(10); Serial.read();}
    Serial.println("Send 's' to enter setup");
    for (uint8_t i = 0; i<10; i++) {
      delay(100);
      Serial.print(".");
      if (Serial.available()) {
        char in = Serial.read();
        if ((in=='s')||(in=='S')) {
          Serial.println("Entering setup!");
          lcd.clear();
          lcd.print("<SETUP>");
          delay(1000);
          deviceSetup();
          break;
        }
      }
    }
    Serial.println();
  }

  Serial.println("MAC address: "+String(mac[0], HEX)+":"+String(mac[1], HEX)+":"+String(mac[2], HEX)+":"+String(mac[3], HEX)+":"+String(mac[4], HEX)+":"+String(mac[5], HEX));
  Serial.println("MQTT server: "+username+":"+password+"@"+String(server[0])+"."+String(server[1])+"."+String(server[2])+"."+String(server[3])+":"+port);
  Serial.println("MQTT topic: "+mqtttopic);
  Serial.println("Device name: "+devicename);
  Serial.println("Ready message: "+lcd_welcome_message);
  Serial.println("\n");

  bool ethernetconnected = false;
  while (!ethernetconnected) {
    Serial.print("Initializing ethernet...");
    lcd.clear();
    lcd.print("Connecting...");
    if (!Ethernet.begin(mac)) {
      Serial.println(" Failed.");
      lcd.clear();
      lcd.print("Network error.");
    } else {
      ip = Ethernet.localIP();
      Serial.println("Connected.");
      delay(1000); //To give the ethernet chip time to stabilize...
      Serial.println("IP address: "+String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]));
      lcd.clear();
      lcd.print("IP address:");
      lcd.setCursor(0,1);
      lcd.print(String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]));
      ethernetconnected = true;
      delay(1000);
    }
  }

  Serial.println("MQTT server: "+String(server[0])+"."+String(server[1])+"."+String(server[2])+"."+String(server[3])+":"+port);
  //Serial.println("Username: "+username);
  //Serial.println("Password: "+password);

  client.setServer(server, port); //Set server
  client.setCallback(mqttCallback); //Configure callback

  //Make sure spirali is ready
  lcd.clear();
  lcd.print("Spirali init...");
  pressKeyOnController(10); //A / back
  pressKeyOnController(10); //A / back
  pressKeyOnController(10); //A / back
  pressKeyOnController(10); //A / back
  pressKeyOnController(10); //A / back
  pressKeyOnController(10); //A / back
  delay(500);

  lcdWelcome();
}

unsigned long heartbeat_time = 0;
uint8_t heartbeat = 0;
bool newdevice = true;

void loop() {
  if (heartbeat_time<millis()) {
    heartbeat_time=millis()+100;
    heartbeat++;
    if (timeout>0) {
      timeout--;
      if (loggedin()) lcdUserHome();
      if (timeout==0) {
        logout();
        Serial.println("Timeout.");
        if (isVending||isCodeCheck) { //Transaction timeout
          isVending = false;
          isCodeCheck = false;
          lcdError();
          Serial.println("Vending error.");
        } else {
          lcdWelcome();
        }
      }
    } else {
      lcdWelcome();
    }
  }
  
  //MQTT connection
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  //OneWire
  bool present = ds.reset();
  ds.skip();
  if (present) { //Device present on the bus
    uint8_t addr_in[8] = {0};
    uint8_t addr[8] = {0};
    if (ds.search(addr_in)) {
      if (newdevice) {
        digitalWrite(ONEWIRE_LED, true);
        if (ds.crc8( addr_in, 7) == addr_in[7]) {
          for (uint8_t i = 0; i<8; i++) {
            addr[i] = addr_in[7-i];
          }
          String idstring = "";
          for (uint8_t i = 0; i<7; i++) { idstring = idstring+String(addr[i]>>4,HEX)+String(addr[i]&0x0F,HEX); }
          timeout = TIMEOUT_TIME;
          Serial.println("Device detected: "+idstring);
          Sha256.init();
          Sha256.print("42SUPERSECRETSALT1337");
          //Sha256.print(idstring);
          for (uint8_t i = 0; i<7; i++) { Sha256.write(addr[i]); }
          sha256string = "";
          uint8_t* hash = Sha256.result();
          for (uint8_t i = 0; i<32; i++) {
            sha256string = sha256string+("0123456789abcdef"[hash[i]>>4])+("0123456789abcdef"[hash[i]&0xf]);
          }
          Serial.println(sha256string);
          String message = devicename+",rq_saldo,"+sha256string;
          user_name = "";
          user_saldo = "";
          lcd.clear();
          lcd.print("Getting saldo...");
          client.publish(mqtttopic.c_str(),message.c_str());
        } else {
          Serial.println("Invalid CRC.");
        }
      }
      newdevice = false;
    } else {
      ds.reset_search();
    }
  } else {
    newdevice = true;
    bool heartbeat_led = 0;
    if (!loggedin()) {
      heartbeat_led = (heartbeat&0b00001000)>>3;
    } else {
      heartbeat_led = true;
    }
    digitalWrite(ONEWIRE_LED, heartbeat_led);
  }

  //User input
  readKeys();
}

/*** Functions ***/

bool loadConfig(void) {
  EESettings settings;
  EEPROM.get(0,settings);
  if (settings.magic!=0x42424242) {
    Serial.println("Magic mismatch.");
    return false;
  }
  uint32_t chksum = 0;
  for (uint8_t i = 0; i<4; i++) {
    server[i] = settings.server[i];
    chksum += settings.server[i];
  }
  port = settings.port;
  chksum += port;
  for (uint8_t i = 0; i<6; i++) {
    mac[i] = settings.mac[i];
    chksum += settings.mac[i];
  }
  username = String(settings.username);
  password = String(settings.password);
  mqtttopic = String(settings.topic);
  devicename = String(settings.devicename);
  lcd_welcome_message = String(settings.welcome);
  if (chksum!=settings.chksum) {
    Serial.println("Invalid checksum. ("+String(chksum)+")");
    return false;
  }
  Serial.println("Configuration loaded from EEPROM.");
  return true;
}

void saveConfig(void) {
  EESettings settings;
  settings.magic=0x42424242;
  settings.chksum=0;
  for (uint8_t i = 0; i<4; i++) {
    settings.server[i] = server[i];
    settings.chksum += server[i];
  }
  settings.port = port;
  settings.chksum += port;
  for (uint8_t i = 0; i<6; i++) {
    settings.mac[i] = mac[i];
    settings.chksum += mac[i];
  }
  username.toCharArray(settings.username, EE_AUTHSTR_LEN);
  password.toCharArray(settings.password, EE_AUTHSTR_LEN);
  mqtttopic.toCharArray(settings.topic, EE_AUTHSTR_LEN);
  devicename.toCharArray(settings.devicename, EE_AUTHSTR_LEN);
  lcd_welcome_message.toCharArray(settings.welcome, EE_AUTHSTR_LEN);
  EEPROM.put(0,settings);
  Serial.println("Configuration saved to EEPROM.");
}

void deviceSetup(void) {
  Serial.setTimeout(100000);
  bool ok = false;
  while (!ok) {
    Serial.println();
    Serial.println();
    Serial.println("##### SETUP ####");
    while (Serial.available()) {delay(10); Serial.read();}
    Serial.print("Welcome. ");
    for (uint8_t i = 0; i<6; i++) {
      Serial.println("Please enter byte number "+String(i+1)+" of the MAC address.");
      while (!Serial.available()) delay(100);
      mac[i] = Serial.parseInt();
      Serial.println("Received '"+String(mac[i])+"' (HEX: "+String(mac[i], HEX)+").");
      while (Serial.available()) {delay(10); Serial.read();}
    }
    for (uint8_t i = 0; i<4; i++) {
      Serial.println("Please enter byte number "+String(i+1)+" of the server IP address.");
      while (!Serial.available()) delay(100);
      server[i] = Serial.parseInt();
      Serial.println("Received '"+String(server[i])+"'.");
      while (Serial.available()) {delay(10); Serial.read();}
    }
    Serial.println("Please enter the port number for the server.");
    while (!Serial.available()) delay(100);
    port = Serial.parseInt();
    Serial.println("Received '"+String(port)+"'.");
    while (Serial.available()) {delay(10); Serial.read();}
    Serial.println("Please enter username for the server. (end with newline)");
    while (Serial.available()) {delay(10); Serial.read();}
    username = "";
    while (true) {
      while (!Serial.available());
      char in = Serial.read();
      if ((in == '\n')||(in == '\r')) {
        break;
      }
      username = username+in;
      if (username.length()>EE_AUTHSTR_LEN-3) { Serial.println("\nLimit reached."); break;}
    }
    Serial.println("Username is set to '"+String(username)+"'");

    Serial.println("Please enter password for the server. (end with newline)");
    while (Serial.available()) {delay(10); Serial.read();}
    password = "";
    while (true) {
      while (!Serial.available());
      char in = Serial.read();
      if ((in == '\n')||(in == '\r')) {
        break;
      }
      password = password+in;
      if (password.length()>EE_AUTHSTR_LEN-3) { Serial.println("\nLimit reached."); break;}
    }
    Serial.println("Password is set to '"+String(password)+"'");

    Serial.println("Please enter MQTT topic. (end with newline)");
    while (Serial.available()) {delay(10); Serial.read();}
    mqtttopic = "";
    while (true) {
      while (!Serial.available());
      char in = Serial.read();
      if ((in == '\n')||(in == '\r')) {
        break;
      }
      mqtttopic = mqtttopic+in;
      if (mqtttopic.length()>EE_AUTHSTR_LEN-3) { Serial.println("\nLimit reached."); break;}
    }
    Serial.println("MQTT topic is set to '"+String(mqtttopic)+"'");

    Serial.println("Please enter device name. (end with newline)");
    while (Serial.available()) {delay(10); Serial.read();}
    devicename = "";
    while (true) {
      while (!Serial.available());
      char in = Serial.read();
      if ((in == '\n')||(in == '\r')) {
        break;
      }
      devicename = devicename+in;
      if (devicename.length()>EE_AUTHSTR_LEN-3) { Serial.println("\nLimit reached."); break;}
    }
    Serial.println("Device name is set to '"+String(devicename)+"'");

    Serial.println("Please enter default ready message. (end with newline)");
    while (Serial.available()) {delay(10); Serial.read();}
    lcd_welcome_message = "";
    while (true) {
      while (!Serial.available());
      char in = Serial.read();
      if ((in == '\n')||(in == '\r')) {
        break;
      }
      lcd_welcome_message = lcd_welcome_message+in;
      if (lcd_welcome_message.length()>EE_AUTHSTR_LEN-3) { Serial.println("\nLimit reached."); break;}
    }
    Serial.println("Default ready message is set to '"+String(lcd_welcome_message)+"'");

    Serial.println("MAC address: "+String(mac[0], HEX)+":"+String(mac[1], HEX)+":"+String(mac[2], HEX)+":"+String(mac[3], HEX)+":"+String(mac[4], HEX)+":"+String(mac[5], HEX));
    Serial.println("MQTT server: "+username+":"+password+"@"+String(server[0])+"."+String(server[1])+"."+String(server[2])+"."+String(server[3])+":"+port);
    Serial.println("MQTT topic: "+mqtttopic);
    Serial.println("Device name: "+devicename);
    Serial.println("Ready message: "+lcd_welcome_message);
    Serial.println("\n");
    bool answered = false;
    bool answer = false;
    while (!answered) {
      Serial.println("Is this correct? (Y/N)");
      while (!Serial.available()) delay(1);
      char ans = Serial.read();
      if ((ans=='y')||(ans=='Y')) { answer = true; answered = true; }
      if ((ans=='n')||(ans=='N')) { answer = false; answered = true; }
      if (!answered) Serial.print("? ");
      while (Serial.available()) {delay(10); Serial.read();}
    }
    if (answer) ok = true;
    Serial.println();
    Serial.println();
  }
  saveConfig();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String m = "";
  for (int i=0;i<length;i++) {
    m = m + (char)payload[i];
  }
  String t = topic;

  if (t.equals(mqtttopic)) {
    String mq_name = "";
    uint16_t pos = 0;
    while (pos<m.length()) {
      if (m.charAt(pos)==',') {pos++; break;}
      mq_name = mq_name+m.charAt(pos);
      pos++;
    }
    String mq_cmd = "";
    while (pos<m.length()) {
      if (m.charAt(pos)==',') {pos++; break;}
      mq_cmd = mq_cmd+m.charAt(pos);
      pos++;
    }
    String mq_param = ""; 
    while (pos<m.length()) {
      //if (m.charAt(pos)==',') {pos++; break;}
      mq_param = mq_param+m.charAt(pos);
      pos++;
    }
    Serial.println("MQ % "+mq_name+" % "+mq_cmd + " % "+mq_param);
    if (mq_name.equals(devicename)) {
      if (mq_cmd.equals("rp_saldo")) {
        user_name = "";
        user_saldo = "";
        pos = 0;
        //Serial.println("RP_SALDO: "+mq_param);
        while (pos<mq_param.length()) {
          if (mq_param.charAt(pos)==',') {pos++; break;}
          user_saldo = user_saldo + mq_param.charAt(pos);
          //Serial.println("S"+mq_param.charAt(pos));
          pos++;
        }
        while (pos<mq_param.length()) {
          if (mq_param.charAt(pos)==',') {pos++; break;}
          user_name = user_name + mq_param.charAt(pos);
          //Serial.println("N"+mq_param.charAt(pos));
          pos++;
        }
        isCodeCheck = 0;
        isVending = 0;
        Serial.println("Logged on as "+user_name+". Saldo: "+user_saldo);
        lcdUserHome();
        timeout = TIMEOUT_TIME;
      } else if (mq_cmd.equals("rp_setmessage")) {
        lcd_welcome_message = mq_param;
        Serial.println("Message set to: "+lcd_welcome_message);
        if ((!loggedin())&&(!isVending)&&(!isCodeCheck)) lcdWelcome();
      } else if (mq_cmd.equals("rp_vend")) {
        pos = 0;
        String to_vend_str = "";
        while (pos<mq_param.length()) {
          if (mq_param.charAt(pos)==',') {pos++; break;}
          to_vend_str = to_vend_str + mq_param.charAt(pos);
          pos++;
        }
        uint8_t to_vend = to_vend_str.toInt();
        Serial.println("Got vend command for product "+String(to_vend));
        if (isVending||isCodeCheck) {
          doActualVend(to_vend);
          logout();
          isVending = 0;
          isCodeCheck = 0;
        } else {
          lcd.clear();
          lcd.print("DEVICE LOCKED");
          lcd.setCursor(0,1);
          lcd.print("(Don't hack me!)");
          delay(10000);
          logout();
          isVending = 0;
          isCodeCheck = 0;
        }
      } else if (mq_cmd.equals("rp_error")) {
        pos = 0;
        String error_str = "";
        while (pos<mq_param.length()) {
          if (mq_param.charAt(pos)==',') {pos++; break;}
          error_str = error_str + mq_param.charAt(pos);
          pos++;
        }
        lcd.clear();
        lcd.print(error_str);
        delay(1000);
        logout();
        isVending = 0;
        isCodeCheck = 0;
        timeout = 1;
      } else if (mq_cmd.equals("rp_passthrough")) {
        logout();
        passthrough();
        isVending = 0;
        isCodeCheck = 0;
      }
    }
  } else {
    Serial.println("[MQTT] "+t+": "+m);
  }
}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    lcd.clear();
    lcd.print("MQTT...");
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(devicename.c_str(), username.c_str(), password.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      String message = devicename+",connect";
      client.publish(mqtttopic.c_str(), message.c_str());
      // ... and resubscribe
      client.subscribe(mqtttopic.c_str());
    } else {
      lcd.clear();
      lcd.print("Network error.");
      lcd.setCursor(0,1);
      lcd.print("rc="+String(client.state()));
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

uint8_t column1d = 0;
uint8_t column2d = 0;
uint8_t column3d = 0;

void keypadCancel() { 
  if (product>0) {
    product = 0;
    if (loggedin()) lcdUserHome();
  } else {
    logout();
    timeout = 5;
    //lcdWelcome();
  }
}

void keypadOk() {
  if (!isVending) {
    uint8_t realproduct = (product&0x0F)+((product&0xF0)>>4)*10;
    Serial.println("Vend? Product is "+String(realproduct));
    String message = devicename+",rq_vend,"+sha256string+","+String(realproduct);
    client.publish(mqtttopic.c_str(),message.c_str());
    isVending = true;
    timeout = 10;
    lcdVending();
  }
}

char lastkey = '-';
uint8_t bc = 0;
void passthrough() {
  lcd.clear();
  lcd.print("PASSTHROUGH");
  while (1) {
    pinMode(KEYPAD_C1, OUTPUT);
    digitalWrite(KEYPAD_C1, LOW);
    pinMode(KEYPAD_C2, INPUT);
    digitalWrite(KEYPAD_C2, HIGH);
    pinMode(KEYPAD_C3, INPUT);
    digitalWrite(KEYPAD_C3, HIGH);
    uint8_t column1 = (~PINL)&0x0F;
    pinMode(KEYPAD_C1, INPUT);
    digitalWrite(KEYPAD_C1, HIGH);
    pinMode(KEYPAD_C2, OUTPUT);
    digitalWrite(KEYPAD_C2, LOW);
    pinMode(KEYPAD_C3, INPUT);
    digitalWrite(KEYPAD_C3, HIGH);
    uint8_t column2 = (~PINL)&0x0F;
    pinMode(KEYPAD_C1, INPUT);
    digitalWrite(KEYPAD_C1, HIGH);
    pinMode(KEYPAD_C2, INPUT);
    digitalWrite(KEYPAD_C2, HIGH);
    pinMode(KEYPAD_C3, OUTPUT);
    digitalWrite(KEYPAD_C3, LOW);
    uint8_t column3 = (~PINL)&0x0F;
  
    if (column1&(1<<0)) { if (!(column1d&(1<<0))) { bc = 0; lastkey = '1'; pressKeyOnController(1); column1d |= (1 << 0); } } else { column1d &= ~(1 << 0); };
    if (column2&(1<<0)) { if (!(column2d&(1<<0))) { bc = 0; lastkey = '2'; pressKeyOnController(2); column2d |= (1 << 0); } } else { column2d &= ~(1 << 0); };
    if (column3&(1<<0)) { if (!(column3d&(1<<0))) { bc = 0; lastkey = '3'; pressKeyOnController(3); column3d |= (1 << 0); } } else { column3d &= ~(1 << 0); };
    if (column1&(1<<1)) { if (!(column1d&(1<<1))) { bc = 0; lastkey = '4'; pressKeyOnController(4); column1d |= (1 << 1); } } else { column1d &= ~(1 << 1); };
    if (column2&(1<<1)) { if (!(column2d&(1<<1))) { bc = 0; lastkey = '5'; pressKeyOnController(5); column2d |= (1 << 1); } } else { column2d &= ~(1 << 1); };
    if (column3&(1<<1)) { if (!(column3d&(1<<1))) { bc = 0; lastkey = '6'; pressKeyOnController(6); column3d |= (1 << 1); } } else { column3d &= ~(1 << 1); };
    if (column1&(1<<2)) { if (!(column1d&(1<<2))) { bc = 0; lastkey = '7'; pressKeyOnController(7); column1d |= (1 << 2); } } else { column1d &= ~(1 << 2); };
    if (column2&(1<<2)) { if (!(column2d&(1<<2))) { bc = 0; lastkey = '8'; pressKeyOnController(8); column2d |= (1 << 2); } } else { column2d &= ~(1 << 2); };
    if (column3&(1<<2)) { if (!(column3d&(1<<2))) { bc = 0; lastkey = '9'; pressKeyOnController(9); column3d |= (1 << 2); } } else { column3d &= ~(1 << 2); };
    if (column2&(1<<3)) { if (!(column2d&(1<<3))) { bc = 0; lastkey = '0'; pressKeyOnController(0); column2d |= (1 << 3); } } else { column2d &= ~(1 << 3); };
    if (column1&(1<<3)) { if (!(column1d&(1<<3))) { bc +=1; if (bc>4) break;  lastkey = 'A'; pressKeyOnController(10); column1d |= (1 << 3); } } else { column1d &= ~(1 << 3); };
    if (column3&(1<<3)) { if (!(column3d&(1<<3))) { bc = 0; lastkey = 'B'; pressKeyOnController(11); } } else { column3d &= ~(1 << 3); };
    if (column1||column2||column3) {
      lcd.clear();
      lcd.print("PASSTHROUGH");
      lcd.setCursor(0,1);
      lcd.print("5x back to exit");
      lcd.write(lastkey);
    }
  }
  lcd.clear();
  logout();
  lcdWelcome();
}

void codeCheck() {
  if ((!isCodeCheck)&&(!isVending)) {
    uint16_t realcode = (code&0x000F)+((code&0x00F0)>>4)*10+((code&0x0F00)>>8)*100+((code&0xF000)>>12)*1000;
    sha256string = "guest_"+String(realcode); //Hack :D
    Serial.println("Guest code: "+String(realcode));
    String message = devicename+",rq_code,"+String(realcode);
    client.publish(mqtttopic.c_str(),message.c_str());
    lcdCodeCheck();
    isCodeCheck = true;
  }
}

void readKeys() {
  pinMode(KEYPAD_C1, OUTPUT);
  digitalWrite(KEYPAD_C1, LOW);
  pinMode(KEYPAD_C2, INPUT);
  digitalWrite(KEYPAD_C2, HIGH);
  pinMode(KEYPAD_C3, INPUT);
  digitalWrite(KEYPAD_C3, HIGH);
  uint8_t column1 = (~PINL)&0x0F;
  pinMode(KEYPAD_C1, INPUT);
  digitalWrite(KEYPAD_C1, HIGH);
  pinMode(KEYPAD_C2, OUTPUT);
  digitalWrite(KEYPAD_C2, LOW);
  pinMode(KEYPAD_C3, INPUT);
  digitalWrite(KEYPAD_C3, HIGH);
  uint8_t column2 = (~PINL)&0x0F;
  pinMode(KEYPAD_C1, INPUT);
  digitalWrite(KEYPAD_C1, HIGH);
  pinMode(KEYPAD_C2, INPUT);
  digitalWrite(KEYPAD_C2, HIGH);
  pinMode(KEYPAD_C3, OUTPUT);
  digitalWrite(KEYPAD_C3, LOW);
  uint8_t column3 = (~PINL)&0x0F;

  if (loggedin()) {
    if (column1&(1<<0)) { if (!(column1d&(1<<0))) { product = (product<<4)+1; column1d |= (1 << 0); } } else { column1d &= ~(1 << 0); };
    if (column2&(1<<0)) { if (!(column2d&(1<<0))) { product = (product<<4)+2; column2d |= (1 << 0); } } else { column2d &= ~(1 << 0); };
    if (column3&(1<<0)) { if (!(column3d&(1<<0))) { product = (product<<4)+3; column3d |= (1 << 0); } } else { column3d &= ~(1 << 0); };
    if (column1&(1<<1)) { if (!(column1d&(1<<1))) { product = (product<<4)+4; column1d |= (1 << 1); } } else { column1d &= ~(1 << 1); };
    if (column2&(1<<1)) { if (!(column2d&(1<<1))) { product = (product<<4)+5; column2d |= (1 << 1); } } else { column2d &= ~(1 << 1); };
    if (column3&(1<<1)) { if (!(column3d&(1<<1))) { product = (product<<4)+6; column3d |= (1 << 1); } } else { column3d &= ~(1 << 1); };
    if (column1&(1<<2)) { if (!(column1d&(1<<2))) { product = (product<<4)+7; column1d |= (1 << 2); } } else { column1d &= ~(1 << 2); };
    if (column2&(1<<2)) { if (!(column2d&(1<<2))) { product = (product<<4)+8; column2d |= (1 << 2); } } else { column2d &= ~(1 << 2); };
    if (column3&(1<<2)) { if (!(column3d&(1<<2))) { product = (product<<4)+9; column3d |= (1 << 2); } } else { column3d &= ~(1 << 2); };
    if (column2&(1<<3)) { if (!(column2d&(1<<3))) { product = (product<<4)+0; column2d |= (1 << 3); } } else { column2d &= ~(1 << 3); };
    if (column1&(1<<3)) { if (!(column1d&(1<<3))) { keypadCancel(); column1d |= (1 << 3); } } else { column1d &= ~(1 << 3); };
    if (column3&(1<<3)) { if (!(column3d&(1<<3))) { keypadOk(); column3d |= (1 << 3); } } else { column3d &= ~(1 << 3); };
    if (column1||column2||column3) {
      if (loggedin()) lcdUserHome();
      timeout = TIMEOUT_TIME;
    }
  } else {
    if (column1&(1<<0)) { if (!(column1d&(1<<0))) { code = (code<<4)+1; column1d |= (1 << 0); } } else { column1d &= ~(1 << 0); };
    if (column2&(1<<0)) { if (!(column2d&(1<<0))) { code = (code<<4)+2; column2d |= (1 << 0); } } else { column2d &= ~(1 << 0); };
    if (column3&(1<<0)) { if (!(column3d&(1<<0))) { code = (code<<4)+3; column3d |= (1 << 0); } } else { column3d &= ~(1 << 0); };
    if (column1&(1<<1)) { if (!(column1d&(1<<1))) { code = (code<<4)+4; column1d |= (1 << 1); } } else { column1d &= ~(1 << 1); };
    if (column2&(1<<1)) { if (!(column2d&(1<<1))) { code = (code<<4)+5; column2d |= (1 << 1); } } else { column2d &= ~(1 << 1); };
    if (column3&(1<<1)) { if (!(column3d&(1<<1))) { code = (code<<4)+6; column3d |= (1 << 1); } } else { column3d &= ~(1 << 1); };
    if (column1&(1<<2)) { if (!(column1d&(1<<2))) { code = (code<<4)+7; column1d |= (1 << 2); } } else { column1d &= ~(1 << 2); };
    if (column2&(1<<2)) { if (!(column2d&(1<<2))) { code = (code<<4)+8; column2d |= (1 << 2); } } else { column2d &= ~(1 << 2); };
    if (column3&(1<<2)) { if (!(column3d&(1<<2))) { code = (code<<4)+9; column3d |= (1 << 2); } } else { column3d &= ~(1 << 2); };
    if (column2&(1<<3)) { if (!(column2d&(1<<3))) { code = (code<<4)+0; column2d |= (1 << 3); } } else { column2d &= ~(1 << 3); };
    if (column1&(1<<3)) { if (!(column1d&(1<<3))) { code = 0; column1d |= (1 << 3); } } else { column1d &= ~(1 << 3); };
    if (column3&(1<<3)) { if (!(column3d&(1<<3))) { codeCheck(); column3d |= (1 << 3); } } else { column3d &= ~(1 << 3); };
    if (column1||column2||column3) {
      lcdWelcome();
      timeout = 10;
    }
  }
}

bool loggedin() {
  if (user_name.equals("")) return false; //Loading saldo
  return !sha256string.equals(""); //Is set when key has been scanned
}

void logout() {
  sha256string = "";
  user_name = "";
  lcd_message = "";
  user_saldo = "";
  code = 0;
  product = 0;
  Serial.println("Logged out.");
  lcd.clear();
  lcd.print("Logged out.");
}

void lcdUserHome() {
  if (loggedin()&&(!isVending)&&(!isCodeCheck)) {
    lcd.clear();
    lcd.print(user_name);
    lcd.setCursor(0,1);
    lcd.print("Saldo: "+user_saldo);
    lcd.setCursor(14,1);
    lcd.print(String((product&0xF0)>>4)+String(product&0x0F)); //Product selection
    //lcd.setCursor(14,0);
    //lcd.print(String((timeout&0xF0)>>4)+String(timeout&0x0F)); //Timeout counter
  }
}

void lcdWelcome() {
  if ((!isVending)&&(!isCodeCheck)) {
    lcd.clear();
    lcd.print(lcd_welcome_message);
    lcd.setCursor(0,1);
    uint16_t realcode = (code&0x000F)+((code&0x00F0)>>4)*10+((code&0x0F00)>>8)*100+((code&0xF000)>>12)*1000;
    if (realcode>0) {
      lcd.print("PIN: "+String((code&0xF000)>>12)+String((code&0x0F00)>>8)+String((code&0x00F0)>>4)+String(code&0x000F));
    }
  }
}

void lcdVending() {
  lcd.clear();
  lcd.print("Vending...");
  timeout = 10;
}

void lcdCodeCheck() {
  lcd.clear();
  lcd.print("Please wait...");
  timeout = 10;
}

void lcdError() {
  lcd.clear();
  lcd.print("Comm. error!");
  timeout = 20;
}

void kpoutLatch() {
  digitalWrite(KPOUT_LAT, HIGH);
  delay(10);
  digitalWrite(KPOUT_LAT, LOW);
}

void pressKeyOnController(uint8_t key) {
  Serial.println("Pressing button "+String(key)+"...");
  uint32_t data = 0;
  //1 - 1.3
  //2 - 2.3
  //3 - 3.3
  //4 - 1.2
  //5 - 2.2
  //6 - 3.2
  //7 - 1.1
  //8 - 2.1?
  //9 - 3.1?
  //A - 1.0
  //0 - 2.0
  //B - 3.0
  /*shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 1<<1);
  digitalWrite(KPOUT_LAT, HIGH);
  delay(10);
  digitalWrite(KPOUT_LAT, LOW);
  delay(1000);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  digitalWrite(KPOUT_LAT, HIGH);
  delay(10);
  digitalWrite(KPOUT_LAT, LOW);
  delay(5000);*/
  switch (key) {
    case 0:
           //  ----369B----2580----147A
      data = 0b000000000000000100000000;
      break;
    case 1:
           //  ----369B----2580----147A
      data = 0b000000000000000000001000;
      break;
    case 2:
           //  ----369B----2580----147A
      data = 0b000000000000100000000000;
      break;
    case 3:
           //  ----369B----2580----147A
      data = 0b000010000000000000000000;
      break;
    case 4:
           //  ----369B----2580----147A
      data = 0b000000000000000000000100;
      break;
    case 5:
           //  ----369B----2580----147A
      data = 0b000000000000010000000000;
      break;
    case 6:
           //  ----369B----2580----147A
      data = 0b000001000000000000000000;
      break;
    case 7:
           //  ----369B----2580----147A
      data = 0b000000000000000000000010;
      break;
    case 8:
           //  ----369B----2580----147A
      data = 0b000000000000001000000000;
      break;
    case 9:
           //  ----369B----2580----147A
      data = 0b000000100000000000000000;
      break;
    case 10:
           //  ----369B----2580----147A
      data = 0b000000000000000000000001;
      break;
    case 11:
           //  ----369B----2580----147A
      data = 0b000000010000000000000000;
      break;
    default:
      data = 0;
  }
  Serial.println("SHIFT: "+String((uint8_t) (data)&0x0F)+", "+String((uint8_t) (data>>8)&0x0F)+", "+String((uint8_t) (data>>16)&0x0F));
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, (uint8_t) (data)&0x0F);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, (uint8_t) (data>>8)&0x0F);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, (uint8_t) (data>>16)&0x0F);
  kpoutLatch();
  delay(125);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  shiftOut(KPOUT_DAT, KPOUT_CLK, MSBFIRST, 0);
  kpoutLatch(); 
  delay(125);
}

uint8_t decToBcd(uint8_t dec) {
  uint8_t result = 0;
  
  result |= (dec / 10) << 4;
  result |= (dec % 10) << 0;
  
  return result;
}

uint8_t bcdToDec(uint8_t bcd) {
  uint8_t result = 0;
  
  result += ((bcd & 0b11110000) >> 4) * 10;
  result += (bcd & 0b00001111);

  return result;  
}

void pressKeysOnController(uint8_t pr) {
  Serial.println("pressKeysOnController("+String(pr)+")!");
  uint8_t bcd = decToBcd(pr);
  pressKeyOnController(bcd>>4);
  pressKeyOnController(bcd&0x0F);
  delay(200);
}

void vendError() {
  lcd.clear();
  lcd.print("Unknown product.");
  Serial.print("Unknown product.");
  delay(1000);
}

void doActualVend(uint8_t to_vend) {
  Serial.println("--- VEND PRODUCT "+String(to_vend)+" ---");
  switch(to_vend) {
    case 11:
    case 13:
    case 15:
    case 21:
    case 23:
    case 25:
    case 31:
    case 33:
    case 35:
    case 41:
    case 43:
    case 45:
    case 51:
    case 53:
    case 55:
    case 61:
    case 62:
    case 63:
    case 64:
    case 65:
    case 66:
      lcd.clear();
      lcd.print("In progress...  ");
      lcd.setCursor(0,1);
      lcd.print("               >");
      pressKeysOnController(to_vend);
      break;
    default:
      vendError();
  }
  isVending = false;
  isCodeCheck = false;
  lcd.clear();
  lcd.print("In progress...  ");
  //lcd.setCursor(0,1);
  //lcd.print("                ");
  delay(2000);
  lcd.clear();
  lcd.print("COMPLETED.      ");
  lcd.setCursor(0,1);
  lcd.print("HAVE A NICE DAY!");
  delay(1000);
}


/*
 * Button pad:
 *  1. C1 DGroen
 *  2. R1 Roze
 *  3. C3 Rood
 *  4. R2 Blauw
 *  5. C2 Wit
 *  6. R3 LGroen
 *  7. -
 *  8. R4 Zwart
 */
