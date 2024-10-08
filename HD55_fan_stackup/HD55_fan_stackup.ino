// edit the private_config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include <string.h>
#include "private_config.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "RTClib.h"
#include "Adafruit_SHT4x.h"
#include "Adafruit_MCP2515.h"
#include "Adafruit_EEPROM_I2C.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/************************** Configuration ***********************************/

//! MCP2515 CanBus FeatherWing CS
#define MCP2515_CS_PIN (5) 
//! 125kbps CAN baudrate
#define CAN_BAUDRATE (125000)
//! polling period [seconds]
#define POLL_DELAY (10000)

//! Board ID pin
#define BOARD_ID_PIN (13) 

//! Force wait for serial monitor before initializing
// #define WAIT_FOR_SERIAL_MONITOR (1)

//! Serial monitor baudrate
#define SERIAL_MONITOR_BAUD (115200)

#define EEPROM_ADDR 0x50  // the default address!

/************************ Global *******************************/

Adafruit_MCP2515 mcp(MCP2515_CS_PIN); //!< CanBus FeatherWing

uint8_t g_board_id; //!< board ID

char ssid[] = WIFI_SSID;    //!< network SSID (name)
char pass[] = WIFI_PASS;    //!< network password (use for WPA, or use as key for WEP)

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

Adafruit_EEPROM_I2C i2ceeprom;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// set up the adafruit IO feeds
AdafruitIO_Feed *io_binary      = NULL;
AdafruitIO_Feed *io_rh          = NULL;
AdafruitIO_Feed *io_temperature = NULL;
AdafruitIO_Feed *io_setpoint    = NULL;
AdafruitIO_Feed *io_status      = NULL;

void setup() {

  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);

  // start the serial connection
  Serial.begin(SERIAL_MONITOR_BAUD);

#ifdef WAIT_FOR_SERIAL_MONITOR
  // wait for serial monitor to open
  while (!Serial)
    ;
#endif

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  int wifi_status = WL_IDLE_STATUS;

  // attempt to connect to WiFi network:
  while ( wifi_status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifi_status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWiFiStatus();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  // Serial.println("\nStarting connection to server...");
  // Udp.begin(localPort);
  timeClient.begin();
  // delay(1000);
  if (timeClient.forceUpdate() == false)
  {
    Serial.println("Failed to force update NTP");
  }

  if (timeClient.isTimeSet())
  {
    rtc.adjust(DateTime(timeClient.getEpochTime()));
    Serial.println("Setting RTC from NTP");
    Serial.print("NTP: ");
    Serial.println(timeClient.getFormattedTime());
  }

  if (rtc.lostPower()) {
    Serial.println("RTC has no set time!");
  }
  else
  {
    Serial.println("RTC maintained power");
  }

  sht40Setup();

  eepromSetup();
  
  mcp2515Setup();

  displaySetup();

  /*
   * Board ID
   */
  //configure Board ID pin as an input and enable the internal pull-up resistor
  pinMode(BOARD_ID_PIN, INPUT_PULLUP);
  delay(100);
  g_board_id = digitalRead(BOARD_ID_PIN);
  if (g_board_id == 1u)
  {
    io_binary      = io.feed("hd55-2.hd55-binary");
    io_rh          = io.feed("hd55-2.relative-humidity");
    io_temperature = io.feed("hd55-2.temperature");
    io_setpoint    = io.feed("hd55-2.setpoint");
    io_status      = io.feed("hd55-2.status");
    Serial.println("Connected to SID 2");
  }
  else
  {
    io_binary      = io.feed("hd55-1.hd55-binary");
    io_rh          = io.feed("hd55-1.relative-humidity");
    io_temperature = io.feed("hd55-1.temperature");
    io_setpoint    = io.feed("hd55-1.setpoint");
    io_status      = io.feed("hd55-1.status");
    Serial.println("Connected to SID 1");
  }

  /*
   * Adafruit IO
   */
  do 
  {
    Serial.println("Connecting to Adafruit IO");
  
    // connect to io.adafruit.com
    io.connect();

    // wait for a connection
    size_t counter = 0;
    while ((io.status() < AIO_CONNECTED) && (++counter < 20))
    {
      Serial.println(io.statusText());
      // Serial.print("status: ");
      // Serial.println(io.status(), DEC);
      //Serial.print(".");
      delay(1000);
    }
  } while (io.status() < AIO_CONNECTED);

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

}

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  sht40Loop();
  
  displayLoop();

  rtcTest();

  // mcp2515Loop();

  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events.
  delay(POLL_DELAY);
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void rtcTest(void)
{
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.println(" C");

    Serial.println();
}

void mcp2515Setup(void)
{
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while (1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void mcp2515Loop(void)
{
  char packet_data[17] = {0};
#if 1
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");
  mcp.beginPacket(0x123);
  mcp.write(0x01);  // always one
  mcp.write(0x00);  // no button pressed
  mcp.write(0x00);  // 0x00 (except when button pressed, see above)
  mcp.write(50);    // relative humidity
  mcp.write(18);    // temperature [C]
  mcp.write(0x00);
  mcp.write(0x00);
  mcp.write(0x00);
  mcp.endPacket();
  Serial.println("done");

  Serial.print("Receiving packet ... ");
  int packetSize = 0;
  auto start_time = millis();
  while (packetSize == 0) {
    packetSize = mcp.parsePacket();
    if (millis() - start_time > POLL_DELAY)
    {
      Serial.println("timeout");
      return;
    }
  }

  Serial.print("Received ");

  if (mcp.packetExtended()) {
    Serial.print("extended ");
  }

  if (mcp.packetRtr()) {
    // Remote transmission request, packet contains no data
    Serial.print("RTR ");
  }

  Serial.print("packet with id 0x");
  Serial.print(mcp.packetId(), HEX);

  if (mcp.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(mcp.packetDlc());
  } else {
    Serial.print(" and length ");
    Serial.println(packetSize);

    // only print packet data for non-RTR packets
    for (int i = 0; i < 8; i++) {
      int read_data = mcp.read();
      snprintf(&packet_data[i * 2], 3, "%02X", read_data);
      Serial.print(read_data, HEX);
      Serial.print(" ");
      switch (read_data)
      {
        case 0: // RH_actual
          io_rh->save(read_data);
          break;
        case 1: // RH_setpoint
          io_setpoint->save(read_data);
          break;
        case 2: // Temperature
          io_temperature->save(read_data);
          break;
        case 4: // Status
          io_status->save(read_data);
          break;
        default:
          break;
      }
    }
    Serial.println();
  }
#else
  memcpy(packet_data, "3032160003000000", 16);
  char value[3] = {0};
  memcpy(value, &packet_data[0], 2);
  io_rh->save(value);
  memcpy(value, &packet_data[2], 2);
  io_setpoint->save(value);
  memcpy(value, &packet_data[4], 2);
  io_temperature->save(value);
  memcpy(value, &packet_data[8], 2);
  io_status->save(value);
#endif

  // save count to the feed on Adafruit IO
  Serial.print("sending -> ");
  Serial.write(packet_data,16);
  Serial.println();

  io_binary->save(packet_data);


  Serial.println();
}

void sht40Setup(void)
{
  Serial.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  Serial.println("Found SHT4x sensor");
  Serial.print("Serial number 0x");
  Serial.println(sht4.readSerial(), HEX);

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }

  // You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_NO_HEATER);
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }

}

void sht40Loop(void)
{
  sensors_event_t humidity, temp;
  
  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp;

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);  
}

void eepromSetup(void)
{
  if (i2ceeprom.begin(EEPROM_ADDR)) {  // you can stick the new i2c addr in here, e.g. begin(0x51);
    Serial.println("Found I2C EEPROM");
  } else {
    Serial.println("I2C EEPROM not identified ... check your connections?\r\n");
    while (1) delay(10);
  }
  
  // Read the first byte
  uint8_t test = i2ceeprom.read(0x0);
  Serial.print("Restarted "); Serial.print(test); Serial.println(" times");
  // Test write ++
  test++;
  i2ceeprom.write(0x0, test);
/*
  // Try to determine the size by writing a value and seeing if it changes the first byte
  Serial.println("Testing size!");

  uint32_t max_addr;
  for (max_addr = 1; max_addr < 0xFFFF; max_addr++) {
    if (i2ceeprom.read(max_addr) != test)
      continue; // def didnt wrap around yet

    // maybe wraped? try writing the inverse
    if (! i2ceeprom.write(max_addr, (byte)~test)) {
        Serial.print("Failed to write address 0x");
        Serial.println(max_addr, HEX);
    }

    // read address 0x0 again
    uint8_t val0 = i2ceeprom.read(0);

    // re-write the old value
    if (! i2ceeprom.write(max_addr, test)) {
        Serial.print("Failed to re-write address 0x");
        Serial.println(max_addr, HEX);
    }    

    // check if addr 0 was changed
    if (val0 == (byte)~test) {
      Serial.println("Found max address");
      break;
    }
  }
  Serial.print("This EEPROM can store ");
  Serial.print(max_addr);
  Serial.println(" bytes");
    
  // dump the memory
  uint8_t val;
  for (uint16_t addr = 0; addr < max_addr; addr++) {
    val = i2ceeprom.read(addr);
    if ((addr % 32) == 0) {
      Serial.print("\n 0x"); Serial.print(addr, HEX); Serial.print(": ");
    }
    Serial.print("0x"); 
    if (val < 0x10) 
      Serial.print('0');
    Serial.print(val, HEX); Serial.print(" ");
  }
  */
}

void eepromLoop(void)
{
  
}

void displaySetup(void)
{
  Serial.println("128x64 OLED FeatherWing test");
  delay(250); // wait for the OLED to power up
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  // delay(1000);
}

void displayLoop(void)
{
  sensors_event_t humidity, temp;
  
  if (sht4.getEvent(&humidity, &temp) == true)
  {
    display.clearDisplay();
    // display.display();

    display.setRotation(1);

    // text display tests
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.print("T:");
    display.print(temp.temperature);
    display.println(" C");
    display.print("H:");
    display.print(humidity.relative_humidity);
    display.println(" %");

    DateTime now = rtc.now();
    display.print(now.month(), DEC);
    display.print('/');
    display.println(now.day(), DEC);
    display.print(now.hour(), DEC);
    display.print(':');
    display.print(now.minute(), DEC);
    display.print(':');
    display.print(now.second(), DEC);

    display.display(); // actually display all of the above
  }
}
