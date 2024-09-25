// Adafruit IO Publish Example
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the private_config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "private_config.h"

//! MCP2515 CanBus FeatherWing CS
#define MCP2515_CS_PIN (5) 
//! 125kbps CAN baudrate
#define CAN_BAUDRATE (125000)
//! polling period [seconds]
#define POLL_DELAY (60000)

//! Force wait for serial monitor before initializing
//#define WAIT_FOR_SERIAL_MONITOR (1)

//! Serial monitor baudrate
#define SERIAL_MONITOR_BAUD (115200)

/************************ Example Starts Here *******************************/

#include <string.h>
#include <Adafruit_MCP2515.h>

Adafruit_MCP2515 mcp(MCP2515_CS_PIN); //!< CanBus FeatherWing

// set up the adafruit IO feeds
AdafruitIO_Feed *io_binary = io.feed("hd55-1.hd55-binary");
AdafruitIO_Feed *io_rh = io.feed("hd55-1.relative-humidity");
AdafruitIO_Feed *io_temperature = io.feed("hd55-1.temperature");
AdafruitIO_Feed *io_setpoint = io.feed("hd55-1.setpoint");
AdafruitIO_Feed *io_status = io.feed("hd55-1.status");

void setup() {

  // start the serial connection
  Serial.begin(SERIAL_MONITOR_BAUD);

#ifdef WAIT_FOR_SERIAL_MONITOR
  // wait for serial monitor to open
  while (!Serial)
    ;
#endif

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while (1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {

  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

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

  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events.
  delay(POLL_DELAY);
}
