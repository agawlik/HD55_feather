# HD55_feather
AlorAir HD55 interface from Feather M0 Wifi module and MCP2515 feather

# Hardware
* [Adafruit Feather M0 WiFi - ATSAMD21 + ATWINC1500](https://www.adafruit.com/product/3010)
* [Adafruit CAN Bus FeatherWing - MCP2515](https://www.adafruit.com/product/5709)

![Adafruit Feather M0 WiFi - ATSAMD21 + ATWINC1500](https://cdn-learn.adafruit.com/assets/assets/000/110/927/original/adafruit_products_Adafruit_Feather_M0_WINC1500_Pinout.png?1650391656)

See [Feather M0 Wifi Arduino setup](https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/setup). Note that WIFI firmware update must be performed with Arduino IDE v1.x.

| Pin | Use |
| --- | --- |
| 2 | WIFI ENA | 
| 4 | WIFI RST | 
| 5 | MCP2515 CS | 
| 7 | WIFI IRQ | 
| 8 | WIFI CS | 
| 13 | ID - #1) GND, #2) pull-up high | 
| 22 | MISO | 
| 23 | MOSI | 
| 24 | SCK |

## HD55 CanBus Protocol
My HD55 units are configured for 125kbps and have the 0x123 response format described in [this forum post](https://community.home-assistant.io/t/automate-alorair-sentinel-hd55-dehumidifier/306084/22). An older, more rigorous description is available at [tinymicros wiki](https://tinymicros.com/wiki/AlorAir_Sentinel_HD55_Dehumidifier).

```
0x123 RH_actual RH_set Temp D3 Status D5 D6 D7

Temp is in celsius
Statusbit1 = 1 → Unit is running
Statusbit0 = 1 → Unit is on
Statusbit0 = 0 → Unit is off
(bit0 is Least Significant Bit)
```

An example of a received packet while the system is running (all in hex): `30 32 16 0 3 0 0 0` where setpoint is RH=50% (0x32).

# Sketches
All sketches include a `private_config.h` file that contains Adafruit IO key and wifi network SSID and password. This private configuration template can be copied from the `adafruitio_00_publish` example sketch.

Note that all sketches using the Feather M0 WiFi require the following:
```
//Configure pins for Adafruit ATWINC1500 Feather
WiFi.setPins(8,7,4,2);
```

## HD55_adafruitio_publish
This is an adaption of the adafruitio_00_publish sketch and MCP2515 example to request telemetry from the HD55 and then publish the data to a number of Adafruit IO feeds every 1 minute.

