# ISS-MQTT-Gateway
Receive data from Davis Vantage Vue ISS - Weather Station and publish Measurements via MQTT

# Hardware
  * ESP 32 
  * Hope RFM69 Module @ 868MHz

## Wireing
| ESP 32 Pin |  Dir   | RWM69 Pin | Signal |
|  3.3V      |  --->   |   3.3V   | RFM69 Power-Supply |
|   GND      |  <-->   |   GND    | Ground | 
|   D23      |  --->   |   MOSI   | SPI-Master Out Slave In | 
|   D19      |  <---   |   MISO   | SPI-Master In Slave Out | 
|   D18      |  --->   |   SCK    | SPI-Clock |
|    D2      |  <---   |   DIO0   | Receive Interrupt |
|    D5      |  --->   |   NSS    | Chip Select | 
|            |         |   ANA     | Antenna 8,6cm (Lambda/4 Dipole) |

## Power Supply
  * ESP 32 via USB
  * Hope RFM69 via ESP32

# Functionality
  * Receive Data from Davis Vanage ISS Weather Station
  * Decode Raw Data to Weather Measurements
  * This Measurments will be decoded:
    * WindSpeed [km/h]
    * WindDirection [0-359°]
    * BattWarning [0|1]
    * Additional Measurments: 
      * GustSpeed [km/h]
      * OutsideTemperature [°C]
      * OutsideHumidity [%rel]
      * Rainrate [mm/h]
      * SolarRadiation 
      * GoldcapVoltage [V]
  * Publish Weather Measurements after each Packet received (every 2.5s) as Json Data e.g:
    * "WindSpeed": 3.22,  
    * "WindDirection": 257,
    * "BattWarning": 0, 
    * "Payload": "80:02:37:1b:ab:0d:4e:dc", 
    * "Channel":1, 
    * "RSSI":-74,
    * "msgID":8,
    * "OutsideTemperature":6.67
  * Each Packet from the Davis ISS contains only one additional Measurment.  
# Core-System-Functionality
* Wifi Connection  
* MQTT Connection 
* Self Monitoring connectivity and reconnect on connection loss
* Command Parser accepts commends over MQTT
* MQTT Status Topic, retained, with LastWill
* CRON System which sends different MQTT Topics every 10s, 30s and 60s
* Automatic Versioning System
  * Version Number is incremented after Upload to Production Target
# Device Specific-Functionality
* Write Text on Display
* Scroll Text when Display is full
* Print text received via MQTT on Display

# Available MQTT-Commands 
* Commands must be published to topic `[PREFIX]/cmd`
* Responses are published to `[PREFIX]/result`

## Print Text on Display and scroll
### `display "String"`
Display `String` on the LCD, scroll if needed
 Example:
 * command: `display "Hello World"` 

## System related Commands
### `reset`
Reboot ESP32

Example:
 * command: `reset` 
 
