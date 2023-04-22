# ISS-MQTT-Gateway
Receive data from Davis Vantage Vue ISS - Weather Station and publish Measurements via MQTT

![Davis Vantage VUE ISS](/doc/vantage_vue_iss.jpg)
# Hardware
  * ESP 32 
  * Hope RFM69 Module @ 868MHz

## Wireing
![Schematic](/doc/schematic.png)


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
    * "Payload": "80:02:e1:1e:1b:05:b5:b3", 
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
# Example JSON-Data
![Example JSON-Data](/doc/jsondata.png)
# Available MQTT-Commands 
* Commands must be published to topic `[PREFIX]/cmd`
* Responses are published to `[PREFIX]/result`

## Reset Daily Rain-Click Counter to 0
### `newday`
 Example:
 * command: `newday` 
 * response: `Daily Rain-Click counter set to 0` :

## Set Rain-Click Counter
### `setrc [newvalue]`
 Example:
 * command: `setrc 42` 
 * response: `Raincounter set to 42`

## Reset Statistics
Set Statistical values to 0:
 * Longest Time without reception 
 * Number of packets with correct CRC
 * How often did we HOP because of missing packets
 * How often did we have to resync
 * Number of uninterruptedly receiverd correct packages
 * Maximum Number of uninterruptedly receiverd correct packages
 * Number of packets with CRC ERROR    
### `reset`
Example:
 * command: `reset` 
 * response: `Raincounter set to 42`
 
## Reboot ESP32
### `reboot`
Example:
 * command: `reboot` 
 * response: `Rebooting in 5 seconds ... [please standby].` 


## Pictures
### ESP32 Board with Connections
![ESP32](/doc/01-ESP32.jpg)
### RFM69 Board with Connections
![RFM69](/doc/02-RFM69.jpg)
### Both Board
![Both Boards](/doc/03-BothBoards.jpg)
### In the Box with exposed Antenna 
![In the Box with Antenna exposed](/doc/04-BoxWithAntenna.jpg)
### Antenna covered
![Antenna covered](/doc/05-BoxAntennaCover.jpg)
### 5V Power Supply
![Power Supply Cable](/doc/07-PowerSupply.jpg)
### Installed
![Installed](/doc/08-BoxInstalled.jpg)
