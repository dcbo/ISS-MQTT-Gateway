Howto OTA-Flash Releases 

To Flash a Release manualy with espota.py:

python3 espota.py -i <ESP-IP> -I <HOST-IP> -p <ESP-PORT> -P <HOST-PORT> -a PASSWORD -f FILENAME

python3 espota.py -d -r -i 192.168.1.123 -I 0.0.0.0 -p 3232 -P 58571 -a UpdateAccessESP32 -f 1.0.1_OTA-Test\firmware.bin

######################################################################################

espota.py
usage: espota.py [--ip ESP-IP] [--host-ip HOST-IP]
                 [--port ESP-PORT] [--host-port HOST-PORT]  
                 [--auth PASSWORD] 
                 [--debug] [--progress]
                 [--file FILENAME]
                 [--spiffs SPIFFSNAME]

arguments:
  --ip ESP-IP, -i ESP-IP                 ESP IP-Address.
  --host_ip HOST-IP, -I HOST-IP          Host IP-Address. (This Host, 0.0.0.0) for all Interfaces.
  --port ESP-PORT, -p ESP-PORT           ESP OTA-Port. Default 8266.
  --host-port HOST-PORT, -P HOST-PORT    Host server OTA-Port. Default random 10000-60000".
  --auth PASSWORD, -a PASSWORD           Authentication Password.
  --file FILENAME, -f FILENAME           Image file to program to ESP Device.
  --spiffs SPIFFSNAME, -s SPIFFSNAME     Use this option to transmit a SPIFFS image and do not flash the module.
  --debug, -d                            Show debug output. And override loglevel with debug.
  --progress, -r                         Show progress output.

python3 espota.py -i <ESP-IP> -I <HOST-IP> -p <ESP-PORT> -P <HOST-PORT> -a PASSWORD -f FILENAME

######################################################################################
