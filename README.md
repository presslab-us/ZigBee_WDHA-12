# ZigBee_WDHA-12

ZigBee modification for WDHA-12 Homelink gateway

### Usage:
To reset ZigBee hold buttons 1,2,3 until all 3 LEDs are lit and hold for 2+ seconds.

SHOCK HAZARD, internal power supply is not isolated, do not touch circuitry while plugged in.
DO NOT plug into AC mains for CC debugger programming.  Use only 9V battery and hold a button down so 3.3V supply stays active, otherwise damage will result!

### Parts needed:
Wayne Dalton WDHA-12
Replace 2.7uF 250V cap with 3.9uF 250V Panasonic ECQ-E2395KF, digikey.com
2x5 pin header M20-9970546, digikey.com
qty 3, 1N4001 diode, digikey.com
CC2530 SZ2 dev board with LNA, goodluckbuy.com
CC debugger, eBay.com

### Connections:
P1_4 connects to 1N4001 anode to LED1 on cathode
P1_5 connects to 1N4001 anode to LED2 on cathode
P1_6 connects to 1N4001 anode to LED3 on cathode
P2_1 connects to debug DC
P2_2 connects to debug CC
RST_N connects to debug RST
GND, 3.3V connect to GND, 3.3V, debug
P1_0 jumper wire to P1_2 for LNA

### Build requirements:
Install Z-STACK-HOME 1.2.2a
Place Homelink folder into C:\Texas Instruments\Z-Stack Home 1.2.2a.44539\Projects\zstack\HomeAutomation

### Programming:
HEX file is located in Homelink\CC2530DB\RouterEB\Exe\Homelink.hex
