EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 10
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 4000 3400 0    50   Input ~ 0
STEP
Text HLabel 4000 3300 0    50   Input ~ 0
DIR
Text HLabel 4000 3600 0    50   Input ~ 0
SDI
Text HLabel 4000 3700 0    50   Input ~ 0
SCK
Text HLabel 3800 3800 0    50   Output ~ 0
SDO
Text HLabel 4000 3900 0    50   Input ~ 0
CS
Wire Wire Line
	4000 3300 4350 3300
Wire Wire Line
	4350 3400 4000 3400
Wire Wire Line
	4000 3600 4350 3600
Wire Wire Line
	4350 3700 4000 3700
Wire Wire Line
	3800 3800 4150 3800
Wire Wire Line
	4350 3900 4000 3900
Text HLabel 6000 3200 2    50   Input ~ 0
EN
Wire Wire Line
	5750 3200 6000 3200
$Comp
L power:GND #PWR0108
U 1 1 60870DCB
P 5750 4400
AR Path="/60869CEA/60870DCB" Ref="#PWR0108"  Part="1" 
AR Path="/608DFF6C/60870DCB" Ref="#PWR0116"  Part="1" 
AR Path="/608FE26D/60870DCB" Ref="#PWR0123"  Part="1" 
AR Path="/608FE277/60870DCB" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0123" H 5750 4150 50  0001 C CNN
F 1 "GND" H 5755 4227 50  0000 C CNN
F 2 "" H 5750 4400 50  0001 C CNN
F 3 "" H 5750 4400 50  0001 C CNN
	1    5750 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4200 5750 4400
$Comp
L power:VBUS #PWR0109
U 1 1 608718E9
P 6050 2650
AR Path="/60869CEA/608718E9" Ref="#PWR0109"  Part="1" 
AR Path="/608DFF6C/608718E9" Ref="#PWR0117"  Part="1" 
AR Path="/608FE26D/608718E9" Ref="#PWR0124"  Part="1" 
AR Path="/608FE277/608718E9" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0124" H 6050 2500 50  0001 C CNN
F 1 "VBUS" H 6065 2823 50  0000 C CNN
F 2 "" H 6050 2650 50  0001 C CNN
F 3 "" H 6050 2650 50  0001 C CNN
	1    6050 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2900 5850 2900
Wire Wire Line
	5850 2900 5850 2750
$Comp
L power:+3V3 #PWR0110
U 1 1 60872ABF
P 5850 2750
AR Path="/60869CEA/60872ABF" Ref="#PWR0110"  Part="1" 
AR Path="/608DFF6C/60872ABF" Ref="#PWR0118"  Part="1" 
AR Path="/608FE26D/60872ABF" Ref="#PWR0125"  Part="1" 
AR Path="/608FE277/60872ABF" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0125" H 5850 2600 50  0001 C CNN
F 1 "+3V3" H 5865 2923 50  0000 C CNN
F 2 "" H 5850 2750 50  0001 C CNN
F 3 "" H 5850 2750 50  0001 C CNN
	1    5850 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3000 6050 3000
$Comp
L TMC2208_SILENTSTEPSTICK:TMC5160_SILENTSTEPSTICK U?
U 1 1 6086E8C9
P 5050 3500
AR Path="/6086E8C9" Ref="U?"  Part="1" 
AR Path="/60869CEA/6086E8C9" Ref="U2"  Part="1" 
AR Path="/608DFF6C/6086E8C9" Ref="U5"  Part="1" 
AR Path="/608FE26D/6086E8C9" Ref="U7"  Part="1" 
AR Path="/608FE277/6086E8C9" Ref="U9"  Part="1" 
F 0 "U7" H 5050 4367 50  0000 C CNN
F 1 "TMC5160_SILENTSTEPSTICK" H 5050 4276 50  0000 C CNN
F 2 "Trinamic:MODULE_TMC_SILENTSTEPSTICK" H 5050 3500 50  0001 L BNN
F 3 "" H 5050 3500 50  0001 L BNN
F 4 "N/A" H 5050 3500 50  0001 L BNN "MAXIMUM_PACKAGE_HEIGHT"
F 5 "v11" H 5050 3500 50  0001 L BNN "PARTREV"
F 6 "Trinamic" H 5050 3500 50  0001 L BNN "MANUFACTURER"
F 7 "Manufacturer Recommendations" H 5050 3500 50  0001 L BNN "STANDARD"
F 8 "1460-TMC5160SILENTSTEPSTICK-ND" H 5050 3500 50  0001 C CNN "Digi-Key_PN"
	1    5050 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C1
U 1 1 60876F12
P 6650 2750
AR Path="/60869CEA/60876F12" Ref="C1"  Part="1" 
AR Path="/608DFF6C/60876F12" Ref="C2"  Part="1" 
AR Path="/608FE26D/60876F12" Ref="C3"  Part="1" 
AR Path="/608FE277/60876F12" Ref="C4"  Part="1" 
F 0 "C3" H 6765 2796 50  0000 L CNN
F 1 "10uF" H 6765 2705 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.7" H 6650 2750 50  0001 C CNN
F 3 "~" H 6650 2750 50  0001 C CNN
F 4 "PCE4976CT-ND" H 6650 2750 50  0001 C CNN "Digi-Key_PN"
	1    6650 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 608785AC
P 6650 2900
AR Path="/60869CEA/608785AC" Ref="#PWR0111"  Part="1" 
AR Path="/608DFF6C/608785AC" Ref="#PWR0119"  Part="1" 
AR Path="/608FE26D/608785AC" Ref="#PWR0126"  Part="1" 
AR Path="/608FE277/608785AC" Ref="#PWR0133"  Part="1" 
F 0 "#PWR0126" H 6650 2650 50  0001 C CNN
F 1 "GND" H 6655 2727 50  0000 C CNN
F 2 "" H 6650 2900 50  0001 C CNN
F 3 "" H 6650 2900 50  0001 C CNN
	1    6650 2900
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0112
U 1 1 60878CAB
P 6650 2600
AR Path="/60869CEA/60878CAB" Ref="#PWR0112"  Part="1" 
AR Path="/608DFF6C/60878CAB" Ref="#PWR0120"  Part="1" 
AR Path="/608FE26D/60878CAB" Ref="#PWR0127"  Part="1" 
AR Path="/608FE277/60878CAB" Ref="#PWR0134"  Part="1" 
F 0 "#PWR0127" H 6650 2450 50  0001 C CNN
F 1 "VBUS" H 6665 2773 50  0000 C CNN
F 2 "" H 6650 2600 50  0001 C CNN
F 3 "" H 6650 2600 50  0001 C CNN
	1    6650 2600
	1    0    0    -1  
$EndComp
$Comp
L Isolator:LTV-817 U3
U 1 1 608834B7
P 4950 1750
AR Path="/60869CEA/608834B7" Ref="U3"  Part="1" 
AR Path="/608DFF6C/608834B7" Ref="U4"  Part="1" 
AR Path="/608FE26D/608834B7" Ref="U6"  Part="1" 
AR Path="/608FE277/608834B7" Ref="U8"  Part="1" 
F 0 "U6" H 4950 2075 50  0000 C CNN
F 1 "LTV-817" H 4950 1984 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W8.89mm_SMDSocket_LongPads" H 4750 1550 50  0001 L CIN
F 3 "http://www.us.liteon.com/downloads/LTV-817-827-847.PDF" H 4950 1650 50  0001 L CNN
F 4 "C106900" H 4950 1750 50  0001 C CNN "LCSC"
F 5 "160-1893-1-ND" H 4950 1750 50  0001 C CNN "Digi-Key_PN"
	1    4950 1750
	-1   0    0    -1  
$EndComp
Text HLabel 4400 1850 0    50   Output ~ 0
Limit
Wire Wire Line
	4650 1850 4500 1850
$Comp
L power:+3V3 #PWR0113
U 1 1 60889A9D
P 4650 1150
AR Path="/60869CEA/60889A9D" Ref="#PWR0113"  Part="1" 
AR Path="/608DFF6C/60889A9D" Ref="#PWR0121"  Part="1" 
AR Path="/608FE26D/60889A9D" Ref="#PWR0128"  Part="1" 
AR Path="/608FE277/60889A9D" Ref="#PWR0135"  Part="1" 
F 0 "#PWR0128" H 4650 1000 50  0001 C CNN
F 1 "+3V3" H 4665 1323 50  0000 C CNN
F 2 "" H 4650 1150 50  0001 C CNN
F 3 "" H 4650 1150 50  0001 C CNN
	1    4650 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 6088A387
P 4650 2300
AR Path="/60869CEA/6088A387" Ref="#PWR0114"  Part="1" 
AR Path="/608DFF6C/6088A387" Ref="#PWR0122"  Part="1" 
AR Path="/608FE26D/6088A387" Ref="#PWR0129"  Part="1" 
AR Path="/608FE277/6088A387" Ref="#PWR0136"  Part="1" 
F 0 "#PWR0129" H 4650 2050 50  0001 C CNN
F 1 "GND" H 4655 2127 50  0000 C CNN
F 2 "" H 4650 2300 50  0001 C CNN
F 3 "" H 4650 2300 50  0001 C CNN
	1    4650 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1850 4650 1950
$Comp
L Device:R_Small_US R27
U 1 1 60890195
P 5250 1400
AR Path="/60869CEA/60890195" Ref="R27"  Part="1" 
AR Path="/608DFF6C/60890195" Ref="R30"  Part="1" 
AR Path="/608FE26D/60890195" Ref="R33"  Part="1" 
AR Path="/608FE277/60890195" Ref="R36"  Part="1" 
F 0 "R33" H 5318 1446 50  0000 L CNN
F 1 "100" H 5318 1355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 5250 1400 50  0001 C CNN
F 3 "~" H 5250 1400 50  0001 C CNN
F 4 "C105588" H 5250 1400 50  0001 C CNN "LCSC"
	1    5250 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1650 5250 1500
Wire Wire Line
	4650 1200 5250 1200
Wire Wire Line
	5250 1200 5250 1300
Wire Wire Line
	4650 1150 4650 1200
Connection ~ 4650 1200
$Comp
L Device:LED D?
U 1 1 608A1732
P 5550 1650
AR Path="/5515D395/608A1732" Ref="D?"  Part="1" 
AR Path="/60869CEA/608A1732" Ref="D27"  Part="1" 
AR Path="/608DFF6C/608A1732" Ref="D28"  Part="1" 
AR Path="/608FE26D/608A1732" Ref="D29"  Part="1" 
AR Path="/608FE277/608A1732" Ref="D30"  Part="1" 
F 0 "D29" H 5543 1867 50  0000 C CNN
F 1 "LED" H 5543 1776 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5550 1650 50  0001 C CNN
F 3 "~" H 5550 1650 50  0001 C CNN
F 4 "C434419" H 5550 1650 50  0001 C CNN "LCSC"
	1    5550 1650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R28
U 1 1 608A77D0
P 5550 1400
AR Path="/60869CEA/608A77D0" Ref="R28"  Part="1" 
AR Path="/608DFF6C/608A77D0" Ref="R31"  Part="1" 
AR Path="/608FE26D/608A77D0" Ref="R34"  Part="1" 
AR Path="/608FE277/608A77D0" Ref="R37"  Part="1" 
F 0 "R34" H 5618 1446 50  0000 L CNN
F 1 "100" H 5618 1355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 5550 1400 50  0001 C CNN
F 3 "~" H 5550 1400 50  0001 C CNN
F 4 "C105588" H 5550 1400 50  0001 C CNN "LCSC"
	1    5550 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1200 5550 1200
Connection ~ 5250 1200
Wire Wire Line
	5550 1200 5550 1300
Wire Wire Line
	5250 1850 5550 1850
Wire Wire Line
	5550 1800 5550 1850
Connection ~ 5550 1850
Wire Wire Line
	4650 2250 4650 2300
$Comp
L Device:R_Small_US R26
U 1 1 60888114
P 4650 2050
AR Path="/60869CEA/60888114" Ref="R26"  Part="1" 
AR Path="/608DFF6C/60888114" Ref="R29"  Part="1" 
AR Path="/608FE26D/60888114" Ref="R32"  Part="1" 
AR Path="/608FE277/60888114" Ref="R35"  Part="1" 
F 0 "R32" H 4718 2096 50  0000 L CNN
F 1 "100" H 4718 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 4650 2050 50  0001 C CNN
F 3 "~" H 4650 2050 50  0001 C CNN
F 4 "C105588" H 4650 2050 50  0001 C CNN "LCSC"
	1    4650 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2150 4650 2250
Connection ~ 4650 2250
Wire Wire Line
	4650 1200 4650 1650
$Comp
L Device:C_Small C8
U 1 1 60A5FD43
P 4500 2050
AR Path="/608FE277/60A5FD43" Ref="C8"  Part="1" 
AR Path="/60869CEA/60A5FD43" Ref="C5"  Part="1" 
AR Path="/608DFF6C/60A5FD43" Ref="C6"  Part="1" 
AR Path="/608FE26D/60A5FD43" Ref="C7"  Part="1" 
F 0 "C7" H 4800 2000 50  0000 R CNN
F 1 "100nF" H 4800 2100 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 4500 2050 50  0001 C CNN
F 3 "~" H 4500 2050 50  0001 C CNN
F 4 "C83054" H 4500 2050 50  0001 C CNN "LCSC"
	1    4500 2050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 1850 4500 1950
Connection ~ 4500 1850
Wire Wire Line
	4500 1850 4400 1850
Wire Wire Line
	4500 2150 4500 2250
Wire Wire Line
	4500 2250 4650 2250
Connection ~ 4650 1850
Wire Wire Line
	6050 2650 6050 3000
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 60F78CCC
P 6350 3500
AR Path="/60869CEA/60F78CCC" Ref="J1"  Part="1" 
AR Path="/608DFF6C/60F78CCC" Ref="J5"  Part="1" 
AR Path="/608FE26D/60F78CCC" Ref="J7"  Part="1" 
AR Path="/608FE277/60F78CCC" Ref="J9"  Part="1" 
F 0 "J7" H 6322 3474 50  0000 R CNN
F 1 "Conn_01x04_Male" H 6322 3383 50  0000 R CNN
F 2 "sl_connector:1719740104" H 6350 3500 50  0001 C CNN
F 3 "~" H 6350 3500 50  0001 C CNN
F 4 "WM22669-ND" H 6350 3500 50  0001 C CNN "Digi-Key_PN"
	1    6350 3500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5750 3600 6150 3600
Wire Wire Line
	6150 3700 5750 3700
Wire Wire Line
	5750 3500 5900 3500
Wire Wire Line
	5900 3500 5900 3400
Wire Wire Line
	5900 3400 6150 3400
Wire Wire Line
	5750 3400 5800 3400
Wire Wire Line
	5800 3400 5800 3350
Wire Wire Line
	5800 3350 6050 3350
Wire Wire Line
	6050 3350 6050 3500
Wire Wire Line
	6050 3500 6150 3500
$Comp
L Connector:Conn_01x03_Male J4
U 1 1 60F911B2
P 6250 1750
AR Path="/608DFF6C/60F911B2" Ref="J4"  Part="1" 
AR Path="/60869CEA/60F911B2" Ref="J3"  Part="1" 
AR Path="/608FE26D/60F911B2" Ref="J6"  Part="1" 
AR Path="/608FE277/60F911B2" Ref="J8"  Part="1" 
F 0 "J6" H 6222 1774 50  0000 R CNN
F 1 "Conn_01x03_Male" H 6222 1683 50  0000 R CNN
F 2 "sl_connector:1719740103" H 6250 1750 50  0001 C CNN
F 3 "~" H 6250 1750 50  0001 C CNN
F 4 "WM22668-ND" H 6250 1750 50  0001 C CNN "Digi-Key_PN"
	1    6250 1750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5550 1850 6050 1850
Wire Wire Line
	6050 1750 5900 1750
Wire Wire Line
	5900 1750 5900 2250
Wire Wire Line
	5900 2250 4650 2250
Wire Wire Line
	6050 1650 5900 1650
Wire Wire Line
	5900 1650 5900 1200
Wire Wire Line
	5900 1200 5550 1200
Connection ~ 5550 1200
$Comp
L Device:R_Small_US R1
U 1 1 610652DE
P 4250 3800
AR Path="/60869CEA/610652DE" Ref="R1"  Part="1" 
AR Path="/608DFF6C/610652DE" Ref="R2"  Part="1" 
AR Path="/608FE26D/610652DE" Ref="R3"  Part="1" 
AR Path="/608FE277/610652DE" Ref="R4"  Part="1" 
F 0 "R3" H 4318 3846 50  0000 L CNN
F 1 "100" H 4318 3755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 4250 3800 50  0001 C CNN
F 3 "~" H 4250 3800 50  0001 C CNN
	1    4250 3800
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
