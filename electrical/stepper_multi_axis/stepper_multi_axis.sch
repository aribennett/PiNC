EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 10
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 5500 2050 1250 1300
U 608DFF6C
F0 "sheet608DFF62" 50
F1 "Stepper_Driver_X.sch" 50
F2 "STEP" I L 5500 2150 50 
F3 "DIR" I L 5500 2250 50 
F4 "SDI" I L 5500 2500 50 
F5 "SCK" I L 5500 2600 50 
F6 "SDO" O L 5500 2700 50 
F7 "CS" I L 5500 2800 50 
F8 "EN" I L 5500 2350 50 
F9 "Limit" O L 5500 3050 50 
$EndSheet
Wire Wire Line
	5250 2800 5500 2800
Wire Wire Line
	5250 1300 5500 1300
Wire Wire Line
	4450 1000 4450 650 
Wire Wire Line
	4450 650  5500 650 
Wire Wire Line
	4550 1100 4550 750 
Wire Wire Line
	4550 750  5500 750 
Wire Wire Line
	5500 850  4650 850 
Wire Wire Line
	4650 850  4650 1200
Wire Wire Line
	4350 1300 4350 1550
Wire Wire Line
	4350 1550 5500 1550
Wire Wire Line
	4850 1650 4850 2150
Wire Wire Line
	4850 2150 5500 2150
Wire Wire Line
	4750 1750 4750 2250
Wire Wire Line
	4750 2250 5500 2250
Wire Wire Line
	4650 1850 4650 2350
Wire Wire Line
	4650 2350 5500 2350
Wire Wire Line
	4450 1950 4450 3050
Wire Wire Line
	4450 3050 5500 3050
$Sheet
S 7750 1050 1150 900 
U 60A6FF98
F0 "Power" 50
F1 "Power.sch" 50
$EndSheet
$Sheet
S 1350 850  1700 4600
U 5515D395
F0 "RPi_GPIO" 60
F1 "RPi_GPIO.sch" 60
F2 "X_STEP" O R 3050 1000 50 
F3 "X_DIR" O R 3050 1100 50 
F4 "Y_STEP" O R 3050 1650 50 
F5 "Y_DIR" O R 3050 1750 50 
F6 "Z_STEP" O R 3050 2250 50 
F7 "Z_DIR" O R 3050 2350 50 
F8 "A_STEP" O R 3050 2900 50 
F9 "A_DIR" O R 3050 3000 50 
F10 "X_EN" O R 3050 1200 50 
F11 "CNC_RESET" I L 1350 1300 50 
F12 "FEED_HOLD" I L 1350 1400 50 
F13 "CYCLE_START" I L 1350 1500 50 
F14 "X_LIMIT" I R 3050 1300 50 
F15 "Y_ENABLE" O R 3050 1850 50 
F16 "Y_LIMIT" I R 3050 1950 50 
F17 "Z_LIMIT" I R 3050 2550 50 
F18 "Z_ENABLE" O R 3050 2450 50 
F19 "A_ENABLE" O R 3050 3100 50 
F20 "B_ENABLE" O R 3050 3650 50 
F21 "B_STEP" O R 3050 3450 50 
F22 "B_DIR" O R 3050 3550 50 
F23 "SAFETY_DOOR" I L 1350 1600 50 
F24 "PROBE" I L 1350 1700 50 
F25 "COOLANT_MIST" O L 1350 1200 50 
F26 "SCL" I L 1350 5200 50 
F27 "SDA" B L 1350 5300 50 
F28 "CS0" O R 3050 5300 50 
F29 "CS1" O R 3050 5200 50 
F30 "SPI_CLK" O R 3050 4700 50 
F31 "SPI_MISO" I R 3050 4600 50 
F32 "SPI_MOSI" I R 3050 4500 50 
F33 "CS2" O R 3050 5100 50 
F34 "CS3" O R 3050 5000 50 
F35 "A_ENCA" I R 3050 3200 50 
F36 "B_ENCA" I R 3050 3750 50 
F37 "A_ENCB" I R 3050 3300 50 
F38 "CS4" O R 3050 4900 50 
F39 "B_ENCB" I R 3050 3850 50 
F40 "USB_TEENSY_P" B L 1350 5000 50 
F41 "USB_TEENSY_N" B L 1350 4900 50 
$EndSheet
$Comp
L Mechanical:MountingHole H1
U 1 1 60F7C2E9
P 9450 1150
F 0 "H1" H 9550 1196 50  0000 L CNN
F 1 "MountingHole" H 9550 1105 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 9450 1150 50  0001 C CNN
F 3 "~" H 9450 1150 50  0001 C CNN
	1    9450 1150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J11
U 1 1 610A5F1C
P 3200 4300
F 0 "J11" V 3262 4344 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3353 4344 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3200 4300 50  0001 C CNN
F 3 "~" H 3200 4300 50  0001 C CNN
	1    3200 4300
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male J14
U 1 1 610A70AF
P 3300 4400
F 0 "J14" V 3362 4444 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3453 4444 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3300 4400 50  0001 C CNN
F 3 "~" H 3300 4400 50  0001 C CNN
	1    3300 4400
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male J12
U 1 1 610BC5BD
P 3300 4800
F 0 "J12" V 3362 4844 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3453 4844 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3300 4800 50  0001 C CNN
F 3 "~" H 3300 4800 50  0001 C CNN
	1    3300 4800
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male J16
U 1 1 610C3299
P 3400 5000
F 0 "J16" V 3462 5044 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3553 5044 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3400 5000 50  0001 C CNN
F 3 "~" H 3400 5000 50  0001 C CNN
	1    3400 5000
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male J18
U 1 1 610C6847
P 3450 5100
F 0 "J18" V 3512 5144 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3603 5144 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3450 5100 50  0001 C CNN
F 3 "~" H 3450 5100 50  0001 C CNN
	1    3450 5100
	0    1    1    0   
$EndComp
Connection ~ 3450 5300
Wire Wire Line
	3450 5300 4150 5300
Connection ~ 3400 5200
Wire Wire Line
	3400 5200 4150 5200
Wire Wire Line
	4250 2350 4250 3750
Wire Wire Line
	4350 2250 4350 3650
Wire Wire Line
	4250 3750 5500 3750
Wire Wire Line
	4350 3650 5500 3650
Wire Wire Line
	4150 3850 5500 3850
Wire Wire Line
	4150 2450 4150 3850
Wire Wire Line
	3350 5100 4150 5100
$Comp
L Connector:Conn_01x01_Male J15
U 1 1 610BFCE4
P 3350 4900
F 0 "J15" V 3412 4944 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3503 4944 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3350 4900 50  0001 C CNN
F 3 "~" H 3350 4900 50  0001 C CNN
	1    3350 4900
	0    1    1    0   
$EndComp
Connection ~ 3350 5100
$Sheet
S 5500 3550 1250 1300
U 608FE26D
F0 "sheet608FE262" 50
F1 "Stepper_Driver_X.sch" 50
F2 "STEP" I L 5500 3650 50 
F3 "DIR" I L 5500 3750 50 
F4 "SDI" I L 5500 4000 50 
F5 "SCK" I L 5500 4100 50 
F6 "SDO" O L 5500 4200 50 
F7 "CS" I L 5500 4300 50 
F8 "EN" I L 5500 3850 50 
F9 "Limit" O L 5500 4550 50 
$EndSheet
Wire Wire Line
	3050 1000 4450 1000
Wire Wire Line
	3050 5000 3300 5000
Wire Wire Line
	3050 5100 3350 5100
Wire Wire Line
	3050 5200 3400 5200
Wire Wire Line
	3050 5300 3450 5300
Wire Wire Line
	3050 2450 4150 2450
Wire Wire Line
	3050 2350 4250 2350
Wire Wire Line
	3050 1950 4450 1950
Wire Wire Line
	3050 1850 4650 1850
Wire Wire Line
	3050 1750 4750 1750
Wire Wire Line
	3050 1650 4850 1650
Wire Wire Line
	3050 1300 4350 1300
Wire Wire Line
	3050 1200 4650 1200
Wire Wire Line
	3050 1100 4550 1100
$Sheet
S 5500 550  1250 1300
U 60869CEA
F0 "Stepper_Driver_X" 50
F1 "Stepper_Driver_X.sch" 50
F2 "STEP" I L 5500 650 50 
F3 "DIR" I L 5500 750 50 
F4 "SDI" I L 5500 1000 50 
F5 "SCK" I L 5500 1100 50 
F6 "SDO" O L 5500 1200 50 
F7 "CS" I L 5500 1300 50 
F8 "EN" I L 5500 850 50 
F9 "Limit" O L 5500 1550 50 
$EndSheet
$Sheet
S 7800 2200 1000 1300
U 61235537
F0 "Sheet61235536" 50
F1 "Servo_Driver.sch" 50
F2 "SDO" O L 7800 2850 50 
F3 "SDI" I L 7800 2650 50 
F4 "SCK" I L 7800 2750 50 
F5 "CSN" I L 7800 2950 50 
F6 "EN" I L 7800 2500 50 
F7 "STEP" I L 7800 2300 50 
F8 "DIR" I L 7800 2400 50 
F9 "ENC_A" O L 7800 3150 50 
F10 "ENC_B" O L 7800 3250 50 
$EndSheet
$Comp
L Connector:Conn_01x01_Male J17
U 1 1 610A8D8B
P 3450 4500
F 0 "J17" V 3512 4544 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3603 4544 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3450 4500 50  0001 C CNN
F 3 "~" H 3450 4500 50  0001 C CNN
	1    3450 4500
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x01_Male J8
U 1 1 612B1FA3
P 3250 4700
F 0 "J8" V 3312 4744 50  0000 L CNN
F 1 "Conn_01x01_Male" V 3403 4744 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3250 4700 50  0001 C CNN
F 3 "~" H 3250 4700 50  0001 C CNN
	1    3250 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 4900 3250 4900
Wire Wire Line
	3050 2250 4350 2250
Text Label 5250 1000 0    50   ~ 0
MOSI
Text Label 5250 1100 0    50   ~ 0
SCK
Text Label 5250 1200 0    50   ~ 0
MISO
Wire Wire Line
	5500 1200 5250 1200
Wire Wire Line
	5500 1100 5250 1100
Wire Wire Line
	5500 1000 5250 1000
Text Label 5250 2500 0    50   ~ 0
MOSI
Text Label 5250 2600 0    50   ~ 0
SCK
Text Label 5250 2700 0    50   ~ 0
MISO
Wire Wire Line
	5500 2700 5250 2700
Wire Wire Line
	5500 2600 5250 2600
Wire Wire Line
	5500 2500 5250 2500
Text Label 5250 4000 0    50   ~ 0
MOSI
Text Label 5250 4100 0    50   ~ 0
SCK
Text Label 5250 4200 0    50   ~ 0
MISO
Wire Wire Line
	5500 4200 5250 4200
Wire Wire Line
	5500 4100 5250 4100
Wire Wire Line
	5500 4000 5250 4000
Text Label 7550 2650 0    50   ~ 0
MOSI
Text Label 7550 2750 0    50   ~ 0
SCK
Text Label 7550 2850 0    50   ~ 0
MISO
Wire Wire Line
	7800 2850 7550 2850
Wire Wire Line
	7800 2750 7550 2750
Wire Wire Line
	7800 2650 7550 2650
Wire Wire Line
	3050 4500 3200 4500
Wire Wire Line
	3050 4600 3300 4600
Wire Wire Line
	3050 4700 3450 4700
Text Label 4350 4500 2    50   ~ 0
MOSI
Text Label 4350 4700 2    50   ~ 0
SCK
Text Label 4350 4600 2    50   ~ 0
MISO
Wire Wire Line
	3200 4500 4350 4500
Connection ~ 3200 4500
Wire Wire Line
	3300 4600 4350 4600
Connection ~ 3300 4600
Wire Wire Line
	3450 4700 4350 4700
Connection ~ 3450 4700
Text Label 7550 2300 0    50   ~ 0
ST_A
Text Label 7550 2400 0    50   ~ 0
D_A
Text Label 7550 2500 0    50   ~ 0
EN_A
Wire Wire Line
	7550 2300 7800 2300
Wire Wire Line
	7550 2400 7800 2400
Wire Wire Line
	7550 2500 7800 2500
Text Label 3150 2900 0    50   ~ 0
ST_A
Text Label 3150 3000 0    50   ~ 0
D_A
Text Label 3150 3100 0    50   ~ 0
EN_A
Text Label 3150 3200 0    50   ~ 0
ENCA_A
Text Label 3150 3300 0    50   ~ 0
ENCB_A
Wire Wire Line
	3050 3300 3150 3300
Wire Wire Line
	3050 3200 3150 3200
Wire Wire Line
	3050 3100 3150 3100
Wire Wire Line
	3050 3000 3150 3000
Wire Wire Line
	3050 2900 3150 2900
Text Label 4150 5000 0    50   ~ 0
CS_A
Wire Wire Line
	3300 5000 4150 5000
Connection ~ 3300 5000
Text Label 7550 2950 0    50   ~ 0
CS_A
Text Label 7500 3150 0    50   ~ 0
ENCA_A
Text Label 7500 3250 0    50   ~ 0
ENCB_A
Wire Wire Line
	7800 2950 7550 2950
Wire Wire Line
	7800 3150 7500 3150
Wire Wire Line
	7800 3250 7500 3250
Text Label 5200 4550 0    50   ~ 0
LIM_Z
Text Label 3300 2550 0    50   ~ 0
LIM_Z
Wire Wire Line
	3050 2550 3300 2550
Wire Wire Line
	5200 4550 5500 4550
Text Label 5200 4300 0    50   ~ 0
CS_Z
Wire Wire Line
	5200 4300 5500 4300
Text Label 4150 5100 0    50   ~ 0
CS_Z
Text Label 3150 3450 0    50   ~ 0
ST_B
Text Label 3150 3550 0    50   ~ 0
D_B
Text Label 3150 3650 0    50   ~ 0
EN_B
Text Label 3150 3750 0    50   ~ 0
ENCA_B
Text Label 3150 3850 0    50   ~ 0
ENCB_B
Wire Wire Line
	3050 3850 3150 3850
Wire Wire Line
	3050 3750 3150 3750
Wire Wire Line
	3050 3650 3150 3650
Wire Wire Line
	3050 3550 3150 3550
Wire Wire Line
	3050 3450 3150 3450
Text Label 4150 5200 0    50   ~ 0
CS_Y
Text Label 5250 2800 0    50   ~ 0
CS_Y
Text Label 4150 5300 0    50   ~ 0
CS_X
Text Label 4150 4900 0    50   ~ 0
CS_B
Wire Wire Line
	3250 4900 4150 4900
Connection ~ 3250 4900
$Sheet
S 7800 3700 1000 1300
U 613BBAC7
F0 "sheet613BBABC" 50
F1 "Servo_Driver.sch" 50
F2 "SDO" O L 7800 4350 50 
F3 "SDI" I L 7800 4150 50 
F4 "SCK" I L 7800 4250 50 
F5 "CSN" I L 7800 4450 50 
F6 "EN" I L 7800 4000 50 
F7 "STEP" I L 7800 3800 50 
F8 "DIR" I L 7800 3900 50 
F9 "ENC_A" O L 7800 4650 50 
F10 "ENC_B" O L 7800 4750 50 
$EndSheet
Text Label 7550 4150 0    50   ~ 0
MOSI
Text Label 7550 4250 0    50   ~ 0
SCK
Text Label 7550 4350 0    50   ~ 0
MISO
Wire Wire Line
	7800 4350 7550 4350
Wire Wire Line
	7800 4250 7550 4250
Wire Wire Line
	7800 4150 7550 4150
Text Label 7550 3800 0    50   ~ 0
ST_B
Text Label 7550 3900 0    50   ~ 0
D_B
Text Label 7550 4000 0    50   ~ 0
EN_B
Wire Wire Line
	7550 3800 7800 3800
Wire Wire Line
	7550 3900 7800 3900
Wire Wire Line
	7550 4000 7800 4000
Text Label 7550 4450 0    50   ~ 0
CS_B
Text Label 7500 4650 0    50   ~ 0
ENCA_B
Text Label 7500 4750 0    50   ~ 0
ENCB_B
Wire Wire Line
	7800 4450 7550 4450
Wire Wire Line
	7800 4650 7500 4650
Wire Wire Line
	7800 4750 7500 4750
Text Label 5250 1300 0    50   ~ 0
CS_X
$Sheet
S 750  6250 950  800 
U 613FBD86
F0 "Sheet613FBD85" 50
F1 "CM4.sch" 50
F2 "RX" I R 1700 6350 50 
F3 "TX" O R 1700 6450 50 
F4 "SWDIO" B R 1700 6550 50 
F5 "SWDCLK" O R 1700 6650 50 
F6 "USB_N" B R 1700 6750 50 
F7 "USB_P" B R 1700 6850 50 
F8 "USB_OTG" I R 1700 6950 50 
$EndSheet
$Sheet
S 2300 6250 950  800 
U 614CFDB7
F0 "Sheet614CFDB6" 50
F1 "USBHub.sch" 50
F2 "USB_N" B L 2300 6750 50 
F3 "USP_P" B L 2300 6850 50 
F4 "USB_OTG" O L 2300 6950 50 
F5 "USB_INTERNAL_P" B L 2300 6500 50 
F6 "USB_INTERNAL_N" B L 2300 6600 50 
$EndSheet
Wire Wire Line
	1700 6750 2300 6750
Wire Wire Line
	1700 6850 2300 6850
Wire Wire Line
	1700 6950 2300 6950
Wire Wire Line
	2300 6500 2100 6500
Wire Wire Line
	2100 6500 2100 5600
Wire Wire Line
	2100 5600 1200 5600
Wire Wire Line
	1200 5600 1200 5000
Wire Wire Line
	1200 5000 1350 5000
Wire Wire Line
	1350 4900 1100 4900
Wire Wire Line
	1100 4900 1100 5700
Wire Wire Line
	1100 5700 2000 5700
Wire Wire Line
	2000 5700 2000 6600
Wire Wire Line
	2000 6600 2300 6600
$EndSCHEMATC
