EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 8
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RPi_Hat:RPi_GPIO J2
U 1 1 5516AE26
P 3550 2150
AR Path="/5516AE26" Ref="J2"  Part="1" 
AR Path="/5515D395/5516AE26" Ref="J2"  Part="1" 
F 0 "J2" H 4300 2400 60  0000 C CNN
F 1 "RPi_GPIO" H 4300 2300 60  0000 C CNN
F 2 "RPi_Hat:Pin_Header_Straight_2x20" H 3550 2150 60  0001 C CNN
F 3 "" H 3550 2150 60  0000 C CNN
	1    3550 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 604AF549
P 3200 4150
F 0 "#PWR0102" H 3200 3900 50  0001 C CNN
F 1 "GND" H 3205 3977 50  0000 C CNN
F 2 "" H 3200 4150 50  0001 C CNN
F 3 "" H 3200 4150 50  0001 C CNN
	1    3200 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 604B0835
P 5400 4150
F 0 "#PWR0103" H 5400 3900 50  0001 C CNN
F 1 "GND" H 5405 3977 50  0000 C CNN
F 2 "" H 5400 4150 50  0001 C CNN
F 3 "" H 5400 4150 50  0001 C CNN
	1    5400 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2350 5400 2350
Wire Wire Line
	5400 2350 5400 2750
Wire Wire Line
	5250 2750 5400 2750
Connection ~ 5400 2750
Wire Wire Line
	5400 2750 5400 3050
Wire Wire Line
	5250 3050 5400 3050
Connection ~ 5400 3050
Wire Wire Line
	5400 3050 5400 3550
Wire Wire Line
	5250 3550 5400 3550
Connection ~ 5400 3550
Wire Wire Line
	5400 3550 5400 3750
Wire Wire Line
	5250 3750 5400 3750
Connection ~ 5400 3750
Wire Wire Line
	5400 3750 5400 4150
Wire Wire Line
	3200 4150 3200 4050
Wire Wire Line
	3200 2550 3350 2550
Wire Wire Line
	3350 3350 3200 3350
Connection ~ 3200 3350
Wire Wire Line
	3200 3350 3200 2550
Wire Wire Line
	3350 4050 3200 4050
Connection ~ 3200 4050
Wire Wire Line
	3200 4050 3200 3350
$Comp
L Device:D_Schottky D26
U 1 1 605A89CC
P 9500 2350
F 0 "D26" H 9500 2567 50  0000 C CNN
F 1 "D_Schottky" H 9500 2476 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-128" H 9500 2350 50  0001 C CNN
F 3 "~" H 9500 2350 50  0001 C CNN
	1    9500 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 605AA90B
P 9650 2350
F 0 "#PWR0106" H 9650 2200 50  0001 C CNN
F 1 "+5V" V 9650 2450 50  0000 L CNN
F 2 "" H 9650 2350 50  0001 C CNN
F 3 "" H 9650 2350 50  0001 C CNN
	1    9650 2350
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 605AC150
P 5250 2050
F 0 "#PWR0107" H 5250 1900 50  0001 C CNN
F 1 "+5V" H 5265 2223 50  0000 C CNN
F 2 "" H 5250 2050 50  0001 C CNN
F 3 "" H 5250 2050 50  0001 C CNN
	1    5250 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2250 5250 2150
Wire Wire Line
	5250 2150 5250 2050
Connection ~ 5250 2150
$Comp
L power:+3V3 #PWR0115
U 1 1 608B771D
P 9500 2550
F 0 "#PWR0115" H 9500 2400 50  0001 C CNN
F 1 "+3V3" V 9500 2750 50  0000 C CNN
F 2 "" H 9500 2550 50  0001 C CNN
F 3 "" H 9500 2550 50  0001 C CNN
	1    9500 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	9250 2550 9500 2550
Wire Wire Line
	9350 2350 9250 2350
$Comp
L power:GND #PWR0101
U 1 1 6096D3DF
P 6550 2350
F 0 "#PWR0101" H 6550 2100 50  0001 C CNN
F 1 "GND" V 6550 2150 50  0000 C CNN
F 2 "" H 6550 2350 50  0001 C CNN
F 3 "" H 6550 2350 50  0001 C CNN
	1    6550 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 2350 6550 2350
Text Label 5750 2450 0    50   ~ 0
PI_TX
Text Label 5750 2550 0    50   ~ 0
PI_RX
$Comp
L teensy:Teensy4.1 U10
U 1 1 60952E67
P 8150 4500
F 0 "U10" H 8150 7065 50  0000 C CNN
F 1 "Teensy4.1" H 8150 6974 50  0000 C CNN
F 2 "teensy:Teensy41" H 7750 4900 50  0001 C CNN
F 3 "" H 7750 4900 50  0001 C CNN
	1    8150 4500
	1    0    0    -1  
$EndComp
Text HLabel 6700 2650 0    50   Output ~ 0
X_STEP
Text HLabel 6700 2750 0    50   Output ~ 0
X_DIR
Text HLabel 6700 2850 0    50   Output ~ 0
Y_STEP
Text HLabel 6700 2950 0    50   Output ~ 0
Y_DIR
Text HLabel 6700 3050 0    50   Output ~ 0
Z_STEP
Text HLabel 6700 3150 0    50   Output ~ 0
Z_DIR
Text HLabel 6700 3250 0    50   Output ~ 0
A_STEP
Text HLabel 6700 3350 0    50   Output ~ 0
A_DIR
Text HLabel 6700 3450 0    50   Output ~ 0
X_EN
Wire Wire Line
	7050 2650 6700 2650
Wire Wire Line
	6700 2750 7050 2750
Wire Wire Line
	7050 2850 6700 2850
Wire Wire Line
	6700 2950 7050 2950
Wire Wire Line
	7050 3050 6700 3050
Wire Wire Line
	6700 3150 7050 3150
Wire Wire Line
	7050 3250 6700 3250
Wire Wire Line
	6700 3350 7050 3350
Wire Wire Line
	7050 3450 6700 3450
Wire Wire Line
	6500 3550 6850 3550
Wire Wire Line
	7050 3650 6700 3650
Wire Wire Line
	9450 3650 9600 3650
Text HLabel 9400 3550 2    50   Input ~ 0
CNC_RESET
Text HLabel 9400 3350 2    50   Input ~ 0
FEED_HOLD
Text HLabel 9400 3250 2    50   Input ~ 0
CYCLE_START
Wire Wire Line
	9400 3250 9250 3250
Wire Wire Line
	9400 3350 9250 3350
Wire Wire Line
	9250 3550 9400 3550
Text HLabel 9400 2950 2    50   Input ~ 0
X_LIMIT
Text HLabel 9400 3950 2    50   Output ~ 0
Y_ENABLE
Wire Wire Line
	9400 3950 9250 3950
Wire Wire Line
	9400 2950 9250 2950
Text HLabel 9400 2850 2    50   Input ~ 0
Y_LIMIT
Text HLabel 9400 2750 2    50   Input ~ 0
Z_LIMIT
Text HLabel 9400 2650 2    50   Input ~ 0
A_ENCA
Wire Wire Line
	9400 2650 9250 2650
Wire Wire Line
	9250 2750 9400 2750
Wire Wire Line
	9400 2850 9250 2850
Text HLabel 9400 4050 2    50   Output ~ 0
Z_ENABLE
Text HLabel 9400 4150 2    50   Output ~ 0
A_ENABLE
Text HLabel 9400 3850 2    50   Output ~ 0
B_ENABLE
Wire Wire Line
	9400 4050 9250 4050
Wire Wire Line
	9250 4150 9400 4150
Wire Wire Line
	9400 3850 9250 3850
Text HLabel 6700 4250 0    50   Input ~ 0
B_ENCA
Wire Wire Line
	6700 4250 7050 4250
Text HLabel 6700 4050 0    50   Output ~ 0
B_STEP
Text HLabel 6700 4150 0    50   Output ~ 0
B_DIR
Wire Wire Line
	7050 4050 6700 4050
Wire Wire Line
	6700 4150 7050 4150
Text HLabel 6700 4350 0    50   Input ~ 0
SAFETY_DOOR
Text HLabel 9400 3450 2    50   Input ~ 0
PROBE
Wire Wire Line
	9250 3450 9400 3450
Wire Wire Line
	7050 4350 6700 4350
Text HLabel 9400 3050 2    50   Input ~ 0
A_ENCB
Text HLabel 9400 3150 2    50   Output ~ 0
COOLANT_MIST
Wire Wire Line
	9250 3050 9400 3050
Wire Wire Line
	9400 3150 9250 3150
Text HLabel 6700 3850 0    50   Input ~ 0
SCL
Text HLabel 6700 3950 0    50   BiDi ~ 0
SDA
Wire Wire Line
	6700 3850 7050 3850
Wire Wire Line
	7050 3950 6700 3950
Wire Wire Line
	5250 2450 7050 2450
Wire Wire Line
	5250 2550 7050 2550
Text HLabel 9900 4550 2    50   Output ~ 0
CS0
Text HLabel 9700 4650 2    50   Output ~ 0
CS1
Wire Wire Line
	9650 4550 9900 4550
Wire Wire Line
	3350 3250 3050 3250
Wire Wire Line
	3350 3150 3050 3150
Wire Wire Line
	3350 3050 3050 3050
Text HLabel 9950 4450 2    50   Output ~ 0
CS2
Text HLabel 9950 4350 2    50   Output ~ 0
CS3
Wire Wire Line
	9650 4450 9950 4450
Wire Wire Line
	9950 4350 9650 4350
Text Label 5550 2950 0    50   ~ 0
SWDIO
Text Label 5550 3150 0    50   ~ 0
SCLK
Wire Wire Line
	5250 2950 5550 2950
Wire Wire Line
	5250 3150 5550 3150
Text HLabel 6500 3550 0    50   Input ~ 0
SPI_MOSI
Text HLabel 6700 3650 0    50   Input ~ 0
SPI_MISO
Text HLabel 9600 3650 2    50   Output ~ 0
SPI_CLK
$Comp
L Device:R_Small_US R?
U 1 1 6106923E
P 9350 4650
AR Path="/60869CEA/6106923E" Ref="R?"  Part="1" 
AR Path="/608DFF6C/6106923E" Ref="R?"  Part="1" 
AR Path="/608FE26D/6106923E" Ref="R?"  Part="1" 
AR Path="/608FE277/6106923E" Ref="R?"  Part="1" 
AR Path="/5515D395/6106923E" Ref="R6"  Part="1" 
F 0 "R6" H 9418 4696 50  0000 L CNN
F 1 "100" H 9418 4605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 9350 4650 50  0001 C CNN
F 3 "~" H 9350 4650 50  0001 C CNN
	1    9350 4650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 6106AEB8
P 6950 3550
AR Path="/60869CEA/6106AEB8" Ref="R?"  Part="1" 
AR Path="/608DFF6C/6106AEB8" Ref="R?"  Part="1" 
AR Path="/608FE26D/6106AEB8" Ref="R?"  Part="1" 
AR Path="/608FE277/6106AEB8" Ref="R?"  Part="1" 
AR Path="/5515D395/6106AEB8" Ref="R5"  Part="1" 
F 0 "R5" H 7018 3596 50  0000 L CNN
F 1 "100" H 7018 3505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 6950 3550 50  0001 C CNN
F 3 "~" H 6950 3550 50  0001 C CNN
	1    6950 3550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small_US R?
U 1 1 610704EE
P 9350 3650
AR Path="/60869CEA/610704EE" Ref="R?"  Part="1" 
AR Path="/608DFF6C/610704EE" Ref="R?"  Part="1" 
AR Path="/608FE26D/610704EE" Ref="R?"  Part="1" 
AR Path="/608FE277/610704EE" Ref="R?"  Part="1" 
AR Path="/5515D395/610704EE" Ref="R7"  Part="1" 
F 0 "R7" H 9418 3696 50  0000 L CNN
F 1 "100" H 9418 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 9350 3650 50  0001 C CNN
F 3 "~" H 9350 3650 50  0001 C CNN
	1    9350 3650
	0    -1   -1   0   
$EndComp
Text HLabel 9950 4250 2    50   Output ~ 0
CS4
Wire Wire Line
	9650 4250 9950 4250
Text HLabel 6700 4450 0    50   Input ~ 0
B_ENCB
Wire Wire Line
	6700 4450 7050 4450
$Comp
L Device:R_Pack04 RN3
U 1 1 613D08E2
P 9450 4350
F 0 "RN3" V 9775 4350 50  0000 C CNN
F 1 "R_100_4" V 9684 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_Array_Concave_4x0603" V 9725 4350 50  0001 C CNN
F 3 "~" H 9450 4350 50  0001 C CNN
	1    9450 4350
	0    1    -1   0   
$EndComp
Wire Wire Line
	9450 4650 9700 4650
$EndSCHEMATC
