EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 10
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
L power:GND #PWR0105
U 1 1 60A71E2D
P 5750 3500
F 0 "#PWR0105" H 5750 3250 50  0001 C CNN
F 1 "GND" H 5755 3327 50  0000 C CNN
F 2 "" H 5750 3500 50  0001 C CNN
F 3 "" H 5750 3500 50  0001 C CNN
	1    5750 3500
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Switching:LM2576S-5 U1
U 1 1 60A7394B
P 6950 3100
F 0 "U1" H 6950 3467 50  0000 C CNN
F 1 "LM2576S-5" H 6950 3376 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 6950 2850 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2576.pdf" H 6950 3100 50  0001 C CNN
F 4 "LM2576S-5.0/NOPB-ND" H 6950 3100 50  0001 C CNN "Digi-Key_PN"
F 5 "C347413" H 6950 3100 50  0001 C CNN "LCSC"
	1    6950 3100
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0137
U 1 1 60A7798F
P 6200 2850
F 0 "#PWR0137" H 6200 2700 50  0001 C CNN
F 1 "VBUS" H 6215 3023 50  0000 C CNN
F 2 "" H 6200 2850 50  0001 C CNN
F 3 "" H 6200 2850 50  0001 C CNN
	1    6200 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3000 6200 2850
$Comp
L power:GND #PWR0138
U 1 1 60A787FB
P 6950 3500
F 0 "#PWR0138" H 6950 3250 50  0001 C CNN
F 1 "GND" H 6955 3327 50  0000 C CNN
F 2 "" H 6950 3500 50  0001 C CNN
F 3 "" H 6950 3500 50  0001 C CNN
	1    6950 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 3400 6950 3450
Wire Wire Line
	6450 3200 6450 3450
Wire Wire Line
	6450 3450 6950 3450
Connection ~ 6950 3450
Wire Wire Line
	6950 3450 6950 3500
Wire Wire Line
	6200 3000 6450 3000
$Comp
L Device:CP1 C9
U 1 1 60A79B6B
P 6200 3150
F 0 "C9" H 6315 3196 50  0000 L CNN
F 1 "100uF" H 6315 3105 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x12.5" H 6200 3150 50  0001 C CNN
F 3 "~" H 6200 3150 50  0001 C CNN
F 4 "10-EEE-FK1H101GLCT-ND" H 6200 3150 50  0001 C CNN "Digi-Key_PN"
	1    6200 3150
	1    0    0    -1  
$EndComp
Connection ~ 6200 3000
$Comp
L Device:CP1 C10
U 1 1 60A7B22C
P 8500 3350
F 0 "C10" H 8615 3396 50  0000 L CNN
F 1 "1000uF" H 8615 3305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x12.5" H 8500 3350 50  0001 C CNN
F 3 "~" H 8500 3350 50  0001 C CNN
F 4 "399-20518-1-ND" H 8500 3350 50  0001 C CNN "Digi-Key_PN"
	1    8500 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 60A7B8CE
P 6200 3500
F 0 "#PWR0139" H 6200 3250 50  0001 C CNN
F 1 "GND" H 6205 3327 50  0000 C CNN
F 2 "" H 6200 3500 50  0001 C CNN
F 3 "" H 6200 3500 50  0001 C CNN
	1    6200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3300 6200 3500
$Comp
L power:GND #PWR0141
U 1 1 60A7DA82
P 8500 3500
F 0 "#PWR0141" H 8500 3250 50  0001 C CNN
F 1 "GND" H 8505 3327 50  0000 C CNN
F 2 "" H 8500 3500 50  0001 C CNN
F 3 "" H 8500 3500 50  0001 C CNN
	1    8500 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3000 8500 3000
Wire Wire Line
	8500 3000 8500 3200
$Comp
L power:+5V #PWR0142
U 1 1 60A7EE74
P 8500 2800
F 0 "#PWR0142" H 8500 2650 50  0001 C CNN
F 1 "+5V" H 8515 2973 50  0000 C CNN
F 2 "" H 8500 2800 50  0001 C CNN
F 3 "" H 8500 2800 50  0001 C CNN
	1    8500 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3000 8500 2800
Connection ~ 8500 3000
$Comp
L pspice:INDUCTOR L1
U 1 1 60A8014B
P 8200 3200
F 0 "L1" H 8200 3300 50  0000 C CNN
F 1 "100uH" H 8200 3150 50  0000 C CNN
F 2 "Inductor_SMD:L_12x12mm_H8mm" H 8200 3200 50  0001 C CNN
F 3 "~" H 8200 3200 50  0001 C CNN
F 4 "308-1331-1-ND" H 8200 3200 50  0001 C CNN "Digi-Key_PN"
F 5 "C169378" H 8200 3200 50  0001 C CNN "LCSC"
	1    8200 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 3200 8500 3200
Connection ~ 8500 3200
Wire Wire Line
	5750 3200 5750 3500
$Comp
L Connector:Conn_01x02_Male J10
U 1 1 60FF98A5
P 5550 3100
F 0 "J10" H 5522 2982 50  0000 R CNN
F 1 "Conn_01x02_Male" H 5522 3073 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal" H 5550 3100 50  0001 C CNN
F 3 "~" H 5550 3100 50  0001 C CNN
	1    5550 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2750 5750 3100
$Comp
L power:VBUS #PWR0104
U 1 1 60A71589
P 5750 2750
F 0 "#PWR0104" H 5750 2600 50  0001 C CNN
F 1 "VBUS" H 5765 2923 50  0000 C CNN
F 2 "" H 5750 2750 50  0001 C CNN
F 3 "" H 5750 2750 50  0001 C CNN
	1    5750 2750
	-1   0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D?
U 1 1 612E2960
P 7650 3350
AR Path="/5515D395/612E2960" Ref="D?"  Part="1" 
AR Path="/60A6FF98/612E2960" Ref="D1"  Part="1" 
F 0 "D1" H 7650 3567 50  0000 C CNN
F 1 "D_Schottky" H 7650 3476 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-128" H 7650 3350 50  0001 C CNN
F 3 "~" H 7650 3350 50  0001 C CNN
F 4 "1727-7810-1-ND" H 7650 3350 50  0001 C CNN "Digi-Key_PN"
F 5 "C456127" H 7650 3350 50  0001 C CNN "LCSC"
	1    7650 3350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 60A7D7A4
P 7650 3500
F 0 "#PWR0140" H 7650 3250 50  0001 C CNN
F 1 "GND" H 7655 3327 50  0000 C CNN
F 2 "" H 7650 3500 50  0001 C CNN
F 3 "" H 7650 3500 50  0001 C CNN
	1    7650 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3200 7650 3200
Connection ~ 7650 3200
Wire Wire Line
	7650 3200 7950 3200
$EndSCHEMATC
