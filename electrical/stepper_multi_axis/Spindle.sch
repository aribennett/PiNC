EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 8
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
L power:VBUS #PWR0143
U 1 1 60A890E2
P 4200 2650
F 0 "#PWR0143" H 4200 2500 50  0001 C CNN
F 1 "VBUS" H 4215 2823 50  0000 C CNN
F 2 "" H 4200 2650 50  0001 C CNN
F 3 "" H 4200 2650 50  0001 C CNN
	1    4200 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:D D2
U 1 1 60B0B3F3
P 3900 2950
F 0 "D2" V 3854 3030 50  0000 L CNN
F 1 "D" V 3945 3030 50  0000 L CNN
F 2 "Diode_THT:D_DO-201AD_P15.24mm_Horizontal" H 3900 2950 50  0001 C CNN
F 3 "~" H 3900 2950 50  0001 C CNN
	1    3900 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 2800 4200 2650
$Comp
L Connector_Generic:Conn_01x02 J11
U 1 1 60A88330
P 4400 2800
F 0 "J11" H 4480 2792 50  0000 L CNN
F 1 "Conn_01x02" H 4480 2701 50  0000 L CNN
F 2 "Connector_Molex:Molex_Mini-Fit_Jr_5569-02A2_2x01_P4.20mm_Horizontal" H 4400 2800 50  0001 C CNN
F 3 "~" H 4400 2800 50  0001 C CNN
	1    4400 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 2800 4200 2800
Connection ~ 4200 2800
Wire Wire Line
	4200 2900 4200 3100
Wire Wire Line
	4200 3100 3900 3100
$Comp
L Transistor_FET:IRF540N Q1
U 1 1 60B0C942
P 4100 3300
F 0 "Q1" H 4304 3346 50  0000 L CNN
F 1 "IRF540N" H 4304 3255 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 4350 3225 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 4100 3300 50  0001 L CNN
	1    4100 3300
	1    0    0    -1  
$EndComp
Connection ~ 4200 3100
$Comp
L power:GND #PWR0152
U 1 1 60B0EB06
P 4200 3500
F 0 "#PWR0152" H 4200 3250 50  0001 C CNN
F 1 "GND" H 4205 3327 50  0000 C CNN
F 2 "" H 4200 3500 50  0001 C CNN
F 3 "" H 4200 3500 50  0001 C CNN
	1    4200 3500
	1    0    0    -1  
$EndComp
$Comp
L Driver_FET:MCP1416 U12
U 1 1 60B1848B
P 2900 3300
F 0 "U12" H 3344 3346 50  0000 L CNN
F 1 "MCP1416" H 3344 3255 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 2900 2900 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20002092F.pdf" H 2700 3550 50  0001 C CNN
	1    2900 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3300 3900 3300
$Comp
L power:GND #PWR0153
U 1 1 60B195A9
P 2900 3600
F 0 "#PWR0153" H 2900 3350 50  0001 C CNN
F 1 "GND" H 2905 3427 50  0000 C CNN
F 2 "" H 2900 3600 50  0001 C CNN
F 3 "" H 2900 3600 50  0001 C CNN
	1    2900 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0154
U 1 1 60B198B4
P 2900 2900
F 0 "#PWR0154" H 2900 2750 50  0001 C CNN
F 1 "+5V" H 2915 3073 50  0000 C CNN
F 2 "" H 2900 2900 50  0001 C CNN
F 3 "" H 2900 2900 50  0001 C CNN
	1    2900 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3000 2900 2900
Text HLabel 2300 3300 0    50   Input ~ 0
SPINDLE_ON
Wire Wire Line
	2300 3300 2400 3300
$Comp
L Device:R_Small_US R?
U 1 1 60B1C5B3
P 2400 3400
AR Path="/60869CEA/60B1C5B3" Ref="R?"  Part="1" 
AR Path="/608DFF6C/60B1C5B3" Ref="R?"  Part="1" 
AR Path="/608FE26D/60B1C5B3" Ref="R?"  Part="1" 
AR Path="/608FE277/60B1C5B3" Ref="R?"  Part="1" 
AR Path="/60A8806F/60B1C5B3" Ref="R5"  Part="1" 
F 0 "R5" H 2468 3446 50  0000 L CNN
F 1 "10k" H 2468 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 2400 3400 50  0001 C CNN
F 3 "~" H 2400 3400 50  0001 C CNN
	1    2400 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 60B1C5B9
P 3350 2800
AR Path="/608FE277/60B1C5B9" Ref="C?"  Part="1" 
AR Path="/60869CEA/60B1C5B9" Ref="C?"  Part="1" 
AR Path="/608DFF6C/60B1C5B9" Ref="C?"  Part="1" 
AR Path="/608FE26D/60B1C5B9" Ref="C?"  Part="1" 
AR Path="/60A8806F/60B1C5B9" Ref="C12"  Part="1" 
F 0 "C12" H 3650 2750 50  0000 R CNN
F 1 "100nF" H 3650 2850 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3350 2800 50  0001 C CNN
F 3 "~" H 3350 2800 50  0001 C CNN
	1    3350 2800
	-1   0    0    1   
$EndComp
Connection ~ 2400 3300
Wire Wire Line
	2400 3300 2600 3300
$Comp
L power:GND #PWR0155
U 1 1 60B1D1B2
P 2400 3600
F 0 "#PWR0155" H 2400 3350 50  0001 C CNN
F 1 "GND" H 2405 3427 50  0000 C CNN
F 2 "" H 2400 3600 50  0001 C CNN
F 3 "" H 2400 3600 50  0001 C CNN
	1    2400 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3500 2400 3600
$Comp
L power:+5V #PWR0156
U 1 1 60B1E7CB
P 3350 2700
F 0 "#PWR0156" H 3350 2550 50  0001 C CNN
F 1 "+5V" H 3365 2873 50  0000 C CNN
F 2 "" H 3350 2700 50  0001 C CNN
F 3 "" H 3350 2700 50  0001 C CNN
	1    3350 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0157
U 1 1 60B1EF34
P 3350 2900
F 0 "#PWR0157" H 3350 2650 50  0001 C CNN
F 1 "GND" H 3355 2727 50  0000 C CNN
F 2 "" H 3350 2900 50  0001 C CNN
F 3 "" H 3350 2900 50  0001 C CNN
	1    3350 2900
	1    0    0    -1  
$EndComp
Text Label 4200 3050 0    50   ~ 0
spindle_drive
$EndSCHEMATC
