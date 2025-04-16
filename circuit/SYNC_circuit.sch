EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Neuropixels SYNC"
Date "4/2025"
Rev ""
Comp "Johns Hopkins University"
Comment1 "Dale Roberts"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 74xx:74HCT00 U?
U 1 1 67FFF025
P 4150 4800
F 0 "U?" H 4150 5125 50  0001 C CNN
F 1 "74HCT00" H 4150 5033 50  0000 C CNN
F 2 "" H 4150 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74hct00" H 4150 4800 50  0001 C CNN
	1    4150 4800
	1    0    0    -1  
$EndComp
$Comp
L teensy:Teensy4.0 U?
U 1 1 6800063C
P 6050 3750
F 0 "U?" H 6050 5365 50  0001 C CNN
F 1 "Teensy4.0" H 6050 5273 50  0000 C CNN
F 2 "" H 5650 3950 50  0001 C CNN
F 3 "" H 5650 3950 50  0001 C CNN
	1    6050 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J?
U 1 1 68003085
P 3450 4700
F 0 "J?" H 3378 4938 50  0001 C CNN
F 1 "Conn_Coaxial" H 3378 4847 50  0001 C CNN
F 2 "" H 3450 4700 50  0001 C CNN
F 3 " ~" H 3450 4700 50  0001 C CNN
	1    3450 4700
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 68003D8D
P 3450 5000
F 0 "R?" H 3520 5046 50  0000 L CNN
F 1 "33" H 3520 4955 50  0000 L CNN
F 2 "" V 3380 5000 50  0001 C CNN
F 3 "~" H 3450 5000 50  0001 C CNN
	1    3450 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 68007BE4
P 3450 5200
F 0 "#PWR?" H 3450 4950 50  0001 C CNN
F 1 "GND" H 3455 5027 50  0000 C CNN
F 2 "" H 3450 5200 50  0001 C CNN
F 3 "" H 3450 5200 50  0001 C CNN
	1    3450 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5150 3450 5200
Wire Wire Line
	3650 4700 3750 4700
Wire Wire Line
	3850 4900 3750 4900
Wire Wire Line
	3750 4900 3750 4700
Connection ~ 3750 4700
Wire Wire Line
	3750 4700 3850 4700
Text Notes 3200 4600 0    50   ~ 0
11.7MHz Clock\nFrom Headstage
$Comp
L power:GND #PWR?
U 1 1 6800CEF9
P 4550 2550
F 0 "#PWR?" H 4550 2300 50  0001 C CNN
F 1 "GND" H 4555 2377 50  0000 C CNN
F 2 "" H 4550 2550 50  0001 C CNN
F 3 "" H 4550 2550 50  0001 C CNN
	1    4550 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6800CFD0
P 7450 3100
F 0 "#PWR?" H 7450 2850 50  0001 C CNN
F 1 "GND" H 7455 2927 50  0000 C CNN
F 2 "" H 7450 3100 50  0001 C CNN
F 3 "" H 7450 3100 50  0001 C CNN
	1    7450 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2400 4550 2400
Wire Wire Line
	4550 2400 4550 2550
Wire Wire Line
	7150 3000 7450 3000
Wire Wire Line
	7450 3000 7450 3100
Text GLabel 7550 2900 2    50   Input ~ 0
3V3
Wire Wire Line
	7550 2900 7150 2900
Text Notes 7250 2800 0    50   ~ 0
Use Teensy 3.3V and Gnd to\npower the 74HCT00 chip.
$Comp
L Connector:Conn_Coaxial J?
U 1 1 6800FA5B
P 3400 3800
F 0 "J?" H 3328 4038 50  0001 C CNN
F 1 "Conn_Coaxial" H 3328 3947 50  0001 C CNN
F 2 "" H 3400 3800 50  0001 C CNN
F 3 " ~" H 3400 3800 50  0001 C CNN
	1    3400 3800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4450 4800 4950 4800
$Comp
L power:GND #PWR?
U 1 1 68012E0A
P 3400 4000
F 0 "#PWR?" H 3400 3750 50  0001 C CNN
F 1 "GND" H 3405 3827 50  0000 C CNN
F 2 "" H 3400 4000 50  0001 C CNN
F 3 "" H 3400 4000 50  0001 C CNN
	1    3400 4000
	1    0    0    -1  
$EndComp
Text Notes 3100 3700 0    50   ~ 0
Stim Trigger Output
$Comp
L Connector:Conn_Coaxial J?
U 1 1 68013A0D
P 3400 3050
F 0 "J?" H 3328 3288 50  0001 C CNN
F 1 "Conn_Coaxial" H 3328 3197 50  0001 C CNN
F 2 "" H 3400 3050 50  0001 C CNN
F 3 " ~" H 3400 3050 50  0001 C CNN
	1    3400 3050
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 68013A13
P 3400 3250
F 0 "#PWR?" H 3400 3000 50  0001 C CNN
F 1 "GND" H 3405 3077 50  0000 C CNN
F 2 "" H 3400 3250 50  0001 C CNN
F 3 "" H 3400 3250 50  0001 C CNN
	1    3400 3250
	1    0    0    -1  
$EndComp
Text Notes 3100 2950 0    50   ~ 0
Stim Trigger Input
Wire Wire Line
	3600 3050 4100 3050
Wire Wire Line
	4100 3050 4800 3700
Wire Wire Line
	4800 3700 4950 3700
Wire Wire Line
	3600 3800 4100 3800
Wire Wire Line
	4100 3800 4750 4500
Wire Wire Line
	4750 4500 4950 4500
Text Notes 3850 5150 0    50   ~ 0
Buffer the small\n11.7MHz clock signal.
$EndSCHEMATC
