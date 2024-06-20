EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:dc_dc_converter
LIBS:lipo_charger_module
LIBS:18650_lipo_battery
LIBS:DC_Calibrator_Battery_Module-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Lipo_charger_Module M1
U 1 1 5A52723D
P 3200 4800
F 0 "M1" H 3150 5400 60  0000 C CNN
F 1 "Lipo_charger_Module" H 3200 4200 60  0000 C CNN
F 2 "" H 3200 4800 60  0001 C CNN
F 3 "" H 3200 4800 60  0001 C CNN
	1    3200 4800
	1    0    0    -1  
$EndComp
$Comp
L 18650_LiPo_Battery B1
U 1 1 5A52736A
P 5300 6000
F 0 "B1" H 5305 6275 60  0000 C CNN
F 1 "18650_LiPo_Battery" H 5325 5710 60  0000 C CNN
F 2 "" H 5300 6000 60  0001 C CNN
F 3 "" H 5300 6000 60  0001 C CNN
	1    5300 6000
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03_Female J2
U 1 1 5A52742D
P 3900 1100
F 0 "J2" H 3900 1300 50  0000 C CNN
F 1 "To J2 on main PCB" H 3900 900 50  0000 C CNB
F 2 "" H 3900 1100 50  0001 C CNN
F 3 "" H 3900 1100 50  0001 C CNN
	1    3900 1100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4100 4500 7800 4500
Wire Wire Line
	4100 4650 4300 4650
Wire Wire Line
	4300 4650 4300 6000
Wire Wire Line
	4100 4950 6250 4950
Wire Wire Line
	6250 4950 6250 6000
Wire Wire Line
	4100 1100 10100 1100
Wire Wire Line
	4600 5100 4100 5100
Wire Wire Line
	4100 1000 9750 1000
$Comp
L Conn_01x02_Female J1
U 1 1 5A5276CC
P 1100 4900
F 0 "J1" H 1100 5000 50  0000 C CNN
F 1 "Optional Charging Input" H 1000 4550 50  0000 C CNB
F 2 "" H 1100 4900 50  0001 C CNN
F 3 "" H 1100 4900 50  0001 C CNN
	1    1100 4900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1300 4900 1300 4800
Wire Wire Line
	1300 4800 2300 4800
Wire Wire Line
	1300 5000 1300 5100
Wire Wire Line
	1300 5100 2300 5100
Text Notes 8100 6950 0    98   ~ 20
Battery Power Module\nfor DC Voltage Calibrator
Text Notes 10600 7650 0    60   ~ 12
Ver. 1.0
Wire Wire Line
	4100 1200 4600 1200
Text Notes 2000 2250 0    60   ~ 12
+5V
Wire Wire Line
	2100 5700 4100 5700
Wire Wire Line
	4100 5700 4100 5100
Connection ~ 4100 5100
$Comp
L IRF9540N Q1
U 1 1 5A54DB15
P 6500 2400
F 0 "Q1" V 6400 2600 50  0000 L CNN
F 1 "IRF9630" V 6800 2250 50  0000 L CNB
F 2 "TO_SOT_Packages_THT:TO-220_Vertical" H 6750 2325 50  0001 L CIN
F 3 "" H 6500 2400 50  0001 L CNN
	1    6500 2400
	0    -1   -1   0   
$EndComp
$Comp
L 2N3904 Q2
U 1 1 5A54DB78
P 6600 3000
F 0 "Q2" H 6800 3075 50  0000 L CNN
F 1 "2N3904" H 6800 3000 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 6800 2925 50  0001 L CIN
F 3 "" H 6600 3000 50  0001 L CNN
	1    6600 3000
	-1   0    0    -1  
$EndComp
$Comp
L 2N3904 Q3
U 1 1 5A54DB9D
P 7300 3200
F 0 "Q3" H 7500 3275 50  0000 L CNN
F 1 "2N3904" H 7500 3200 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 7500 3125 50  0001 L CIN
F 3 "" H 7300 3200 50  0001 L CNN
	1    7300 3200
	-1   0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A54DBD6
P 6900 2550
F 0 "R1" V 6980 2550 50  0000 C CNN
F 1 "100K" V 6900 2550 50  0000 C CNN
F 2 "" V 6830 2550 50  0001 C CNN
F 3 "" H 6900 2550 50  0001 C CNN
	1    6900 2550
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A54DC23
P 7200 2550
F 0 "R2" V 7280 2550 50  0000 C CNN
F 1 "100K" V 7200 2550 50  0000 C CNN
F 2 "" V 7130 2550 50  0001 C CNN
F 3 "" H 7200 2550 50  0001 C CNN
	1    7200 2550
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5A54DC54
P 7500 3800
F 0 "R3" V 7580 3800 50  0000 C CNN
F 1 "100K" V 7500 3800 50  0000 C CNN
F 2 "" V 7430 3800 50  0001 C CNN
F 3 "" H 7500 3800 50  0001 C CNN
	1    7500 3800
	-1   0    0    1   
$EndComp
$Comp
L DC_DC_Down_Converter M2
U 1 1 5A54DCE4
P 3200 2600
F 0 "M2" H 3150 3200 60  0000 C CNN
F 1 "DC_DC_Down_Converter" H 3200 2000 60  0000 C CNN
F 2 "" H 3200 2600 60  0001 C CNN
F 3 "" H 3200 2600 60  0001 C CNN
	1    3200 2600
	-1   0    0    -1  
$EndComp
$Comp
L DC_DC_Up_Converter M3
U 1 1 5A54DD99
P 8700 4800
F 0 "M3" H 8650 5400 60  0000 C CNN
F 1 "DC_DC_Up_Converter" H 8700 4200 60  0000 C CNN
F 2 "" H 8700 4800 60  0001 C CNN
F 3 "" H 8700 4800 60  0001 C CNN
	1    8700 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2900 2100 2900
Wire Wire Line
	2100 2900 2100 5700
Connection ~ 2100 5100
Wire Wire Line
	2300 2300 1800 2300
Connection ~ 1800 4800
Wire Wire Line
	7200 2300 7200 2400
Wire Wire Line
	6900 2400 6900 2300
Connection ~ 6900 2300
Wire Wire Line
	6500 2600 6500 2800
Wire Wire Line
	6900 2700 6500 2700
Connection ~ 6500 2700
Wire Wire Line
	7200 2700 7200 3000
Wire Wire Line
	7200 3000 6800 3000
Connection ~ 7200 3000
Wire Wire Line
	4100 2900 4600 2900
Wire Wire Line
	4600 1200 4600 5100
Connection ~ 4600 2900
Wire Wire Line
	6500 3400 6500 3200
Connection ~ 4600 3400
Wire Wire Line
	9750 1000 9750 4500
Wire Wire Line
	9750 4500 9600 4500
Connection ~ 9750 2300
Connection ~ 7200 2300
Wire Wire Line
	9600 5700 9600 5100
Wire Wire Line
	7200 5700 10100 5700
Wire Wire Line
	4100 2300 6300 2300
Wire Wire Line
	6700 2300 9750 2300
Wire Wire Line
	7200 3400 7200 5700
Wire Wire Line
	7800 5100 7200 5100
Connection ~ 7200 5100
Wire Wire Line
	7500 3650 7500 3200
Wire Wire Line
	7500 3950 7500 4500
Connection ~ 7500 4500
Wire Wire Line
	6500 3400 4600 3400
Wire Wire Line
	10100 5700 10100 1100
Connection ~ 9600 5700
Text Notes 8300 2250 0    60   ~ 12
+18V
Text Notes 4100 950  0    60   ~ 12
+18V
Text Notes 4650 1600 0    60   ~ 12
GND
Text Notes 5750 1250 0    60   ~ 12
Switched GND
Text Notes 850  4750 0    60   ~ 12
Micro USB
Text Notes 2800 2850 0    60   ~ 12
Set for 5V Ouput
Text Notes 8250 5050 0    60   ~ 12
Set for 18V Output
Text Notes 1050 3600 0    60   ~ 12
+5v Supply for\nCharger Module
Text Notes 3050 5850 0    60   ~ 12
GND
Text Notes 8400 5850 0    60   ~ 12
Switched GND
Text Notes 9000 7650 0    60   ~ 12
9th January 2018
Text Notes 8950 7300 0    60   ~ 12
Scullcom Hobby Electronics
Text Notes 7500 7500 0    60   ~ 12
Battery Power Module for DC Voltage Calibrator
Text Notes 5550 2750 0    60   ~ 12
Switching Circuit
Text Notes 5350 4450 0    60   ~ 12
+3.7V (nominal)
Text Notes 4500 5250 0    60   ~ 12
GND
$Comp
L Fuse F1
U 1 1 5A5512BC
P 1800 2750
F 0 "F1" H 1900 2750 50  0000 C CNN
F 1 "Fuse" H 1650 2800 50  0000 C CNN
F 2 "" V 1730 2750 50  0001 C CNN
F 3 "" H 1800 2750 50  0001 C CNN
	1    1800 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2300 1800 2600
Wire Wire Line
	1800 2900 1800 4800
Text Notes 1450 2850 0    60   ~ 12
1 Amp
$EndSCHEMATC
