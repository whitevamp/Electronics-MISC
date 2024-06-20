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
LIBS:TFT_selectable_voltage_Ref-cache
LIBS:DC_Cal_TFT_Arduino_Module-cache
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
L TFT_240x320_SPI_ILI9341 U1
U 1 1 5A2DAB32
P 2400 2850
F 0 "U1" H 2360 3760 70  0000 L CNN
F 1 "TFT_240x320_SPI_ILI9341" H 1700 1910 70  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x14_Pitch2.54mm" H 2400 2850 60  0001 C CNN
F 3 "" H 2400 2850 60  0001 C CNN
	1    2400 2850
	1    0    0    -1  
$EndComp
$Comp
L arduino_nano U2
U 1 1 5A2DAB5B
P 8100 2250
F 0 "U2" H 8600 1300 70  0000 C CNN
F 1 "arduino_nano" H 8050 -300 70  0000 C CNN
F 2 "Arduino_Socket:DIP-30_0_ELL" H 8100 2200 60  0001 C CNN
F 3 "" H 8100 2250 60  0001 C CNN
	1    8100 2250
	-1   0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5A2DAC80
P 5850 2200
F 0 "R8" V 5800 2050 50  0000 C CNN
F 1 "10K" V 5850 2200 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 2200 50  0001 C CNN
F 3 "" H 5850 2200 50  0001 C CNN
	1    5850 2200
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 5A2DAD00
P 5850 2600
F 0 "R10" V 5950 2600 50  0000 C CNN
F 1 "10K" V 5850 2600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 2600 50  0001 C CNN
F 3 "" H 5850 2600 50  0001 C CNN
	1    5850 2600
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5A2DAD1F
P 5850 1600
F 0 "R5" V 5800 1450 50  0000 C CNN
F 1 "10K" V 5850 1600 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 1600 50  0001 C CNN
F 3 "" H 5850 1600 50  0001 C CNN
	1    5850 1600
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 5A2DAD50
P 5850 1800
F 0 "R6" V 5800 1650 50  0000 C CNN
F 1 "10K" V 5850 1800 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 1800 50  0001 C CNN
F 3 "" H 5850 1800 50  0001 C CNN
	1    5850 1800
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5A2DAD7B
P 5500 1900
F 0 "R2" V 5450 1750 50  0000 C CNN
F 1 "10K" V 5500 1900 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5430 1900 50  0001 C CNN
F 3 "" H 5500 1900 50  0001 C CNN
	1    5500 1900
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 5A2DADAE
P 5850 2000
F 0 "R7" V 5800 1850 50  0000 C CNN
F 1 "10K" V 5850 2000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 2000 50  0001 C CNN
F 3 "" H 5850 2000 50  0001 C CNN
	1    5850 2000
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 5A2DADF5
P 5500 2500
F 0 "R4" V 5450 2350 50  0000 C CNN
F 1 "10K" V 5500 2500 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5430 2500 50  0001 C CNN
F 3 "" H 5500 2500 50  0001 C CNN
	1    5500 2500
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 5A2DAE7F
P 5850 2400
F 0 "R9" V 5800 2250 50  0000 C CNN
F 1 "10K" V 5850 2400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5780 2400 50  0001 C CNN
F 3 "" H 5850 2400 50  0001 C CNN
	1    5850 2400
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 5A2DB432
P 5500 2100
F 0 "R3" V 5450 1950 50  0000 C CNN
F 1 "10K" V 5500 2100 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 5430 2100 50  0001 C CNN
F 3 "" H 5500 2100 50  0001 C CNN
	1    5500 2100
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 5A2DB4C5
P 4550 3750
F 0 "R1" H 4600 3600 50  0000 C CNN
F 1 "56" V 4550 3750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 4480 3750 50  0001 C CNN
F 3 "" H 4550 3750 50  0001 C CNN
	1    4550 3750
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 2200 5700 2200
Wire Wire Line
	4400 2400 5700 2400
Wire Wire Line
	4400 2500 5350 2500
Wire Wire Line
	4400 2600 5700 2600
Wire Wire Line
	4400 3100 4950 3100
Wire Wire Line
	4400 3200 5050 3200
Wire Wire Line
	4400 3300 4850 3300
Wire Wire Line
	6300 2650 7400 2650
Wire Wire Line
	6000 2200 7400 2200
Wire Wire Line
	6200 2750 7400 2750
Wire Wire Line
	4400 2900 4650 2900
Wire Wire Line
	4650 2900 4650 1600
Wire Wire Line
	4650 1600 5700 1600
Wire Wire Line
	6000 1600 7400 1600
Wire Wire Line
	4400 2700 4550 2700
Wire Wire Line
	4550 2700 4550 1700
Wire Wire Line
	4400 3000 4750 3000
Wire Wire Line
	4750 3000 4750 1800
Wire Wire Line
	4750 1800 5700 1800
Wire Wire Line
	6000 1800 7400 1800
Wire Wire Line
	5650 1900 7400 1900
Wire Wire Line
	4850 3300 4850 1900
Wire Wire Line
	4850 1900 5350 1900
Wire Wire Line
	4950 3100 4950 2000
Wire Wire Line
	4950 2000 5700 2000
Wire Wire Line
	6000 2000 7400 2000
Wire Wire Line
	5050 3200 5050 2100
Wire Wire Line
	5050 2100 5350 2100
Wire Wire Line
	6000 2400 6400 2400
Wire Wire Line
	6400 2400 6400 2550
Wire Wire Line
	6400 2550 7400 2550
Wire Wire Line
	5650 2500 6300 2500
Wire Wire Line
	6300 2500 6300 2650
Wire Wire Line
	6000 2600 6200 2600
Wire Wire Line
	6200 2600 6200 2750
Wire Wire Line
	5650 2100 7400 2100
Wire Wire Line
	8000 3800 8100 3800
$Comp
L +5V #PWR01
U 1 1 5A2DDBEC
P 8100 1000
F 0 "#PWR01" H 8100 850 50  0001 C CNN
F 1 "+5V" H 8100 1140 50  0000 C CNB
F 2 "" H 8100 1000 50  0001 C CNN
F 3 "" H 8100 1000 50  0001 C CNN
	1    8100 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR02
U 1 1 5A2DE2C8
P 4750 3900
F 0 "#PWR02" H 4750 3750 50  0001 C CNN
F 1 "+5V" H 4750 4040 50  0000 C CNB
F 2 "" H 4750 3900 50  0001 C CNN
F 3 "" H 4750 3900 50  0001 C CNN
	1    4750 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 2800 4550 2800
Wire Wire Line
	4550 2800 4550 3600
Wire Wire Line
	4400 3500 4400 3900
Wire Wire Line
	4400 3900 4750 3900
Connection ~ 4550 3900
$Comp
L GND #PWR03
U 1 1 5A2DE48A
P 4750 3450
F 0 "#PWR03" H 4750 3200 50  0001 C CNN
F 1 "GND" H 4750 3300 50  0000 C CNB
F 2 "" H 4750 3450 50  0001 C CNN
F 3 "" H 4750 3450 50  0001 C CNN
	1    4750 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3400 4750 3400
Wire Wire Line
	4750 3400 4750 3450
$Comp
L GND #PWR04
U 1 1 5A2DE877
P 8050 3900
F 0 "#PWR04" H 8050 3650 50  0001 C CNN
F 1 "GND" H 8050 3750 50  0000 C CNB
F 2 "" H 8050 3900 50  0001 C CNN
F 3 "" H 8050 3900 50  0001 C CNN
	1    8050 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 3900 8050 3800
Connection ~ 8050 3800
Wire Wire Line
	8100 1000 8100 1100
$Comp
L Conn_01x03_Male J3
U 1 1 5A2DEA4C
P 9200 2650
F 0 "J3" H 9350 2850 50  0000 C CNB
F 1 "Upgrade Option" H 9350 2950 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B03B-XH-A_03x2.50mm_Straight" H 9200 2650 50  0001 C CNN
F 3 "" H 9200 2650 50  0001 C CNN
	1    9200 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	6600 1800 6600 4500
Connection ~ 6600 1800
Wire Wire Line
	6700 1600 6700 4400
Connection ~ 6700 1600
Wire Wire Line
	6800 2850 6800 4300
Wire Wire Line
	6800 2850 7400 2850
$Comp
L Conn_01x10_Male J4
U 1 1 5A2DED48
P 10300 2450
F 0 "J4" H 9900 2750 50  0000 C CNB
F 1 "Input from Main PCB" H 9850 2600 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B10B-XH-A_10x2.50mm_Straight" H 10300 2450 50  0001 C CNN
F 3 "" H 10300 2450 50  0001 C CNN
	1    10300 2450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8800 2050 10100 2050
Wire Wire Line
	8800 2150 10100 2150
Wire Wire Line
	8800 2250 10100 2250
Wire Wire Line
	8800 2350 10100 2350
Wire Wire Line
	8800 2450 10100 2450
Wire Wire Line
	9600 4300 9600 2550
Wire Wire Line
	9600 2550 10100 2550
Wire Wire Line
	6700 4400 9700 4400
Wire Wire Line
	9700 4400 9700 2650
Wire Wire Line
	9700 2650 10100 2650
Wire Wire Line
	9800 4500 6600 4500
Wire Wire Line
	9800 2750 10100 2750
Wire Wire Line
	8800 2550 9000 2550
Wire Wire Line
	8800 2650 9000 2650
Wire Wire Line
	8800 2750 9000 2750
$Comp
L Conn_01x02_Male J2
U 1 1 5A2E0588
P 7400 3900
F 0 "J2" H 7400 4000 50  0000 C CNB
F 1 "Upgrade Option" H 7400 4100 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 7400 3900 50  0001 C CNN
F 3 "" H 7400 3900 50  0001 C CNN
	1    7400 3900
	-1   0    0    1   
$EndComp
Wire Wire Line
	7200 3800 7100 3800
Wire Wire Line
	7100 3800 7100 3050
Wire Wire Line
	7100 3050 7400 3050
Wire Wire Line
	7400 2950 7000 2950
Wire Wire Line
	7000 2950 7000 3900
Wire Wire Line
	7000 3900 7200 3900
$Comp
L +5V #PWR05
U 1 1 5A2E0733
P 10250 3300
F 0 "#PWR05" H 10250 3150 50  0001 C CNN
F 1 "+5V" H 10250 3440 50  0000 C CNB
F 2 "" H 10250 3300 50  0001 C CNN
F 3 "" H 10250 3300 50  0001 C CNN
	1    10250 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5A2E0765
P 9900 3150
F 0 "#PWR06" H 9900 2900 50  0001 C CNN
F 1 "GND" H 9900 3000 50  0000 C CNB
F 2 "" H 9900 3150 50  0001 C CNN
F 3 "" H 9900 3150 50  0001 C CNN
	1    9900 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1000 7950 1000
Wire Wire Line
	7950 1000 7950 1100
NoConn ~ 8250 1100
NoConn ~ 8800 1850
NoConn ~ 8800 3450
NoConn ~ 8800 3550
Wire Wire Line
	9900 3150 9900 2850
Wire Wire Line
	9900 2850 10100 2850
Wire Wire Line
	10100 2950 10100 3300
Wire Wire Line
	10100 3300 10250 3300
Wire Wire Line
	4550 1700 7400 1700
Wire Wire Line
	4400 2300 7400 2300
Wire Wire Line
	6800 4300 9600 4300
Wire Wire Line
	9800 2750 9800 4500
$Comp
L Conn_01x01_Male J1
U 1 1 5A2EB98C
P 6950 1000
F 0 "J1" H 6950 1100 50  0000 C CNB
F 1 "+3V3 output (option)" H 6950 900 50  0000 C CNB
F 2 "Connectors:1pin" H 6950 1000 50  0001 C CNN
F 3 "" H 6950 1000 50  0001 C CNN
	1    6950 1000
	1    0    0    -1  
$EndComp
Text Notes 7450 7500 0    60   ~ 12
DC VOLTAGE CALIBRATOR  - TFT Arduino Nano Module
Text Notes 8300 6900 0    60   ~ 12
SCULLCOM HOBBY ELECTRONICS
Text Notes 8450 7650 0    60   ~ 12
27th December 2017
Text Notes 10700 7650 0    60   ~ 12
1.0
$EndSCHEMATC
