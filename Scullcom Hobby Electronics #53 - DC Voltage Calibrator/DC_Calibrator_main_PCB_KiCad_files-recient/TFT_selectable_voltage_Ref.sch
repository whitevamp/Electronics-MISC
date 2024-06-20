EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "22 nov 2017"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:CP C3
U 1 1 5A15BBB9
P 1800 1250
F 0 "C3" H 1825 1350 50  0000 L CNN
F 1 "220uF" H 1825 1150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P5.00mm" H 1838 1100 50  0001 C CNN
F 3 "" H 1800 1250 50  0001 C CNN
	1    1800 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C10
U 1 1 5A15BC5E
P 3400 1250
F 0 "C10" H 3425 1350 50  0000 L CNN
F 1 "100uF" H 3425 1150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P2.50mm" H 3438 1100 50  0001 C CNN
F 3 "" H 3400 1250 50  0001 C CNN
	1    3400 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C16
U 1 1 5A15BCB5
P 4900 1250
F 0 "C16" H 4925 1350 50  0000 L CNN
F 1 "100uF" H 4925 1150 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P2.50mm" H 4938 1100 50  0001 C CNN
F 3 "" H 4900 1250 50  0001 C CNN
	1    4900 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5A15BCF2
P 800 2600
F 0 "C2" H 825 2700 50  0000 L CNN
F 1 "22uF" H 825 2500 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 838 2450 50  0001 C CNN
F 3 "" H 800 2600 50  0001 C CNN
	1    800  2600
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C7
U 1 1 5A15BD41
P 2950 3150
F 0 "C7" H 2975 3250 50  0000 L CNN
F 1 "100uF" H 2975 3050 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P2.50mm" H 2988 3000 50  0001 C CNN
F 3 "" H 2950 3150 50  0001 C CNN
	1    2950 3150
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C14
U 1 1 5A15BD8C
P 4300 2450
F 0 "C14" H 4150 2600 50  0000 L CNN
F 1 "1uF" H 4325 2350 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 4338 2300 50  0001 C CNN
F 3 "" H 4300 2450 50  0001 C CNN
	1    4300 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:D D1
U 1 1 5A15BE26
P 1450 1000
F 0 "D1" H 1400 900 50  0000 C CNN
F 1 "1N4007" H 1500 1100 50  0000 C CNN
F 2 "Diodes_THT:D_T-1_P10.16mm_Horizontal" H 1450 1000 50  0001 C CNN
F 3 "" H 1450 1000 50  0001 C CNN
	1    1450 1000
	-1   0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:LM7805_TO220 U8
U 1 1 5A15BEE5
P 4200 1000
F 0 "U8" H 4300 700 70  0000 C CNB
F 1 "LM7805_TO220" H 3950 1150 70  0000 L CNB
F 2 "Power_Integrations:TO-220" H 4200 1225 50  0001 C CIN
F 3 "" H 4200 950 50  0001 C CNN
	1    4200 1000
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:LM7812_TO220 U4
U 1 1 5A15BF84
P 2600 1000
F 0 "U4" H 2700 700 70  0000 C CNB
F 1 "LM7815_TO220" H 2350 1150 70  0000 L CNB
F 2 "Power_Integrations:TO-220" H 2600 1225 50  0001 C CIN
F 3 "" H 2600 950 50  0001 C CNN
	1    2600 1000
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:LM7912_TO220 U7
U 1 1 5A15BFE1
P 3650 2800
F 0 "U7" H 3750 3100 50  0000 C CNB
F 1 "LM7915_TO220" H 3300 2650 50  0000 L CNB
F 2 "Power_Integrations:TO-220" H 3650 2600 50  0001 C CIN
F 3 "" H 3650 2800 50  0001 C CNN
	1    3650 2800
	1    0    0    1   
$EndComp
Wire Wire Line
	1800 1000 2100 1000
Wire Wire Line
	1800 600  1800 1000
Connection ~ 1800 1000
Wire Wire Line
	2900 1000 3100 1000
Connection ~ 3100 1000
Wire Wire Line
	3400 850  3400 1000
Connection ~ 3400 1000
Wire Wire Line
	4500 1000 4600 1000
Wire Wire Line
	4900 850  4900 1000
Wire Wire Line
	1800 1700 1800 1400
Wire Wire Line
	1100 1700 1200 1700
Wire Wire Line
	4900 1700 4900 1400
Wire Wire Line
	3400 1400 3400 1700
Connection ~ 3400 1700
Wire Wire Line
	4200 1300 4200 1700
Connection ~ 4200 1700
Wire Wire Line
	2600 1300 2600 1700
Connection ~ 2600 1700
Wire Wire Line
	800  2450 800  2400
Wire Wire Line
	800  2400 1200 2400
Wire Wire Line
	800  2750 800  2800
Wire Wire Line
	800  2800 1200 2800
Wire Wire Line
	1200 2600 1100 2600
Connection ~ 4900 1000
Wire Wire Line
	1100 2600 1100 3350
Wire Wire Line
	1100 3350 2950 3350
Wire Wire Line
	3650 3350 3650 3100
Connection ~ 2950 3350
Wire Wire Line
	2900 2800 2950 2800
Connection ~ 2950 2800
Wire Wire Line
	4550 2600 4550 3350
Wire Wire Line
	3950 2800 4050 2800
Wire Wire Line
	4050 2800 4050 2750
Connection ~ 3650 3350
$Comp
L power:+5V #PWR01
U 1 1 5A16502A
P 4900 850
F 0 "#PWR01" H 4900 700 50  0001 C CNN
F 1 "+5V" H 4900 990 50  0000 C CNB
F 2 "" H 4900 850 50  0001 C CNN
F 3 "" H 4900 850 50  0001 C CNN
	1    4900 850 
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:POT_TRIM RV1
U 1 1 5A1656E4
P 6200 2600
F 0 "RV1" H 6050 2650 50  0000 C CNN
F 1 "10K" H 6350 2700 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Bourns_3296W" H 6200 2600 50  0001 C CNN
F 3 "" H 6200 2600 50  0001 C CNN
	1    6200 2600
	-1   0    0    1   
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:+4.096V #PWR02
U 1 1 5A165AFC
P 6450 2100
F 0 "#PWR02" H 6450 1950 50  0001 C CNN
F 1 "+4.096V" H 6450 2250 50  0000 C CNB
F 2 "" H 6700 2150 50  0001 C CNN
F 3 "" H 6700 2150 50  0001 C CNN
	1    6450 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C18
U 1 1 5A165F80
P 6450 3050
F 0 "C18" H 6475 3150 50  0000 L CNN
F 1 "22uF" H 6475 2950 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.00mm" H 6488 2900 50  0001 C CNN
F 3 "" H 6450 3050 50  0001 C CNN
	1    6450 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2100 6450 2400
Wire Wire Line
	5950 2400 6200 2400
Connection ~ 6200 2400
Wire Wire Line
	5950 2600 6050 2600
Wire Wire Line
	6200 3350 6200 2750
Wire Wire Line
	6450 3200 6450 3350
Connection ~ 6450 3350
$Comp
L power:+5V #PWR03
U 1 1 5A168173
P 6050 3700
F 0 "#PWR03" H 6050 3550 50  0001 C CNN
F 1 "+5V" H 6050 3840 50  0000 C CNB
F 2 "" H 6050 3700 50  0001 C CNN
F 3 "" H 6050 3700 50  0001 C CNN
	1    6050 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5A168C59
P 4300 4100
F 0 "#PWR04" H 4300 3850 50  0001 C CNN
F 1 "GND" H 4300 3950 50  0000 C CNB
F 2 "" H 4300 4100 50  0001 C CNN
F 3 "" H 4300 4100 50  0001 C CNN
	1    4300 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5A169FDE
P 900 7300
F 0 "#PWR05" H 900 7050 50  0001 C CNN
F 1 "GND" H 900 7150 50  0000 C CNB
F 2 "" H 900 7300 50  0001 C CNN
F 3 "" H 900 7300 50  0001 C CNN
	1    900  7300
	1    0    0    -1  
$EndComp
Text GLabel 4250 6800 0    60   Input ~ 12
A1
Text GLabel 10050 2750 0    60   Input ~ 12
A1
Wire Wire Line
	10400 3700 10600 3700
Text GLabel 10050 2950 0    60   Input ~ 12
A2
Text GLabel 5550 6800 2    60   Input ~ 12
A2
$Comp
L Device:C C8
U 1 1 5A16D894
P 3100 1250
F 0 "C8" H 3125 1350 50  0000 L CNN
F 1 "100nF" H 3125 1150 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3138 1100 50  0001 C CNN
F 3 "" H 3100 1250 50  0001 C CNN
	1    3100 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1100 3100 1000
Wire Wire Line
	3100 1400 3100 1700
Connection ~ 3100 1700
$Comp
L Device:C C5
U 1 1 5A16DBEE
P 2100 1250
F 0 "C5" H 2125 1350 50  0000 L CNN
F 1 "100nF" H 2125 1150 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 2138 1100 50  0001 C CNN
F 3 "" H 2100 1250 50  0001 C CNN
	1    2100 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5A16DC4D
P 4600 1250
F 0 "C15" H 4625 1350 50  0000 L CNN
F 1 "100nF" H 4625 1150 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 4638 1100 50  0001 C CNN
F 3 "" H 4600 1250 50  0001 C CNN
	1    4600 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1100 4600 1000
Connection ~ 4600 1000
Wire Wire Line
	4600 1400 4600 1700
Connection ~ 4600 1700
Wire Wire Line
	2100 1400 2100 1700
Connection ~ 2100 1700
Wire Wire Line
	2100 1000 2100 1100
Connection ~ 2100 1000
$Comp
L Device:C C1
U 1 1 5A16E0E0
P 650 4450
F 0 "C1" H 675 4550 50  0000 L CNN
F 1 "100nF" H 675 4350 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 688 4300 50  0001 C CNN
F 3 "" H 650 4450 50  0001 C CNN
	1    650  4450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5A16E153
P 2400 4450
F 0 "C6" H 2425 4550 50  0000 L CNN
F 1 "100nF" H 2150 4300 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 2438 4300 50  0001 C CNN
F 3 "" H 2400 4450 50  0001 C CNN
	1    2400 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5A16E1D0
P 2400 4650
F 0 "#PWR06" H 2400 4400 50  0001 C CNN
F 1 "GND" H 2400 4500 50  0000 C CNB
F 2 "" H 2400 4650 50  0001 C CNN
F 3 "" H 2400 4650 50  0001 C CNN
	1    2400 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5A16E22A
P 650 4650
F 0 "#PWR07" H 650 4400 50  0001 C CNN
F 1 "GND" H 650 4500 50  0000 C CNB
F 2 "" H 650 4650 50  0001 C CNN
F 3 "" H 650 4650 50  0001 C CNN
	1    650  4650
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:Jack-DC J1
U 1 1 5A16EEAB
P 900 1100
F 0 "J1" H 900 1310 50  0000 C CNB
F 1 "18 Volt DC In" H 900 925 50  0000 C CNB
F 2 "Connectors:BARREL_JACK" H 950 1060 50  0001 C CNN
F 3 "" H 950 1060 50  0001 C CNN
	1    900  1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1200 1200 1700
Connection ~ 1800 1700
$Comp
L power:GND #PWR08
U 1 1 5A16F256
P 2600 1850
F 0 "#PWR08" H 2600 1600 50  0001 C CNN
F 1 "GND" H 2600 1700 50  0000 C CNB
F 2 "" H 2600 1850 50  0001 C CNN
F 3 "" H 2600 1850 50  0001 C CNN
	1    2600 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5A16F3AB
P 9100 3500
F 0 "#PWR09" H 9100 3250 50  0001 C CNN
F 1 "GND" H 9100 3350 50  0000 C CNB
F 2 "" H 9100 3500 50  0001 C CNN
F 3 "" H 9100 3500 50  0001 C CNN
	1    9100 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 5A172A7A
P 5800 3550
F 0 "C17" H 5825 3650 50  0000 L CNN
F 1 "100nF" H 5450 3600 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 5838 3400 50  0001 C CNN
F 3 "" H 5800 3550 50  0001 C CNN
	1    5800 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 3350 5800 3400
Connection ~ 5800 3350
$Comp
L power:GND #PWR010
U 1 1 5A172B45
P 3950 7300
F 0 "#PWR010" H 3950 7050 50  0001 C CNN
F 1 "GND" H 3950 7150 50  0000 C CNB
F 2 "" H 3950 7300 50  0001 C CNN
F 3 "" H 3950 7300 50  0001 C CNN
	1    3950 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4900 4350 5100
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x02 J3
U 1 1 5A17391B
P 3950 4300
F 0 "J3" H 3950 4400 50  0000 C CNN
F 1 "B Vref." H 3650 4350 50  0000 C CNB
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3950 4300 50  0001 C CNN
F 3 "" H 3950 4300 50  0001 C CNN
	1    3950 4300
	-1   0    0    1   
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x02 J4
U 1 1 5A173B88
P 3950 4600
F 0 "J4" H 3950 4700 50  0000 C CNN
F 1 "DAC B Out" H 3600 4550 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 3950 4600 50  0001 C CNN
F 3 "" H 3950 4600 50  0001 C CNN
	1    3950 4600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5A173CF3
P 4200 4700
F 0 "#PWR011" H 4200 4450 50  0001 C CNN
F 1 "GND" H 4200 4550 50  0000 C CNB
F 2 "" H 4200 4700 50  0001 C CNN
F 3 "" H 4200 4700 50  0001 C CNN
	1    4200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4600 4200 4600
Wire Wire Line
	4200 4600 4200 4700
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x02 J5
U 1 1 5A173F68
P 6100 6300
F 0 "J5" H 6100 6400 50  0000 C CNN
F 1 "DC Reference Output" H 6600 6300 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 6100 6300 50  0001 C CNN
F 3 "" H 6100 6300 50  0001 C CNN
	1    6100 6300
	1    0    0    1   
$EndComp
$Comp
L Device:C C9
U 1 1 5A1752DF
P 3300 3150
F 0 "C9" H 3325 3250 50  0000 L CNN
F 1 "100nF" H 3325 3050 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3338 3000 50  0001 C CNN
F 3 "" H 3300 3150 50  0001 C CNN
	1    3300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 3000 3300 2800
Connection ~ 3300 2800
Wire Wire Line
	3300 3300 3300 3350
Connection ~ 3300 3350
Text GLabel 10050 2550 0    60   Input ~ 12
A0
Wire Wire Line
	10500 3600 10600 3600
Wire Wire Line
	10500 2550 10500 3600
Wire Wire Line
	10400 2750 10400 3700
Wire Wire Line
	10050 2750 10400 2750
Wire Wire Line
	10600 3800 10300 3800
Connection ~ 4550 3350
Connection ~ 6200 3350
Wire Wire Line
	10200 3900 10600 3900
Wire Wire Line
	10100 4000 10600 4000
$Comp
L Device:CP C13
U 1 1 5A18AB23
P 4200 3050
F 0 "C13" H 4050 2950 50  0000 L CNN
F 1 "100uF" H 3950 3200 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P2.50mm" H 4238 2900 50  0001 C CNN
F 3 "" H 4200 3050 50  0001 C CNN
	1    4200 3050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4200 2800 4200 2900
Connection ~ 4050 2800
Wire Wire Line
	4200 3200 4200 3350
Connection ~ 4200 3350
Wire Wire Line
	2950 3300 2950 3350
Wire Wire Line
	2950 3000 2950 2800
$Comp
L Device:C C12
U 1 1 5A18B1C2
P 3950 3150
F 0 "C12" H 3975 3250 50  0000 L CNN
F 1 "100nF" H 3700 3050 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3988 3000 50  0001 C CNN
F 3 "" H 3950 3150 50  0001 C CNN
	1    3950 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 3000 3950 2800
Wire Wire Line
	3950 3300 3950 3350
Connection ~ 3950 3350
Connection ~ 3950 2800
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x04_Female J8
U 1 1 5A190475
P 9600 5600
F 0 "J8" H 9600 5800 50  0000 C CNB
F 1 "Option Power" H 9600 5300 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B04B-XH-A_04x2.50mm_Straight" H 9600 5600 50  0001 C CNN
F 3 "" H 9600 5600 50  0001 C CNN
	1    9600 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR012
U 1 1 5A190656
P 8700 5450
F 0 "#PWR012" H 8700 5300 50  0001 C CNN
F 1 "+5V" H 8700 5590 50  0000 C CNB
F 2 "" H 8700 5450 50  0001 C CNN
F 3 "" H 8700 5450 50  0001 C CNN
	1    8700 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5A1906C8
P 9050 5900
F 0 "#PWR013" H 9050 5650 50  0001 C CNN
F 1 "GND" H 9050 5750 50  0000 C CNB
F 2 "" H 9050 5900 50  0001 C CNN
F 3 "" H 9050 5900 50  0001 C CNN
	1    9050 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 5900 9050 5800
Wire Wire Line
	9050 5800 9400 5800
Wire Wire Line
	9250 5500 9400 5500
Wire Wire Line
	4300 2600 4550 2600
Wire Wire Line
	4300 2300 4550 2300
Wire Wire Line
	4550 2300 4550 2400
NoConn ~ 1200 2200
NoConn ~ 2900 2400
NoConn ~ 2900 2600
NoConn ~ 4350 4700
Wire Wire Line
	6200 2450 6200 2400
$Comp
L TFT_selectable_voltage_Ref-rescue:INA105-RESCUE-TFT_selectable_voltage_Ref U5
U 1 1 5A17F56D
P 3050 4000
F 0 "U5" H 3010 4490 70  0000 L CNB
F 1 "INA105" H 2860 3490 70  0000 L CNB
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 3050 4000 60  0001 C CNN
F 3 "" H 3050 4000 60  0001 C CNN
	1    3050 4000
	-1   0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:MAX6341 U10
U 1 1 5A17FB31
P 5250 2300
F 0 "U10" H 5200 2800 70  0000 L CNB
F 1 "MAX6341" H 5000 1700 70  0000 L CNB
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 5250 2300 60  0001 C CNN
F 3 "" H 5250 2300 60  0001 C CNN
	1    5250 2300
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:TC962 U3
U 1 1 5A1806F6
P 2050 2500
F 0 "U3" H 2000 3000 70  0000 L CNB
F 1 "TC962" H 1900 2000 70  0000 L CNB
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 2050 2500 60  0001 C CNN
F 3 "" H 2050 2500 60  0001 C CNN
	1    2050 2500
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:MCP4922EP U9
U 1 1 5A180EBF
P 5000 4300
F 0 "U9" H 4960 5110 70  0000 L CNB
F 1 "MCP4922EP" H 4700 3500 70  0000 L CNB
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 5000 4300 60  0001 C CNN
F 3 "" H 5000 4300 60  0001 C CNN
	1    5000 4300
	-1   0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:DG303B U6
U 1 1 5A181FF9
P 4900 6400
F 0 "U6" H 4860 7210 70  0000 L CNB
F 1 "DG303B" H 4700 5560 70  0000 L CNB
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 4900 6400 60  0001 C CNN
F 3 "" H 4900 6400 60  0001 C CNN
	1    4900 6400
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:DG303B U2
U 1 1 5A186DF6
P 1600 6400
F 0 "U2" H 1560 7210 70  0000 L CNB
F 1 "DG303B" H 1400 5560 70  0000 L CNB
F 2 "Housings_DIP:DIP-14_W7.62mm_Socket_LongPads" H 1600 6400 60  0001 C CNN
F 3 "" H 1600 6400 60  0001 C CNN
	1    1600 6400
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:INA105-RESCUE-TFT_selectable_voltage_Ref U1
U 1 1 5A186EE7
P 1350 4000
F 0 "U1" H 1310 4490 70  0000 L CNB
F 1 "INA105" H 1160 3490 70  0000 L CNB
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 1350 4000 60  0001 C CNN
F 3 "" H 1350 4000 60  0001 C CNN
	1    1350 4000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5650 3700 5800 3700
Connection ~ 5800 3700
Wire Wire Line
	4150 4500 4350 4500
Wire Wire Line
	4150 4300 4350 4300
Wire Wire Line
	2400 3900 2600 3900
Wire Wire Line
	2400 3600 2400 3900
Wire Wire Line
	2150 4100 2600 4100
Wire Wire Line
	4250 6200 4250 6300
Wire Wire Line
	950  6200 950  6300
Wire Wire Line
	5550 6200 5550 6300
Wire Wire Line
	2250 6200 2250 6300
Wire Wire Line
	4150 3900 4200 3900
Wire Wire Line
	3400 6000 4250 6000
Connection ~ 2600 4300
Wire Wire Line
	3500 3900 3800 3900
Wire Wire Line
	3800 6300 4250 6300
Connection ~ 4250 6300
Wire Wire Line
	600  6300 950  6300
Connection ~ 950  6300
Wire Wire Line
	650  4650 650  4600
Wire Wire Line
	2400 4600 2400 4650
Wire Wire Line
	900  4100 900  4300
Wire Wire Line
	1800 3900 2000 3900
Wire Wire Line
	1800 3700 1900 3700
Wire Wire Line
	1900 3700 1900 4100
Wire Wire Line
	1900 4100 1800 4100
Wire Wire Line
	3500 4100 3600 4100
Text GLabel 10050 3150 0    60   Input ~ 12
A3
Wire Wire Line
	10050 3150 10200 3150
Wire Wire Line
	10200 3150 10200 3900
Wire Wire Line
	10050 2950 10300 2950
Wire Wire Line
	10300 2950 10300 3800
Wire Wire Line
	10050 2550 10500 2550
Wire Wire Line
	3950 6600 3950 7000
Wire Wire Line
	3950 6600 4250 6600
Wire Wire Line
	3950 7000 4250 7000
Connection ~ 3950 7000
Wire Wire Line
	2250 7000 2350 7000
Wire Wire Line
	2250 5800 2400 5800
Wire Wire Line
	2400 5800 2400 5700
Text GLabel 950  6800 0    60   Input ~ 12
A3
Connection ~ 5550 6300
Wire Wire Line
	600  7600 6000 7600
Wire Wire Line
	6000 7600 6000 6600
Wire Wire Line
	6000 6600 5550 6600
$Comp
L power:GND #PWR014
U 1 1 5A19C567
P 2000 4000
F 0 "#PWR014" H 2000 3750 50  0001 C CNN
F 1 "GND" H 2000 3850 50  0000 C CNN
F 2 "" H 2000 4000 50  0001 C CNN
F 3 "" H 2000 4000 50  0001 C CNN
	1    2000 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3700 2150 4100
Wire Wire Line
	600  7600 600  6300
Wire Wire Line
	900  7000 950  7000
Wire Wire Line
	900  5000 700  5000
Wire Wire Line
	700  5000 700  6600
Connection ~ 900  4300
Wire Wire Line
	700  6600 950  6600
Connection ~ 2150 4100
Connection ~ 1900 3700
Wire Wire Line
	2000 3900 2000 4000
Wire Wire Line
	3800 3900 3800 6300
Text GLabel 2250 6800 2    60   Input ~ 12
A4
Wire Wire Line
	2250 6600 2800 6600
Wire Wire Line
	2250 6000 2800 6000
Wire Wire Line
	2250 6300 2450 6300
Connection ~ 2250 6300
Text GLabel 4150 5100 0    60   Input ~ 12
A0
Wire Wire Line
	4350 5100 4150 5100
Text GLabel 10050 3350 0    60   Input ~ 12
A4
Wire Wire Line
	10050 3350 10100 3350
Wire Wire Line
	10100 3350 10100 4000
$Comp
L Device:C C4
U 1 1 5A1ACD54
P 1800 4650
F 0 "C4" H 1825 4750 50  0000 L CNN
F 1 "100nF" H 1825 4550 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 1838 4500 50  0001 C CNN
F 3 "" H 1800 4650 50  0001 C CNN
	1    1800 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5A1ACE35
P 3550 4900
F 0 "C11" H 3575 5000 50  0000 L CNN
F 1 "100nF" H 3575 4800 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3588 4750 50  0001 C CNN
F 3 "" H 3550 4900 50  0001 C CNN
	1    3550 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5A1ACF35
P 1800 4900
F 0 "#PWR015" H 1800 4650 50  0001 C CNN
F 1 "GND" H 1800 4750 50  0000 C CNN
F 2 "" H 1800 4900 50  0001 C CNN
F 3 "" H 1800 4900 50  0001 C CNN
	1    1800 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5A1ACFB3
P 3550 5100
F 0 "#PWR016" H 3550 4850 50  0001 C CNN
F 1 "GND" H 3550 4950 50  0000 C CNN
F 2 "" H 3550 5100 50  0001 C CNN
F 3 "" H 3550 5100 50  0001 C CNN
	1    3550 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 4300 1800 4500
Wire Wire Line
	1800 4900 1800 4800
Connection ~ 1800 4300
Connection ~ 2400 3900
$Comp
L power:GND #PWR017
U 1 1 5A1AECBB
P 6450 6000
F 0 "#PWR017" H 6450 5750 50  0001 C CNN
F 1 "GND" H 6450 5850 50  0000 C CNB
F 2 "" H 6450 6000 50  0001 C CNN
F 3 "" H 6450 6000 50  0001 C CNN
	1    6450 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 6000 5900 6000
Wire Wire Line
	5550 7000 5650 7000
Wire Wire Line
	1800 4300 1850 4300
Wire Wire Line
	5550 5800 5750 5800
Wire Wire Line
	3500 2200 3500 2050
Connection ~ 3500 2200
Wire Wire Line
	650  3600 650  3900
Wire Wire Line
	650  3900 900  3900
Connection ~ 650  3900
Wire Wire Line
	3550 4300 3550 4700
Connection ~ 3550 4700
Wire Wire Line
	3550 4300 3500 4300
Wire Wire Line
	3550 5100 3550 5050
Wire Wire Line
	3600 4100 3600 3700
Connection ~ 3600 3700
Wire Wire Line
	8700 5450 8700 5600
Wire Wire Line
	8700 5600 9400 5600
Wire Wire Line
	8400 5450 8400 5700
Wire Wire Line
	8400 5700 9400 5700
Wire Wire Line
	9250 5500 9250 5450
Wire Wire Line
	5550 6300 5900 6300
Wire Wire Line
	5900 6000 5900 6200
Connection ~ 5900 6000
Wire Wire Line
	2900 2200 3500 2200
Wire Wire Line
	2150 5300 800  5300
Wire Wire Line
	950  6000 800  6000
Wire Wire Line
	800  6000 800  5300
$Comp
L power:+15V #PWR018
U 1 1 5A208583
P 3400 850
F 0 "#PWR018" H 3400 700 50  0001 C CNN
F 1 "+15V" H 3400 990 50  0000 C CNB
F 2 "" H 3400 850 50  0001 C CNN
F 3 "" H 3400 850 50  0001 C CNN
	1    3400 850 
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR019
U 1 1 5A2095E2
P 3500 2050
F 0 "#PWR019" H 3500 1900 50  0001 C CNN
F 1 "+15V" H 3500 2190 50  0000 C CNB
F 2 "" H 3500 2050 50  0001 C CNN
F 3 "" H 3500 2050 50  0001 C CNN
	1    3500 2050
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR020
U 1 1 5A20BB0B
P 2400 3600
F 0 "#PWR020" H 2400 3450 50  0001 C CNN
F 1 "+15V" H 2400 3740 50  0000 C CNB
F 2 "" H 2400 3600 50  0001 C CNN
F 3 "" H 2400 3600 50  0001 C CNN
	1    2400 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR021
U 1 1 5A20D65F
P 650 3600
F 0 "#PWR021" H 650 3450 50  0001 C CNN
F 1 "+15V" H 650 3740 50  0000 C CNB
F 2 "" H 650 3600 50  0001 C CNN
F 3 "" H 650 3600 50  0001 C CNN
	1    650  3600
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR022
U 1 1 5A212855
P 8400 5450
F 0 "#PWR022" H 8400 5300 50  0001 C CNN
F 1 "+15V" H 8400 5590 50  0000 C CNB
F 2 "" H 8400 5450 50  0001 C CNN
F 3 "" H 8400 5450 50  0001 C CNN
	1    8400 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR023
U 1 1 5A2131E9
P 5750 5650
F 0 "#PWR023" H 5750 5500 50  0001 C CNN
F 1 "+15V" H 5750 5790 50  0000 C CNB
F 2 "" H 5750 5650 50  0001 C CNN
F 3 "" H 5750 5650 50  0001 C CNN
	1    5750 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+15V #PWR024
U 1 1 5A21701D
P 2400 5700
F 0 "#PWR024" H 2400 5550 50  0001 C CNN
F 1 "+15V" H 2400 5840 50  0000 C CNB
F 2 "" H 2400 5700 50  0001 C CNN
F 3 "" H 2400 5700 50  0001 C CNN
	1    2400 5700
	1    0    0    -1  
$EndComp
$Comp
L power:-15V #PWR0101
U 1 1 5A21A841
P 4050 2750
F 0 "#PWR0101" H 4050 2850 50  0001 C CNN
F 1 "-15V" H 4050 2900 50  0000 C CNB
F 2 "" H 4050 2750 50  0001 C CNN
F 3 "" H 4050 2750 50  0001 C CNN
	1    4050 2750
	1    0    0    -1  
$EndComp
$Comp
L power:-15V #PWR0102
U 1 1 5A21CA2F
P 1850 4300
F 0 "#PWR0102" H 1850 4400 50  0001 C CNN
F 1 "-15V" V 1900 4400 50  0000 C CNB
F 2 "" H 1850 4300 50  0001 C CNN
F 3 "" H 1850 4300 50  0001 C CNN
	1    1850 4300
	0    1    1    0   
$EndComp
$Comp
L power:-15V #PWR0103
U 1 1 5A22225D
P 3350 4700
F 0 "#PWR0103" H 3350 4800 50  0001 C CNN
F 1 "-15V" V 3250 4800 50  0000 C CNB
F 2 "" H 3350 4700 50  0001 C CNN
F 3 "" H 3350 4700 50  0001 C CNN
	1    3350 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:-15V #PWR0104
U 1 1 5A2232CB
P 2350 7000
F 0 "#PWR0104" H 2350 7100 50  0001 C CNN
F 1 "-15V" V 2450 7000 50  0000 C CNB
F 2 "" H 2350 7000 50  0001 C CNN
F 3 "" H 2350 7000 50  0001 C CNN
	1    2350 7000
	0    1    1    0   
$EndComp
$Comp
L power:-15V #PWR0105
U 1 1 5A22335F
P 5650 7000
F 0 "#PWR0105" H 5650 7100 50  0001 C CNN
F 1 "-15V" V 5750 7000 50  0000 C CNB
F 2 "" H 5650 7000 50  0001 C CNN
F 3 "" H 5650 7000 50  0001 C CNN
	1    5650 7000
	0    1    1    0   
$EndComp
$Comp
L power:-15V #PWR034
U 1 1 5A2251C0
P 9250 5450
F 0 "#PWR034" H 9250 5550 50  0001 C CNN
F 1 "-15V" H 9250 5600 50  0000 C CNB
F 2 "" H 9250 5450 50  0001 C CNN
F 3 "" H 9250 5450 50  0001 C CNN
	1    9250 5450
	1    0    0    -1  
$EndComp
Text Notes 7400 7500 0    60   ~ 12
DC VOLTAGE CALIBRATOR - Main PCB
Text Notes 7700 6900 0    120  ~ 24
SCULLCOM HOBBY ELECTRONICS
Text Notes 5250 750  0    98   ~ 20
DC VOLTAGE CALIBRATOR - MAIN PCB
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x10_Male J7
U 1 1 5A342196
P 10800 4000
F 0 "J7" H 10800 4500 50  0000 C CNB
F 1 "To Display/Nano" V 10600 3950 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B10B-XH-A_10x2.50mm_Straight" H 10800 4000 50  0001 C CNN
F 3 "" H 10800 4000 50  0001 C CNN
	1    10800 4000
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR025
U 1 1 5A3424F3
P 10500 5000
F 0 "#PWR025" H 10500 4850 50  0001 C CNN
F 1 "+5V" H 10500 5140 50  0000 C CNB
F 2 "" H 10500 5000 50  0001 C CNN
F 3 "" H 10500 5000 50  0001 C CNN
	1    10500 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5A34256F
P 10200 4650
F 0 "#PWR026" H 10200 4400 50  0001 C CNN
F 1 "GND" H 10200 4500 50  0000 C CNB
F 2 "" H 10200 4650 50  0001 C CNN
F 3 "" H 10200 4650 50  0001 C CNN
	1    10200 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 5000 10400 5000
Wire Wire Line
	10400 5000 10400 4500
Wire Wire Line
	10400 4500 10600 4500
Wire Wire Line
	10200 4650 10200 4400
Wire Wire Line
	10200 4400 10600 4400
Wire Wire Line
	5650 4100 10600 4100
Wire Wire Line
	6550 4200 10600 4200
Wire Wire Line
	6700 4300 10600 4300
Wire Wire Line
	6550 4200 6550 4300
Wire Wire Line
	6550 4300 5650 4300
Wire Wire Line
	6700 4300 6700 4500
Wire Wire Line
	6700 4500 5650 4500
$Comp
L Device:C C19
U 1 1 5A345E02
P 7000 2350
F 0 "C19" H 7025 2450 50  0000 L CNN
F 1 "2.2uF" H 7025 2250 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 7038 2200 50  0001 C CNN
F 3 "" H 7000 2350 50  0001 C CNN
	1    7000 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2600 7600 3350
Connection ~ 7600 3350
Wire Wire Line
	900  7300 900  7000
Wire Wire Line
	2600 4100 2600 4300
Wire Wire Line
	3400 6000 3400 5400
Wire Wire Line
	3400 5400 2600 5400
Wire Wire Line
	5750 5800 5750 5650
Wire Wire Line
	3350 4700 3550 4700
$Comp
L TFT_selectable_voltage_Ref-rescue:+1.024V #PWR027
U 1 1 5A34D479
P 9100 2100
F 0 "#PWR027" H 9100 1950 50  0001 C CNN
F 1 "+1.024V" H 9100 2240 50  0000 C CNB
F 2 "" H 9100 2100 50  0001 C CNN
F 3 "" H 9100 2100 50  0001 C CNN
	1    9100 2100
	1    0    0    -1  
$EndComp
Connection ~ 6450 2400
$Comp
L power:+5V #PWR028
U 1 1 5A34EB11
P 7000 2050
F 0 "#PWR028" H 7000 1900 50  0001 C CNN
F 1 "+5V" H 7000 2190 50  0000 C CNB
F 2 "" H 7000 2050 50  0001 C CNN
F 3 "" H 7000 2050 50  0001 C CNN
	1    7000 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2050 7000 2100
$Comp
L TFT_selectable_voltage_Ref-rescue:+1.024V #PWR029
U 1 1 5A34FA1E
P 2800 6000
F 0 "#PWR029" H 2800 5850 50  0001 C CNN
F 1 "+1.024V" H 2800 6140 50  0000 C CNB
F 2 "" H 2800 6000 50  0001 C CNN
F 3 "" H 2800 6000 50  0001 C CNN
	1    2800 6000
	1    0    0    -1  
$EndComp
$Comp
L TFT_selectable_voltage_Ref-rescue:+4.096V #PWR030
U 1 1 5A34FA8C
P 2800 6600
F 0 "#PWR030" H 2800 6450 50  0001 C CNN
F 1 "+4.096V" H 2800 6750 50  0000 C CNB
F 2 "" H 3050 6650 50  0001 C CNN
F 3 "" H 3050 6650 50  0001 C CNN
	1    2800 6600
	1    0    0    -1  
$EndComp
Text GLabel 2450 6300 2    60   Input ~ 12
Vref
Text GLabel 4150 3900 0    60   Input ~ 12
Vref
Wire Wire Line
	4300 4100 4350 4100
Wire Wire Line
	4150 4200 4200 4200
Wire Wire Line
	4200 4200 4200 3900
Connection ~ 4200 3900
Text Notes 10650 7650 0    60   ~ 12
Ver 2.0
Text Notes 7900 7650 0    60   ~ 12
XXXXXXXXXXXXXX
Text Notes 8950 7650 0    60   ~ 12
27th December 2017
Text Notes 5550 3000 0    60   ~ 12
+4.096V Trim\n(optional)
Connection ~ 2600 4100
Wire Wire Line
	1200 1100 1350 1100
Wire Wire Line
	1350 1100 1350 1800
Connection ~ 1200 1700
Wire Wire Line
	1350 1800 1100 1800
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x03_Male J2
U 1 1 5A370F29
P 900 1800
F 0 "J2" H 900 1600 50  0000 C CNB
F 1 "Battery" H 950 2000 50  0000 C CNB
F 2 "Connectors_JST:JST_XH_B03B-XH-A_03x2.50mm_Straight" H 900 1800 50  0001 C CNN
F 3 "" H 900 1800 50  0001 C CNN
	1    900  1800
	1    0    0    1   
$EndComp
Wire Wire Line
	1600 700  1600 1000
Wire Wire Line
	1600 1900 1100 1900
Connection ~ 1600 1000
$Comp
L Device:D_Zener D2
U 1 1 5A3727F0
P 3650 1000
F 0 "D2" H 3700 900 50  0000 C CNN
F 1 "1N5338B" H 3650 1100 50  0000 C CNN
F 2 "Diodes_THT:D_DO-201AD_P15.24mm_Horizontal" H 3650 1000 50  0001 C CNN
F 3 "" H 3650 1000 50  0001 C CNN
	1    3650 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1000 3900 1000
$Comp
L Device:Jumper JP1
U 1 1 5A352E5D
P 4000 3700
F 0 "JP1" H 3800 3850 50  0000 C CNN
F 1 "Jumper" H 4050 3850 50  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_0.80mmDrill" H 4000 3700 50  0001 C CNN
F 3 "" H 4000 3700 50  0001 C CNN
	1    4000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3700 4350 3700
Wire Wire Line
	3500 3700 3600 3700
$Comp
L TFT_selectable_voltage_Ref-rescue:MCP1501_SOIC-8 U11
U 1 1 5A43E2AF
P 8000 2300
F 0 "U11" H 8000 2750 70  0000 C CNB
F 1 "MCP1501_10" H 7750 1800 70  0000 L CNB
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 8000 2850 50  0001 C CNN
F 3 "" H 8000 2300 50  0001 C CNN
	1    8000 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 2000 7600 2000
Connection ~ 7000 2100
Wire Wire Line
	8400 2400 8600 2400
Wire Wire Line
	8600 2400 8600 2600
Connection ~ 8600 3350
Wire Wire Line
	8400 2600 8600 2600
Connection ~ 8600 2600
Wire Wire Line
	8400 2000 8600 2000
Wire Wire Line
	8600 2000 8600 2100
Wire Wire Line
	8600 2200 8400 2200
Wire Wire Line
	9100 2100 8600 2100
Connection ~ 8600 2100
$Comp
L Device:C C20
U 1 1 5A43FB4C
P 9100 2650
F 0 "C20" H 9125 2750 50  0000 L CNN
F 1 "220pF" H 9125 2550 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L4.6mm_W2.0mm_P2.50mm_MKS02_FKP02" H 9138 2500 50  0001 C CNN
F 3 "" H 9100 2650 50  0001 C CNN
	1    9100 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2100 9100 2500
Connection ~ 9100 2100
Wire Wire Line
	7450 2600 7600 2600
Connection ~ 7600 2600
Wire Wire Line
	7450 2600 7450 2200
Wire Wire Line
	7450 2200 7600 2200
Wire Wire Line
	7000 2500 7000 3350
Connection ~ 7000 3350
Wire Wire Line
	7300 2000 7300 2100
Wire Wire Line
	7300 2400 7600 2400
Wire Wire Line
	7000 2100 7300 2100
Connection ~ 7300 2100
$Comp
L TFT_selectable_voltage_Ref-rescue:Conn_01x02_Male J6
U 1 1 5A445744
P 1300 600
F 0 "J6" H 1250 650 50  0000 C CNN
F 1 "On/Off Switch" H 950 550 50  0000 C CNN
F 2 "Connectors_JST:JST_XH_B02B-XH-A_02x2.50mm_Straight" H 1300 600 50  0001 C CNN
F 3 "" H 1300 600 50  0001 C CNN
	1    1300 600 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1000 1300 1000
Wire Wire Line
	1600 700  1500 700 
Wire Wire Line
	1500 600  1800 600 
Wire Wire Line
	9100 2800 9100 3350
Connection ~ 9100 3350
Wire Wire Line
	1800 1000 1800 1100
Wire Wire Line
	3100 1000 3400 1000
Wire Wire Line
	3400 1000 3400 1100
Wire Wire Line
	3400 1000 3500 1000
Wire Wire Line
	3400 1700 4200 1700
Wire Wire Line
	4200 1700 4600 1700
Wire Wire Line
	2600 1700 3100 1700
Wire Wire Line
	2600 1700 2600 1850
Wire Wire Line
	4900 1000 4900 1100
Wire Wire Line
	2950 3350 3300 3350
Wire Wire Line
	2950 2800 3300 2800
Wire Wire Line
	3650 3350 3950 3350
Wire Wire Line
	6200 2400 6450 2400
Wire Wire Line
	6450 3350 7000 3350
Wire Wire Line
	3100 1700 3400 1700
Wire Wire Line
	4600 1000 4900 1000
Wire Wire Line
	4600 1700 4900 1700
Wire Wire Line
	2100 1700 2600 1700
Wire Wire Line
	2100 1000 2300 1000
Wire Wire Line
	1800 1700 2100 1700
Wire Wire Line
	5800 3350 6200 3350
Wire Wire Line
	3300 2800 3350 2800
Wire Wire Line
	3300 3350 3650 3350
Wire Wire Line
	4550 3350 5800 3350
Wire Wire Line
	6200 3350 6450 3350
Wire Wire Line
	4050 2800 4200 2800
Wire Wire Line
	4200 3350 4550 3350
Wire Wire Line
	3950 3350 4200 3350
Wire Wire Line
	5800 3700 6050 3700
Wire Wire Line
	2600 4300 2600 5400
Wire Wire Line
	4250 6300 4250 6400
Wire Wire Line
	950  6300 950  6400
Wire Wire Line
	3950 7000 3950 7300
Wire Wire Line
	5550 6300 5550 6400
Wire Wire Line
	900  4300 900  5000
Wire Wire Line
	2150 4100 2150 5300
Wire Wire Line
	1900 3700 2150 3700
Wire Wire Line
	2250 6300 2250 6400
Wire Wire Line
	2400 3900 2400 4300
Wire Wire Line
	3500 2200 4550 2200
Wire Wire Line
	650  3900 650  4300
Wire Wire Line
	3550 4700 3550 4750
Wire Wire Line
	3600 3700 3700 3700
Wire Wire Line
	5900 6000 6450 6000
Wire Wire Line
	7600 3350 8600 3350
Wire Wire Line
	6450 2400 6450 2900
Wire Wire Line
	4200 3900 4350 3900
Wire Wire Line
	1200 1700 1800 1700
Wire Wire Line
	1600 1000 1600 1900
Wire Wire Line
	7000 2100 7000 2200
Wire Wire Line
	8600 3350 9100 3350
Wire Wire Line
	8600 2600 8600 3350
Wire Wire Line
	8600 2100 8600 2200
Wire Wire Line
	7000 3350 7600 3350
Wire Wire Line
	7300 2100 7300 2400
Wire Wire Line
	9100 3350 9100 3500
$EndSCHEMATC