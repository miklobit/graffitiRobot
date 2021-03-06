EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
L R R2
U 1 1 578CE4A3
P 3100 2250
F 0 "R2" V 3180 2250 50  0000 C CNN
F 1 "rpusl1" V 3100 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3030 2250 50  0001 C CNN
F 3 "" H 3100 2250 50  0000 C CNN
	1    3100 2250
	0    1    1    0   
$EndComp
$Comp
L R rpufc2
U 1 1 578CE534
P 6950 4850
F 0 "rpufc2" V 7030 4850 50  0000 C CNN
F 1 "rpufc2" V 6950 4850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6880 4850 50  0001 C CNN
F 3 "" H 6950 4850 50  0000 C CNN
	1    6950 4850
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 578CE5DC
P 4450 2250
F 0 "R3" V 4530 2250 50  0000 C CNN
F 1 "rpum" V 4450 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4380 2250 50  0001 C CNN
F 3 "" H 4450 2250 50  0000 C CNN
	1    4450 2250
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 578CE661
P 6900 2150
F 0 "R4" V 6980 2150 50  0000 C CNN
F 1 "rpusl2" V 6900 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6830 2150 50  0001 C CNN
F 3 "" H 6900 2150 50  0000 C CNN
	1    6900 2150
	0    -1   -1   0   
$EndComp
$Comp
L R rpusda1
U 1 1 578CE717
P 4500 4650
F 0 "rpusda1" V 4580 4650 50  0000 C CNN
F 1 "rpusda" V 4500 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4430 4650 50  0001 C CNN
F 3 "" H 4500 4650 50  0000 C CNN
	1    4500 4650
	1    0    0    -1  
$EndComp
$Comp
L R rpuscl1
U 1 1 578CE7A0
P 5050 4650
F 0 "rpuscl1" V 5130 4650 50  0000 C CNN
F 1 "rpuscl" V 5050 4650 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4980 4650 50  0001 C CNN
F 3 "" H 5050 4650 50  0000 C CNN
	1    5050 4650
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 578CE815
P 1750 3250
F 0 "R1" V 1830 3250 50  0000 C CNN
F 1 "R" V 1750 3250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1680 3250 50  0001 C CNN
F 3 "" H 1750 3250 50  0000 C CNN
	1    1750 3250
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 condib1
U 1 1 578CE9D1
P 4150 3800
F 0 "condib1" H 4150 4050 50  0000 C CNN
F 1 "conndib" V 4250 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 4150 3800 50  0001 C CNN
F 3 "" H 4150 3800 50  0000 C CNN
	1    4150 3800
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 fcm2
U 1 1 578CF195
P 6750 5350
F 0 "fcm2" H 6750 5500 50  0000 C CNN
F 1 "fcm2" V 6850 5350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6750 5350 50  0001 C CNN
F 3 "" H 6750 5350 50  0000 C CNN
	1    6750 5350
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P1
U 1 1 578CF254
P 1400 3550
F 0 "P1" H 1400 3700 50  0000 C CNN
F 1 "CONN_01X02" V 1500 3550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 1400 3550 50  0001 C CNN
F 3 "" H 1400 3550 50  0000 C CNN
	1    1400 3550
	-1   0    0    1   
$EndComp
$Comp
L SUPP28 J3
U 1 1 578CFDA6
P 7800 3100
F 0 "J3" H 7800 3200 50  0000 C CNN
F 1 "slave2" H 7800 3000 50  0000 C CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 7800 3100 50  0001 C CNN
F 3 "" H 7800 3100 50  0000 C CNN
	1    7800 3100
	1    0    0    -1  
$EndComp
$Comp
L SUPP28 J2
U 1 1 578D0069
P 5450 3150
F 0 "J2" H 5450 3250 50  0000 C CNN
F 1 "masteer" H 5450 3050 50  0000 C CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 5450 3150 50  0001 C CNN
F 3 "" H 5450 3150 50  0000 C CNN
	1    5450 3150
	1    0    0    -1  
$EndComp
$Comp
L SUPP28 J1
U 1 1 578D0102
P 3100 3150
F 0 "J1" H 3100 3250 50  0000 C CNN
F 1 "slave1" H 3100 3050 50  0000 C CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 3100 3150 50  0001 C CNN
F 3 "" H 3100 3150 50  0000 C CNN
	1    3100 3150
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P2
U 1 1 578D1B02
P 3650 1650
F 0 "P2" H 3650 1800 50  0000 C CNN
F 1 "CONN_01X02" V 3750 1650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3650 1650 50  0001 C CNN
F 3 "" H 3650 1650 50  0000 C CNN
	1    3650 1650
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 rxtx1
U 1 1 578D2089
P 6400 4750
F 0 "rxtx1" H 6400 4900 50  0000 C CNN
F 1 "rtmaster" V 6500 4750 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02" H 6400 4750 50  0001 C CNN
F 3 "" H 6400 4750 50  0000 C CNN
	1    6400 4750
	0    1    1    0   
$EndComp
$Comp
L DIL18 uln1
U 1 1 578D2207
P 5400 5750
F 0 "uln1" H 5400 6300 50  0000 C CNN
F 1 "uln" H 5400 5200 50  0000 C CNN
F 2 "Housings_DIP:DIP-18_W7.62mm_LongPads" H 5400 5750 50  0001 C CNN
F 3 "" H 5400 5750 50  0000 C CNN
	1    5400 5750
	1    0    0    -1  
$EndComp
NoConn ~ 4850 3300
NoConn ~ 4850 3400
NoConn ~ 6050 2500
NoConn ~ 6050 2600
NoConn ~ 6050 2700
NoConn ~ 6050 2800
NoConn ~ 6050 2900
NoConn ~ 6050 3000
NoConn ~ 6050 3100
NoConn ~ 6050 3200
NoConn ~ 6050 3300
NoConn ~ 6050 3400
$Comp
L LED D12
U 1 1 578D41D2
P 6850 2550
F 0 "D12" H 6850 2650 50  0000 C CNN
F 1 "LED" H 6850 2450 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 2550 50  0001 C CNN
F 3 "" H 6850 2550 50  0000 C CNN
	1    6850 2550
	1    0    0    -1  
$EndComp
$Comp
L LED D13
U 1 1 578D425F
P 6850 2650
F 0 "D13" H 6850 2750 50  0000 C CNN
F 1 "LED" H 6850 2550 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 2650 50  0001 C CNN
F 3 "" H 6850 2650 50  0000 C CNN
	1    6850 2650
	1    0    0    -1  
$EndComp
$Comp
L LED D14
U 1 1 578D42DA
P 6850 2750
F 0 "D14" H 6850 2850 50  0000 C CNN
F 1 "LED" H 6850 2650 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 2750 50  0001 C CNN
F 3 "" H 6850 2750 50  0000 C CNN
	1    6850 2750
	1    0    0    -1  
$EndComp
$Comp
L LED D15
U 1 1 578D4337
P 6850 2850
F 0 "D15" H 6850 2950 50  0000 C CNN
F 1 "LED" H 6850 2750 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 2850 50  0001 C CNN
F 3 "" H 6850 2850 50  0000 C CNN
	1    6850 2850
	1    0    0    -1  
$EndComp
$Comp
L LED D16
U 1 1 578D4392
P 6850 2950
F 0 "D16" H 6850 3050 50  0000 C CNN
F 1 "LED" H 6850 2850 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 2950 50  0001 C CNN
F 3 "" H 6850 2950 50  0000 C CNN
	1    6850 2950
	1    0    0    -1  
$EndComp
$Comp
L LED D17
U 1 1 578D4407
P 6850 3050
F 0 "D17" H 6850 3150 50  0000 C CNN
F 1 "LED" H 6850 2950 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 6850 3050 50  0001 C CNN
F 3 "" H 6850 3050 50  0000 C CNN
	1    6850 3050
	1    0    0    -1  
$EndComp
$Comp
L LED D6
U 1 1 578D44A4
P 4550 2600
F 0 "D6" H 4550 2700 50  0000 C CNN
F 1 "dm1" H 4550 2500 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 2600 50  0001 C CNN
F 3 "" H 4550 2600 50  0000 C CNN
	1    4550 2600
	1    0    0    -1  
$EndComp
$Comp
L LED D7
U 1 1 578D4529
P 4550 2700
F 0 "D7" H 4550 2800 50  0000 C CNN
F 1 "dm2" H 4550 2600 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 2700 50  0001 C CNN
F 3 "" H 4550 2700 50  0000 C CNN
	1    4550 2700
	1    0    0    -1  
$EndComp
$Comp
L LED D8
U 1 1 578D45C2
P 4550 2800
F 0 "D8" H 4550 2900 50  0000 C CNN
F 1 "dm3" H 4550 2700 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 2800 50  0001 C CNN
F 3 "" H 4550 2800 50  0000 C CNN
	1    4550 2800
	1    0    0    -1  
$EndComp
$Comp
L LED D9
U 1 1 578D465D
P 4550 2900
F 0 "D9" H 4550 3000 50  0000 C CNN
F 1 "dm4" H 4550 2800 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 2900 50  0001 C CNN
F 3 "" H 4550 2900 50  0000 C CNN
	1    4550 2900
	1    0    0    -1  
$EndComp
$Comp
L LED D10
U 1 1 578D46B2
P 4550 3000
F 0 "D10" H 4550 3100 50  0000 C CNN
F 1 "dm5" H 4550 2900 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 3000 50  0001 C CNN
F 3 "" H 4550 3000 50  0000 C CNN
	1    4550 3000
	1    0    0    -1  
$EndComp
$Comp
L LED D11
U 1 1 578D4709
P 4550 3100
F 0 "D11" H 4550 3200 50  0000 C CNN
F 1 "dm6" H 4550 3000 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 4550 3100 50  0001 C CNN
F 3 "" H 4550 3100 50  0000 C CNN
	1    4550 3100
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 578D475E
P 2200 2600
F 0 "D1" H 2200 2700 50  0000 C CNN
F 1 "d11" H 2200 2500 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 2600 50  0001 C CNN
F 3 "" H 2200 2600 50  0000 C CNN
	1    2200 2600
	1    0    0    -1  
$EndComp
$Comp
L LED d11
U 1 1 578D47B9
P 2200 2700
F 0 "d11" H 2200 2800 50  0000 C CNN
F 1 "d12" H 2200 2600 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 2700 50  0001 C CNN
F 3 "" H 2200 2700 50  0000 C CNN
	1    2200 2700
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 578D4812
P 2200 2800
F 0 "D2" H 2200 2900 50  0000 C CNN
F 1 "d13" H 2200 2700 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 2800 50  0001 C CNN
F 3 "" H 2200 2800 50  0000 C CNN
	1    2200 2800
	1    0    0    -1  
$EndComp
$Comp
L LED D3
U 1 1 578D486D
P 2200 2900
F 0 "D3" H 2200 3000 50  0000 C CNN
F 1 "d14" H 2200 2800 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 2900 50  0001 C CNN
F 3 "" H 2200 2900 50  0000 C CNN
	1    2200 2900
	1    0    0    -1  
$EndComp
$Comp
L LED D4
U 1 1 578D48CA
P 2200 3000
F 0 "D4" H 2200 3100 50  0000 C CNN
F 1 "d15" H 2200 2900 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 3000 50  0001 C CNN
F 3 "" H 2200 3000 50  0000 C CNN
	1    2200 3000
	1    0    0    -1  
$EndComp
$Comp
L LED D5
U 1 1 578D4929
P 2200 3100
F 0 "D5" H 2200 3200 50  0000 C CNN
F 1 "d16" H 2200 3000 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 2200 3100 50  0001 C CNN
F 3 "" H 2200 3100 50  0000 C CNN
	1    2200 3100
	1    0    0    -1  
$EndComp
NoConn ~ 6050 3700
Wire Wire Line
	4850 2500 4600 2500
Wire Wire Line
	4600 2500 4600 2250
Wire Wire Line
	2950 2250 2500 2250
Wire Wire Line
	2500 2250 2500 2500
Wire Wire Line
	7050 2150 7050 2450
Wire Wire Line
	7050 2450 7200 2450
Wire Wire Line
	3600 1850 3600 2250
Wire Wire Line
	3250 2250 4300 2250
Connection ~ 3600 2250
Wire Wire Line
	6750 2150 3600 2150
Connection ~ 3600 2150
Wire Wire Line
	5750 6150 5850 6150
Wire Wire Line
	5850 6150 5850 6400
Wire Wire Line
	5850 6400 3950 6400
Wire Wire Line
	3950 6400 3950 2250
Connection ~ 3950 2250
Wire Wire Line
	4000 6150 5050 6150
Wire Wire Line
	4000 1850 4000 6150
Wire Wire Line
	4000 1850 3700 1850
Wire Wire Line
	3700 3800 3700 4300
Wire Wire Line
	3700 4300 8400 4300
Wire Wire Line
	4500 4300 4500 4500
Wire Wire Line
	6050 4300 6050 3800
Connection ~ 4500 4300
Wire Wire Line
	8400 4300 8400 3750
Connection ~ 6050 4300
Wire Wire Line
	7200 4250 7200 3750
Wire Wire Line
	2500 4250 7200 4250
Wire Wire Line
	4850 4250 4850 3800
Wire Wire Line
	5050 4500 5050 4250
Connection ~ 5050 4250
Wire Wire Line
	2500 4250 2500 3800
Connection ~ 4850 4250
Wire Wire Line
	3950 4800 5050 4800
Connection ~ 3950 4800
Connection ~ 4500 4800
Wire Wire Line
	6050 3600 6350 3600
Wire Wire Line
	6350 3600 6350 4550
Wire Wire Line
	6450 4550 6450 3500
Wire Wire Line
	6450 3500 6050 3500
Wire Wire Line
	2500 3700 2450 3700
Wire Wire Line
	2450 3700 2450 4200
Wire Wire Line
	2450 4200 4600 4200
Wire Wire Line
	4600 4200 4600 3600
Wire Wire Line
	4600 3600 4850 3600
Wire Wire Line
	4850 3700 4750 3700
Wire Wire Line
	4750 3700 4750 4200
Wire Wire Line
	4750 4200 7100 4200
Wire Wire Line
	7100 4200 7100 3650
Wire Wire Line
	7100 3650 7200 3650
Wire Wire Line
	4850 3200 4000 3200
Connection ~ 4000 3200
Connection ~ 4000 2050
Wire Wire Line
	6550 3150 7200 3150
Wire Wire Line
	2400 2600 2500 2600
Wire Wire Line
	2500 2700 2400 2700
Wire Wire Line
	2400 2800 2500 2800
Wire Wire Line
	2500 2900 2400 2900
Wire Wire Line
	2400 3000 2500 3000
Wire Wire Line
	2500 3100 2400 3100
Wire Wire Line
	2000 3200 2500 3200
Wire Wire Line
	2000 2050 2000 3200
Connection ~ 2000 3100
Connection ~ 2000 3000
Connection ~ 2000 2900
Connection ~ 2000 2800
Connection ~ 2000 2700
Wire Wire Line
	2000 2050 6550 2050
Connection ~ 2000 2600
Wire Wire Line
	4750 3100 4850 3100
Wire Wire Line
	4850 3000 4750 3000
Wire Wire Line
	4750 2900 4850 2900
Wire Wire Line
	4850 2800 4750 2800
Wire Wire Line
	4750 2700 4850 2700
Wire Wire Line
	4850 2600 4750 2600
Wire Wire Line
	4350 3100 4000 3100
Connection ~ 4000 3100
Wire Wire Line
	4000 3000 4350 3000
Connection ~ 4000 3000
Wire Wire Line
	4350 2900 4000 2900
Connection ~ 4000 2900
Wire Wire Line
	4000 2800 4350 2800
Connection ~ 4000 2800
Wire Wire Line
	4350 2700 4000 2700
Connection ~ 4000 2700
Wire Wire Line
	4350 2600 4000 2600
Connection ~ 4000 2600
Wire Wire Line
	6550 2050 6550 3150
Wire Wire Line
	6550 2550 6650 2550
Wire Wire Line
	6650 2650 6550 2650
Connection ~ 6550 2650
Wire Wire Line
	6550 2750 6650 2750
Connection ~ 6550 2750
Wire Wire Line
	6650 2850 6550 2850
Connection ~ 6550 2850
Wire Wire Line
	6550 2950 6650 2950
Connection ~ 6550 2950
Wire Wire Line
	6650 3050 6550 3050
Connection ~ 6550 3050
Wire Wire Line
	7050 2550 7200 2550
Wire Wire Line
	7200 2650 7050 2650
Wire Wire Line
	7050 2750 7200 2750
Wire Wire Line
	7200 2850 7050 2850
Wire Wire Line
	7050 2950 7200 2950
Wire Wire Line
	7200 3050 7050 3050
Connection ~ 6550 2550
Wire Wire Line
	4850 3500 4500 3500
Wire Wire Line
	4500 3500 4500 3750
Wire Wire Line
	4500 3750 4350 3750
Wire Wire Line
	4350 3650 4350 3450
Wire Wire Line
	4350 3450 4000 3450
Connection ~ 4000 3450
Wire Wire Line
	2500 3600 2400 3600
Wire Wire Line
	2400 3600 2400 4150
Wire Wire Line
	2400 4150 4350 4150
Wire Wire Line
	4350 4150 4350 3950
Wire Wire Line
	4350 3850 4450 3850
Wire Wire Line
	4450 3850 4450 4150
Wire Wire Line
	4450 4150 7000 4150
Wire Wire Line
	7000 4150 7000 3550
Wire Wire Line
	7000 3550 7200 3550
Wire Wire Line
	6800 5150 6800 3450
Wire Wire Line
	6800 3450 7200 3450
Wire Wire Line
	6700 5150 6700 3150
Connection ~ 6700 3150
Wire Wire Line
	6950 4700 6800 4700
Connection ~ 6800 4700
Wire Wire Line
	6950 5000 6700 5000
Connection ~ 6700 5000
Wire Wire Line
	2500 3500 2300 3500
Wire Wire Line
	2300 3500 2300 3600
Wire Wire Line
	2300 3600 1600 3600
Wire Wire Line
	1750 3400 1750 3600
Connection ~ 1750 3600
Wire Wire Line
	1600 3500 1600 3100
Wire Wire Line
	1600 3100 1750 3100
Wire Wire Line
	1750 3100 1750 2550
Wire Wire Line
	1750 2550 2000 2550
Connection ~ 2000 2550
Wire Wire Line
	5750 5350 6450 5350
Wire Wire Line
	6450 5350 6450 5700
Wire Wire Line
	6450 5700 6950 5700
Wire Wire Line
	6400 5800 7000 5800
Wire Wire Line
	6400 5800 6400 5450
Wire Wire Line
	6400 5450 5750 5450
Wire Wire Line
	5750 5550 6350 5550
Wire Wire Line
	6350 5550 6350 5900
Wire Wire Line
	6350 5900 7050 5900
Wire Wire Line
	6300 6000 7100 6000
Wire Wire Line
	6300 6000 6300 5650
Wire Wire Line
	6300 5650 5750 5650
Wire Wire Line
	5750 5750 6200 5750
Wire Wire Line
	6200 5750 6200 6200
Wire Wire Line
	6200 6200 6450 6200
Wire Wire Line
	6100 6300 6500 6300
Wire Wire Line
	6100 6300 6100 5850
Wire Wire Line
	6100 5850 5750 5850
Wire Wire Line
	5750 5950 6000 5950
Wire Wire Line
	6000 5950 6000 6400
Wire Wire Line
	6000 6400 6550 6400
Wire Wire Line
	5750 6050 5900 6050
Wire Wire Line
	5900 6050 5900 6500
Wire Wire Line
	5900 6500 6600 6500
Wire Wire Line
	3700 2900 3900 2900
Wire Wire Line
	3900 2900 3900 5350
Wire Wire Line
	3900 5350 5050 5350
Wire Wire Line
	5050 5450 3850 5450
Wire Wire Line
	3850 5450 3850 3000
Wire Wire Line
	3850 3000 3700 3000
Wire Wire Line
	3700 3100 3800 3100
Wire Wire Line
	3800 3100 3800 5550
Wire Wire Line
	3800 5550 5050 5550
Wire Wire Line
	5050 5650 3750 5650
Wire Wire Line
	3750 5650 3750 3200
Wire Wire Line
	3750 3200 3700 3200
Wire Wire Line
	8400 3150 8450 3150
Wire Wire Line
	8450 3150 8450 4350
Wire Wire Line
	8450 4350 4050 4350
Wire Wire Line
	4050 4350 4050 6050
Wire Wire Line
	4050 6050 5050 6050
Wire Wire Line
	5050 5950 4100 5950
Wire Wire Line
	4100 5950 4100 4400
Wire Wire Line
	4100 4400 8500 4400
Wire Wire Line
	8500 4400 8500 3050
Wire Wire Line
	8500 3050 8400 3050
Wire Wire Line
	8400 2950 8550 2950
Wire Wire Line
	8550 2950 8550 4450
Wire Wire Line
	8550 4450 4150 4450
Wire Wire Line
	4150 4450 4150 5850
Wire Wire Line
	4150 5850 5050 5850
Wire Wire Line
	5050 5750 4200 5750
Wire Wire Line
	4200 5750 4200 4500
Wire Wire Line
	4200 4500 8600 4500
Wire Wire Line
	8600 4500 8600 2850
Wire Wire Line
	8600 2850 8400 2850
NoConn ~ 2500 3300
NoConn ~ 2500 3400
NoConn ~ 3700 3700
NoConn ~ 3700 3600
NoConn ~ 3700 3500
NoConn ~ 3700 3400
NoConn ~ 3700 3300
NoConn ~ 3700 2500
NoConn ~ 3700 2600
NoConn ~ 3700 2700
NoConn ~ 3700 2800
NoConn ~ 7200 3250
NoConn ~ 7200 3350
NoConn ~ 8400 2450
NoConn ~ 8400 2550
NoConn ~ 8400 2650
NoConn ~ 8400 2750
NoConn ~ 8400 3250
NoConn ~ 8400 3350
NoConn ~ 8400 3450
NoConn ~ 8400 3550
NoConn ~ 8400 3650
$Comp
L CONN_01X06 P3
U 1 1 578DECF1
P 7400 5550
F 0 "P3" H 7400 5900 50  0000 C CNN
F 1 "pap1" V 7500 5550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7400 5550 50  0001 C CNN
F 3 "" H 7400 5550 50  0000 C CNN
	1    7400 5550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X06 P4
U 1 1 578DED78
P 7400 6200
F 0 "P4" H 7400 6550 50  0000 C CNN
F 1 "pap2" V 7500 6200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7400 6200 50  0001 C CNN
F 3 "" H 7400 6200 50  0000 C CNN
	1    7400 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 5700 6950 5300
Wire Wire Line
	6950 5300 7200 5300
Wire Wire Line
	7000 5800 7000 5400
Wire Wire Line
	7000 5400 7200 5400
Wire Wire Line
	7050 5900 7050 5500
Wire Wire Line
	7050 5500 7200 5500
Wire Wire Line
	7100 6000 7100 5600
Wire Wire Line
	7100 5600 7200 5600
Wire Wire Line
	7200 5700 7200 5850
Wire Wire Line
	7200 5850 6250 5850
Wire Wire Line
	6250 5850 6250 6250
Wire Wire Line
	6250 6250 5850 6250
Connection ~ 5850 6250
Connection ~ 7200 5800
Wire Wire Line
	7200 5950 6450 5950
Wire Wire Line
	6450 5950 6450 6200
Wire Wire Line
	6500 6300 6500 6050
Wire Wire Line
	6500 6050 7200 6050
Wire Wire Line
	6550 6400 6550 6150
Wire Wire Line
	6550 6150 7200 6150
Wire Wire Line
	7200 6250 6600 6250
Wire Wire Line
	6600 6250 6600 6500
Wire Wire Line
	5850 6350 7200 6350
Connection ~ 5850 6350
Wire Wire Line
	7200 6350 7200 6450
$EndSCHEMATC
