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
LIBS:SHT31-DIS-B
LIBS:mysensors_sensors
LIBS:ms5637
LIBS:Sht31x2-cache
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
L SHT31-DIS-B U3
U 1 1 583FF3EB
P 4000 1500
F 0 "U3" H 3600 1820 50  0000 L CNN
F 1 "SHT31-DIS-B" H 3600 1100 50  0000 L CNN
F 2 "SON50P250X250X100-9N" H 4000 1500 50  0001 L CNN
F 3 "SHT31 Series 5.5 V 800 µA Surface Mount Humidity and Temperature Sensor" H 4000 1500 50  0001 L CNN
F 4 "Good" H 4000 1500 50  0001 L CNN "Availability"
F 5 "Sensirion" H 4000 1500 50  0001 L CNN "MF"
F 6 "TDFN-8 Sensirion" H 4000 1500 50  0001 L CNN "Package"
F 7 "4.74 USD" H 4000 1500 50  0001 L CNN "Price"
F 8 "SHT31-DIS-B" H 4000 1500 50  0001 L CNN "MP"
	1    4000 1500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR4
U 1 1 583FF4F0
P 2800 1200
F 0 "#PWR4" H 2800 1050 50  0001 C CNN
F 1 "+3.3V" H 2800 1340 50  0000 C CNN
F 2 "" H 2800 1200 50  0000 C CNN
F 3 "" H 2800 1200 50  0000 C CNN
	1    2800 1200
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 583FF529
P 2800 1400
F 0 "R1" V 2880 1400 50  0000 C CNN
F 1 "10k" V 2800 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2730 1400 50  0000 C CNN
F 3 "" H 2800 1400 50  0000 C CNN
	1    2800 1400
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 583FF5B0
P 2950 1400
F 0 "R2" V 3030 1400 50  0000 C CNN
F 1 "10k" V 2950 1400 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 2880 1400 50  0000 C CNN
F 3 "" H 2950 1400 50  0000 C CNN
	1    2950 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 583FF69A
P 4600 1750
F 0 "#PWR10" H 4600 1500 50  0001 C CNN
F 1 "GND" H 4600 1600 50  0000 C CNN
F 2 "" H 4600 1750 50  0000 C CNN
F 3 "" H 4600 1750 50  0000 C CNN
	1    4600 1750
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR9
U 1 1 583FF6C0
P 4600 1250
F 0 "#PWR9" H 4600 1100 50  0001 C CNN
F 1 "+3.3V" H 4600 1390 50  0000 C CNN
F 2 "" H 4600 1250 50  0000 C CNN
F 3 "" H 4600 1250 50  0000 C CNN
	1    4600 1250
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 583FF728
P 4900 1450
F 0 "C3" H 4925 1550 50  0000 L CNN
F 1 "100nF" H 4925 1350 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4938 1300 50  0000 C CNN
F 3 "" H 4900 1450 50  0000 C CNN
	1    4900 1450
	-1   0    0    1   
$EndComp
$Comp
L +3.3V #PWR8
U 1 1 583FF8CF
P 4500 2200
F 0 "#PWR8" H 4500 2050 50  0001 C CNN
F 1 "+3.3V" H 4500 2340 50  0000 C CNN
F 2 "" H 4500 2200 50  0000 C CNN
F 3 "" H 4500 2200 50  0000 C CNN
	1    4500 2200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR12
U 1 1 583FFC10
P 6100 2100
F 0 "#PWR12" H 6100 1850 50  0001 C CNN
F 1 "GND" H 6100 1950 50  0000 C CNN
F 2 "" H 6100 2100 50  0000 C CNN
F 3 "" H 6100 2100 50  0000 C CNN
	1    6100 2100
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 583FFF27
P 6000 700
F 0 "C5" H 6025 800 50  0000 L CNN
F 1 "C" H 6025 600 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6038 550 50  0000 C CNN
F 3 "" H 6000 700 50  0000 C CNN
	1    6000 700 
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 583FFF7C
P 6500 700
F 0 "C6" H 6525 800 50  0000 L CNN
F 1 "C" H 6525 600 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6538 550 50  0000 C CNN
F 3 "" H 6500 700 50  0000 C CNN
	1    6500 700 
	0    1    1    0   
$EndComp
$Comp
L LED D1
U 1 1 584004CF
P 3400 2650
F 0 "D1" H 3400 2750 50  0000 C CNN
F 1 "LED" H 3400 2550 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 3400 2650 50  0000 C CNN
F 3 "" H 3400 2650 50  0000 C CNN
	1    3400 2650
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 58400640
P 3400 2250
F 0 "R3" V 3480 2250 50  0000 C CNN
F 1 "1k" V 3400 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_1206" V 3330 2250 50  0000 C CNN
F 3 "" H 3400 2250 50  0000 C CNN
	1    3400 2250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 58400742
P 3400 2850
F 0 "#PWR6" H 3400 2600 50  0001 C CNN
F 1 "GND" H 3400 2700 50  0000 C CNN
F 2 "" H 3400 2850 50  0000 C CNN
F 3 "" H 3400 2850 50  0000 C CNN
	1    3400 2850
	1    0    0    -1  
$EndComp
$Comp
L LM2931AZ-3.3/5.0 U2
U 1 1 58400911
P 3750 750
F 0 "U2" H 3750 1000 50  0000 C CNN
F 1 "LM2931AZ-3.3/5.0" H 3750 950 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 3750 850 50  0000 C CIN
F 3 "" H 3750 750 50  0000 C CNN
	1    3750 750 
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 58400AC7
P 3150 850
F 0 "C1" H 3175 950 50  0000 L CNN
F 1 "0.1uF" H 3175 750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3188 700 50  0000 C CNN
F 3 "" H 3150 850 50  0000 C CNN
	1    3150 850 
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 58402594
P 4800 2600
F 0 "C2" H 4825 2700 50  0000 L CNN
F 1 "100nF" H 4825 2500 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4838 2450 50  0000 C CNN
F 3 "" H 4800 2600 50  0000 C CNN
	1    4800 2600
	-1   0    0    1   
$EndComp
$Comp
L C C4
U 1 1 58402955
P 5800 2200
F 0 "C4" H 5825 2300 50  0000 L CNN
F 1 "100nF" H 5825 2100 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5838 2050 50  0000 C CNN
F 3 "" H 5800 2200 50  0000 C CNN
	1    5800 2200
	0    -1   -1   0   
$EndComp
$Comp
L C C7
U 1 1 58402ABA
P 6500 2150
F 0 "C7" H 6525 2250 50  0000 L CNN
F 1 "100nF" H 6525 2050 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6538 2000 50  0000 C CNN
F 3 "" H 6500 2150 50  0000 C CNN
	1    6500 2150
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR5
U 1 1 5840366F
P 3200 1350
F 0 "#PWR5" H 3200 1200 50  0001 C CNN
F 1 "+3.3V" H 3200 1490 50  0000 C CNN
F 2 "" H 3200 1350 50  0000 C CNN
F 3 "" H 3200 1350 50  0000 C CNN
	1    3200 1350
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR3
U 1 1 584051AE
P 2000 1700
F 0 "#PWR3" H 2000 1450 50  0001 C CNN
F 1 "GND" H 2000 1550 50  0000 C CNN
F 2 "" H 2000 1700 50  0000 C CNN
F 3 "" H 2000 1700 50  0000 C CNN
	1    2000 1700
	1    0    0    -1  
$EndComp
$Comp
L OPT3001 U1
U 1 1 5847DA4D
P 2000 1250
F 0 "U1" H 2150 1550 60  0000 C CNN
F 1 "OPT3001" H 2200 950 60  0000 C CNN
F 2 "uson-6:USON-6" H 2000 1250 60  0000 C CNN
F 3 "" H 2000 1250 60  0000 C CNN
	1    2000 1250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR11
U 1 1 586BA182
P 4750 3750
F 0 "#PWR11" H 4750 3500 50  0001 C CNN
F 1 "GND" H 4750 3600 50  0000 C CNN
F 2 "" H 4750 3750 50  0000 C CNN
F 3 "" H 4750 3750 50  0000 C CNN
	1    4750 3750
	0    -1   -1   0   
$EndComp
$Comp
L MAX232 U5
U 1 1 583FFBAA
P 6000 1450
F 0 "U5" H 5550 2300 50  0000 L CNN
F 1 "MAX232" H 6200 2300 50  0000 L CNN
F 2 "SMD_Packages:SO-16-N" H 6000 1450 50  0000 C CNN
F 3 "" H 6000 1450 50  0000 C CNN
	1    6000 1450
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 586CD9CD
P 7000 2350
F 0 "C8" H 7025 2450 50  0000 L CNN
F 1 "100nF" H 7025 2250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 7038 2200 50  0000 C CNN
F 3 "" H 7000 2350 50  0000 C CNN
	1    7000 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 586CDA4D
P 7000 2550
F 0 "#PWR13" H 7000 2300 50  0001 C CNN
F 1 "GND" H 7000 2400 50  0000 C CNN
F 2 "" H 7000 2550 50  0000 C CNN
F 3 "" H 7000 2550 50  0000 C CNN
	1    7000 2550
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 P1
U 1 1 586E1FC5
P 4400 3950
F 0 "P1" H 4400 4250 50  0000 C CNN
F 1 "CONN_02X05" H 4400 3650 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x05" H 4400 2750 50  0000 C CNN
F 3 "" H 4400 2750 50  0000 C CNN
	1    4400 3950
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR7
U 1 1 586E4EBE
P 4100 3850
F 0 "#PWR7" H 4100 3700 50  0001 C CNN
F 1 "+3.3V" H 4100 3990 50  0000 C CNN
F 2 "" H 4100 3850 50  0000 C CNN
F 3 "" H 4100 3850 50  0000 C CNN
	1    4100 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 1250 2800 1250
Wire Wire Line
	2800 1250 2800 1200
Wire Wire Line
	2950 1550 2950 1600
Wire Wire Line
	2650 1600 3400 1600
Wire Wire Line
	2800 1550 2800 1700
Wire Wire Line
	2600 1700 3400 1700
Wire Wire Line
	4600 1600 4600 1750
Wire Wire Line
	4600 1250 4600 1300
Connection ~ 4600 1700
Wire Wire Line
	4600 1300 4900 1300
Wire Wire Line
	4900 1600 4900 1700
Wire Wire Line
	4900 1700 4600 1700
Wire Wire Line
	4500 2200 4500 2250
Wire Wire Line
	4500 3000 4500 2950
Wire Wire Line
	3200 1600 3200 3150
Wire Wire Line
	3200 3150 3800 3150
Wire Wire Line
	3800 3150 3800 2950
Connection ~ 3200 1600
Wire Wire Line
	3600 2950 3250 2950
Wire Wire Line
	3250 2950 3250 1700
Connection ~ 3250 1700
Wire Wire Line
	6100 2100 6100 2050
Wire Wire Line
	5800 850  5800 700 
Wire Wire Line
	5800 700  5850 700 
Wire Wire Line
	6150 700  6200 700 
Wire Wire Line
	6200 700  6200 850 
Wire Wire Line
	6300 850  6300 700 
Wire Wire Line
	6300 700  6350 700 
Wire Wire Line
	6650 700  6700 700 
Wire Wire Line
	6700 700  6700 850 
Wire Wire Line
	3900 2250 3900 2100
Wire Wire Line
	3900 2100 3400 2100
Wire Wire Line
	3400 2400 3400 2450
Wire Wire Line
	3750 1000 3450 1000
Wire Wire Line
	3350 700  3350 1050
Wire Wire Line
	3350 1050 3950 1050
Wire Wire Line
	3950 1050 3950 1000
Wire Wire Line
	3950 1000 4650 1000
Wire Wire Line
	4150 700  4350 700 
Wire Wire Line
	3350 700  3150 700 
Wire Wire Line
	3150 1000 3150 1100
Wire Wire Line
	3150 1100 3450 1100
Wire Wire Line
	5950 2600 5950 2550
Wire Wire Line
	4500 2250 4800 2250
Wire Wire Line
	4800 2250 4800 2450
Wire Wire Line
	4800 2950 4800 2750
Connection ~ 4500 2950
Wire Wire Line
	5800 2050 5650 2050
Wire Wire Line
	5650 2050 5650 2200
Wire Wire Line
	5950 2200 5950 2100
Wire Wire Line
	5950 2100 6100 2100
Wire Wire Line
	6300 2050 6300 2150
Wire Wire Line
	6300 2150 6350 2150
Wire Wire Line
	6650 2150 7050 2150
Wire Wire Line
	3400 1400 3200 1400
Wire Wire Line
	3200 1400 3200 1350
Wire Wire Line
	4350 700  4350 500 
Wire Wire Line
	4350 500  7050 500 
Wire Wire Line
	7050 500  7050 2150
Connection ~ 6700 2150
Wire Wire Line
	6700 2450 6650 2450
Wire Wire Line
	2400 1250 2650 1250
Wire Wire Line
	2650 1250 2650 2100
Connection ~ 2950 1600
Wire Wire Line
	2400 1150 2600 1150
Wire Wire Line
	2600 1150 2600 2200
Connection ~ 2800 1700
Connection ~ 3250 1400
Wire Wire Line
	2000 1700 2000 1600
Wire Wire Line
	4650 1000 4650 1150
Wire Wire Line
	4650 1150 5000 1150
Wire Wire Line
	5000 1150 5000 3850
Wire Wire Line
	5400 3850 5400 2050
Wire Wire Line
	3700 2950 3700 3300
Wire Wire Line
	3700 3300 5150 3300
Wire Wire Line
	5150 3300 5150 750 
Wire Wire Line
	5150 750  5400 750 
Wire Wire Line
	5400 750  5400 850 
Wire Wire Line
	3600 2250 3600 1950
Wire Wire Line
	3600 1950 5050 1950
Wire Wire Line
	5050 1950 5050 650 
Wire Wire Line
	5050 650  5600 650 
Wire Wire Line
	5600 650  5600 850 
Wire Wire Line
	5600 2050 5600 4050
Wire Wire Line
	7000 2150 7000 2200
Connection ~ 7000 2150
Wire Wire Line
	7000 2500 7000 2550
Wire Wire Line
	1600 1250 1600 1650
Wire Wire Line
	1400 1650 2000 1650
Connection ~ 2000 1650
Wire Wire Line
	3450 1100 3450 1000
Wire Wire Line
	3400 1150 3400 1300
Wire Wire Line
	3400 1150 4500 1150
Wire Wire Line
	4500 1150 4500 1250
Wire Wire Line
	4500 1250 4600 1250
Wire Wire Line
	6700 2150 6700 2050
Wire Wire Line
	4500 2950 4800 2950
$Comp
L DIL20 U4
U 1 1 583FF807
P 4050 2600
F 0 "U4" H 4050 3150 50  0000 C CNN
F 1 "PIC16F1508" V 4050 2600 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-20_7.5x12.8mm_Pitch1.27mm" H 4050 2600 50  0000 C CNN
F 3 "" H 4050 2600 50  0000 C CNN
	1    4050 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 3750 4750 3750
Wire Wire Line
	4700 3750 4700 3700
Wire Wire Line
	4700 3700 4150 3700
Wire Wire Line
	4150 3700 4150 3750
Connection ~ 4700 3750
Wire Wire Line
	4100 3850 4150 3850
Wire Wire Line
	5000 3850 4650 3850
Wire Wire Line
	3700 2250 3700 2000
Wire Wire Line
	3700 2000 3000 2000
Wire Wire Line
	4300 2950 4300 3600
Wire Wire Line
	4300 3600 3900 3600
Wire Wire Line
	3900 3600 3900 3950
Wire Wire Line
	3900 3950 4150 3950
Wire Wire Line
	4150 4050 3750 4050
Wire Wire Line
	3750 4050 3750 3450
Wire Wire Line
	3750 3450 4400 3450
Wire Wire Line
	4400 3450 4400 2950
Wire Wire Line
	4200 2250 4200 1900
Wire Wire Line
	4200 1900 2700 1900
Wire Wire Line
	2700 1900 2700 4150
Wire Wire Line
	2700 4150 4150 4150
Wire Wire Line
	3000 2000 3000 4300
Wire Wire Line
	3000 4300 4650 4300
Wire Wire Line
	4650 4300 4650 4150
Wire Wire Line
	5400 3850 5200 3850
Wire Wire Line
	5200 3850 5200 3950
Wire Wire Line
	5200 3950 4650 3950
Wire Wire Line
	5600 4050 4650 4050
$Comp
L MS5637 U7
U 1 1 586F781B
P 2000 2150
F 0 "U7" V 2050 2150 60  0000 C CNN
F 1 "MS5637" V 1950 2150 60  0000 C CNN
F 2 "KiC:MS563702" H 1900 2300 60  0000 C CNN
F 3 "" H 1900 2300 60  0000 C CNN
	1    2000 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2200 2500 2200
Connection ~ 2600 1700
Wire Wire Line
	2650 2100 2500 2100
Connection ~ 2650 1600
Wire Wire Line
	1500 2200 1400 2200
Wire Wire Line
	1400 2200 1400 1650
Connection ~ 1600 1650
Connection ~ 2000 900 
$Comp
L +3.3V #PWR2
U 1 1 586F7CE7
P 2000 800
F 0 "#PWR2" H 2000 650 50  0001 C CNN
F 1 "+3.3V" H 2000 940 50  0000 C CNN
F 2 "" H 2000 800 50  0000 C CNN
F 3 "" H 2000 800 50  0000 C CNN
	1    2000 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 800  2000 900 
$Comp
L +3.3V #PWR1
U 1 1 586F7E60
P 1300 2100
F 0 "#PWR1" H 1300 1950 50  0001 C CNN
F 1 "+3.3V" H 1300 2240 50  0000 C CNN
F 2 "" H 1300 2100 50  0000 C CNN
F 3 "" H 1300 2100 50  0000 C CNN
	1    1300 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 2100 1500 2100
$EndSCHEMATC
