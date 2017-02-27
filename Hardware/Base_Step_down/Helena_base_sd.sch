EESchema Schematic File Version 2
LIBS:Helena_base_sd-rescue
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
LIBS:Helena_base_sd-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Helena Dual 3.0A Step Down Converter"
Date "2017-02-27"
Rev "2.2"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_01X01 P3
U 1 1 58074D5A
P 1200 5400
F 0 "P3" H 1200 5500 50  0000 C CNN
F 1 "CONN_01X01" V 1300 5400 50  0000 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 1200 5400 60  0001 C CNN
F 3 "" H 1200 5400 60  0000 C CNN
	1    1200 5400
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P2
U 1 1 58074E0D
P 1200 5200
F 0 "P2" H 1200 5300 50  0000 C CNN
F 1 "CONN_01X01" V 1300 5200 50  0000 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 1200 5200 60  0001 C CNN
F 3 "" H 1200 5200 60  0000 C CNN
	1    1200 5200
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P1
U 1 1 58074E88
P 1200 5000
F 0 "P1" H 1200 5100 50  0000 C CNN
F 1 "CONN_01X01" V 1300 5000 50  0000 C CNN
F 2 "rt_pins:1,5x3mm_SMD" H 1200 5000 60  0001 C CNN
F 3 "" H 1200 5000 60  0000 C CNN
	1    1200 5000
	-1   0    0    1   
$EndComp
$Comp
L ZENER_3pin_KNCA D2
U 1 1 5807502A
P 2900 5500
F 0 "D2" H 2900 5600 50  0000 C CNN
F 1 "18V" H 2900 5400 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 2900 5500 60  0001 C CNN
F 3 "" H 2900 5500 60  0000 C CNN
	1    2900 5500
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 58075085
P 2900 6100
F 0 "#PWR01" H 2900 5850 50  0001 C CNN
F 1 "GND" H 2900 5950 50  0000 C CNN
F 2 "" H 2900 6100 60  0000 C CNN
F 3 "" H 2900 6100 60  0000 C CNN
	1    2900 6100
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR02
U 1 1 580750A3
P 1500 4600
F 0 "#PWR02" H 1500 4450 50  0001 C CNN
F 1 "VPP" H 1500 4750 50  0000 C CNN
F 2 "" H 1500 4600 60  0000 C CNN
F 3 "" H 1500 4600 60  0000 C CNN
	1    1500 4600
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q2
U 1 1 580750D0
P 3200 5100
F 0 "Q2" H 3500 5150 50  0000 R CNN
F 1 "BSS138" H 3850 5050 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 3400 5200 29  0001 C CNN
F 3 "" H 3200 5100 60  0000 C CNN
	1    3200 5100
	0    -1   1    0   
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 580751BD
P 3200 4700
F 0 "#PWR03" H 3200 4550 50  0001 C CNN
F 1 "+3.3V" H 3200 4840 50  0000 C CNN
F 2 "" H 3200 4700 60  0000 C CNN
F 3 "" H 3200 4700 60  0000 C CNN
	1    3200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5000 1500 5000
Wire Wire Line
	1500 5000 1500 4600
$Comp
L R R3
U 1 1 58075165
P 3450 4800
F 0 "R3" V 3530 4800 50  0000 C CNN
F 1 "4k7" V 3450 4800 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 3380 4800 30  0001 C CNN
F 3 "" H 3450 4800 30  0000 C CNN
	1    3450 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 5300 2900 5200
Wire Wire Line
	1400 5200 2500 5200
Connection ~ 2900 5200
Wire Wire Line
	2800 5200 3000 5200
$Comp
L R R2
U 1 1 58074F69
P 2650 5200
F 0 "R2" V 2730 5200 50  0000 C CNN
F 1 "10" V 2650 5200 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 2580 5200 30  0001 C CNN
F 3 "" H 2650 5200 30  0000 C CNN
	1    2650 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 5700 2900 6100
Wire Wire Line
	3200 4700 3200 4900
Wire Wire Line
	3300 4800 3200 4800
Connection ~ 3200 4800
Wire Wire Line
	3400 5200 3800 5200
Wire Wire Line
	3700 5200 3700 4800
Wire Wire Line
	3700 4800 3600 4800
Connection ~ 3700 5200
$Comp
L ATTINY85-S-RESCUE-Helena_base_sd IC1
U 1 1 580753C0
P 5950 5450
F 0 "IC1" H 4800 5850 40  0000 C CNN
F 1 "ATTINY85-S" H 6950 5050 40  0000 C CNN
F 2 "rt_Housings_SOIC:SOIC-8_5.29x4.9mm_Pitch1.27mm" H 6900 5450 35  0001 C CIN
F 3 "" H 5950 5450 60  0000 C CNN
	1    5950 5450
	-1   0    0    -1  
$EndComp
$Comp
L FB FB1
U 1 1 58075527
P 4500 4950
F 0 "FB1" V 4580 4950 50  0000 C CNN
F 1 "FB" V 4425 4950 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 4430 4950 30  0001 C CNN
F 3 "" H 4490 4950 30  0000 C CNN
	1    4500 4950
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5807558A
P 4500 5450
F 0 "C4" H 4525 5550 50  0000 L CNN
F 1 "100n" H 4525 5350 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 4538 5300 30  0001 C CNN
F 3 "" H 4500 5450 60  0000 C CNN
	1    4500 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 580756B9
P 4500 5800
F 0 "#PWR04" H 4500 5550 50  0001 C CNN
F 1 "GND" H 4500 5650 50  0000 C CNN
F 2 "" H 4500 5800 60  0000 C CNN
F 3 "" H 4500 5800 60  0000 C CNN
	1    4500 5800
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 580756E2
P 4500 4700
F 0 "#PWR05" H 4500 4550 50  0001 C CNN
F 1 "+3.3V" H 4500 4840 50  0000 C CNN
F 2 "" H 4500 4700 60  0000 C CNN
F 3 "" H 4500 4700 60  0000 C CNN
	1    4500 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 4700 4500 4800
Wire Wire Line
	4500 5100 4500 5300
Wire Wire Line
	4600 5200 4500 5200
Connection ~ 4500 5200
Wire Wire Line
	4500 5600 4500 5800
Wire Wire Line
	4600 5700 4500 5700
Connection ~ 4500 5700
Text Label 3800 5200 0    60   ~ 0
BDEP_RX
$Comp
L MCP1703 U2
U 1 1 58075CC0
P 8500 5250
F 0 "U2" H 8750 5450 60  0000 C CNN
F 1 "MCP1703CB" H 8450 5450 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 8400 5700 60  0001 C CNN
F 3 "" H 8400 5700 60  0000 C CNN
	1    8500 5250
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 58075E03
P 8000 5450
F 0 "C10" H 8025 5550 50  0000 L CNN
F 1 "2u2/16" H 8025 5350 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 8038 5300 30  0001 C CNN
F 3 "" H 8000 5450 60  0000 C CNN
	1    8000 5450
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 58075FF1
P 9000 5450
F 0 "C12" H 9025 5550 50  0000 L CNN
F 1 "2u2/16" H 9025 5350 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 9038 5300 30  0001 C CNN
F 3 "" H 9000 5450 60  0000 C CNN
	1    9000 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5807605E
P 8500 5800
F 0 "#PWR06" H 8500 5550 50  0001 C CNN
F 1 "GND" H 8500 5650 50  0000 C CNN
F 2 "" H 8500 5800 60  0000 C CNN
F 3 "" H 8500 5800 60  0000 C CNN
	1    8500 5800
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR07
U 1 1 5807613C
P 8000 5100
F 0 "#PWR07" H 8000 4950 50  0001 C CNN
F 1 "VPP" H 8000 5250 50  0000 C CNN
F 2 "" H 8000 5100 60  0000 C CNN
F 3 "" H 8000 5100 60  0000 C CNN
	1    8000 5100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR08
U 1 1 580761E3
P 9000 5100
F 0 "#PWR08" H 9000 4950 50  0001 C CNN
F 1 "+3.3V" H 9000 5240 50  0000 C CNN
F 2 "" H 9000 5100 60  0000 C CNN
F 3 "" H 9000 5100 60  0000 C CNN
	1    9000 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 5600 8000 5700
Wire Wire Line
	8000 5700 9000 5700
Wire Wire Line
	9000 5700 9000 5600
Wire Wire Line
	8500 5600 8500 5800
Connection ~ 8500 5700
Wire Wire Line
	8000 5100 8000 5300
Wire Wire Line
	8050 5250 8000 5250
Connection ~ 8000 5250
Wire Wire Line
	9000 5100 9000 5300
Wire Wire Line
	8950 5250 9000 5250
Connection ~ 9000 5250
$Comp
L MCP6482 U1
U 2 1 58076CE2
P 9100 2200
F 0 "U1" H 9050 2400 60  0000 L CNN
F 1 "MCP6482" H 9050 1950 60  0000 L CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 9100 2200 60  0001 C CNN
F 3 "" H 9100 2200 60  0000 C CNN
	2    9100 2200
	1    0    0    1   
$EndComp
$Comp
L MCP6482 U1
U 1 1 58076E4A
P 2500 2200
F 0 "U1" H 2450 2400 60  0000 L CNN
F 1 "MCP6482" H 2450 1950 60  0000 L CNN
F 2 "Housings_SSOP:MSOP-8_3x3mm_Pitch0.65mm" H 2500 2200 60  0001 C CNN
F 3 "" H 2500 2200 60  0000 C CNN
	1    2500 2200
	-1   0    0    1   
$EndComp
$Comp
L Q_PNP_BEC Q1
U 1 1 58077258
P 1600 2200
F 0 "Q1" H 1900 2250 50  0000 R CNN
F 1 "BC857C" H 2200 2150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 1800 2300 29  0001 C CNN
F 3 "" H 1600 2200 60  0000 C CNN
	1    1600 2200
	-1   0    0    1   
$EndComp
$Comp
L R R4
U 1 1 5807746C
P 3350 1600
F 0 "R4" V 3430 1600 50  0000 C CNN
F 1 "680" V 3350 1600 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 3280 1600 30  0001 C CNN
F 3 "" H 3350 1600 30  0000 C CNN
	1    3350 1600
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 580775E8
P 3600 1950
F 0 "C3" H 3625 2050 50  0000 L CNN
F 1 "100n" H 3625 1850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 3638 1800 30  0001 C CNN
F 3 "" H 3600 1950 60  0000 C CNN
	1    3600 1950
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5807779D
P 3850 2300
F 0 "R5" V 3930 2300 50  0000 C CNN
F 1 "1k5" V 3850 2300 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 3780 2300 30  0001 C CNN
F 3 "" H 3850 2300 30  0000 C CNN
	1    3850 2300
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 58077927
P 1900 1850
F 0 "C2" H 1925 1950 50  0000 L CNN
F 1 "100n" H 1925 1750 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1938 1700 30  0001 C CNN
F 3 "" H 1900 1850 60  0000 C CNN
	1    1900 1850
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR09
U 1 1 580779B6
P 2600 2600
F 0 "#PWR09" H 2600 2450 50  0001 C CNN
F 1 "VPP" H 2600 2750 50  0000 C CNN
F 2 "" H 2600 2600 60  0000 C CNN
F 3 "" H 2600 2600 60  0000 C CNN
	1    2600 2600
	1    0    0    1   
$EndComp
Wire Wire Line
	1500 1600 3200 1600
Wire Wire Line
	1500 1600 1500 2000
Wire Wire Line
	1900 1700 1900 1600
Connection ~ 1900 1600
Wire Wire Line
	1800 2200 2000 2200
Wire Wire Line
	1900 2000 1900 2200
Connection ~ 1900 2200
Wire Wire Line
	3500 1600 8100 1600
Wire Wire Line
	3600 1600 3600 1800
Wire Wire Line
	3000 2300 3700 2300
Wire Wire Line
	3600 2300 3600 2100
Connection ~ 3600 2300
Wire Wire Line
	3000 2100 3100 2100
Wire Wire Line
	3100 2100 3100 1600
Connection ~ 3100 1600
$Comp
L INDUCTOR_SMALL L1
U 1 1 5807818C
P 4100 1950
F 0 "L1" H 4100 2050 50  0000 C CNN
F 1 "3u-18" H 4100 1900 50  0000 C CNN
F 2 "rt_chokes:Choke_SMD_Fastron_242418_FPS" H 4100 1950 60  0001 C CNN
F 3 "" H 4100 1950 60  0000 C CNN
	1    4100 1950
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 58078265
P 4100 2650
F 0 "C5" H 4125 2750 50  0000 L CNN
F 1 "10u/16" H 4125 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4138 2500 30  0001 C CNN
F 3 "" H 4100 2650 60  0000 C CNN
	1    4100 2650
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q3
U 1 1 58078302
P 4200 3200
F 0 "Q3" H 4500 3250 50  0000 R CNN
F 1 "IRLML0030" H 4850 3150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 4400 3300 29  0001 C CNN
F 3 "" H 4200 3200 60  0000 C CNN
	1    4200 3200
	-1   0    0    -1  
$EndComp
$Comp
L D_Schottky D3
U 1 1 58078506
P 4400 2250
F 0 "D3" H 4400 2350 50  0000 C CNN
F 1 "SK24A" H 4400 2150 50  0000 C CNN
F 2 "rt_diodes:SMA" H 4400 2250 60  0001 C CNN
F 3 "" H 4400 2250 60  0000 C CNN
	1    4400 2250
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 58078704
P 4500 3450
F 0 "R6" V 4580 3450 50  0000 C CNN
F 1 "10k" V 4500 3450 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 4430 3450 30  0001 C CNN
F 3 "" H 4500 3450 30  0000 C CNN
	1    4500 3450
	-1   0    0    1   
$EndComp
$Comp
L R R7
U 1 1 58078816
P 4750 3200
F 0 "R7" V 4830 3200 50  0000 C CNN
F 1 "10" V 4750 3200 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 4680 3200 30  0001 C CNN
F 3 "" H 4750 3200 30  0000 C CNN
	1    4750 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 5807889D
P 1500 3450
F 0 "R1" V 1580 3450 50  0000 C CNN
F 1 "10k" V 1500 3450 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 1430 3450 30  0001 C CNN
F 3 "" H 1500 3450 30  0000 C CNN
	1    1500 3450
	-1   0    0    1   
$EndComp
$Comp
L C C1
U 1 1 580789B1
P 1700 3450
F 0 "C1" H 1725 3550 50  0000 L CNN
F 1 "22n" H 1725 3350 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1738 3300 30  0001 C CNN
F 3 "" H 1700 3450 60  0000 C CNN
	1    1700 3450
	1    0    0    -1  
$EndComp
$Comp
L ZENER_3pin_KNCA D1
U 1 1 58078AEC
P 1900 3400
F 0 "D1" H 1900 3500 50  0000 C CNN
F 1 "3V3" H 1900 3300 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 1900 3400 60  0001 C CNN
F 3 "" H 1900 3400 60  0000 C CNN
	1    1900 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 1600 4100 1700
Connection ~ 3600 1600
Wire Wire Line
	4400 1600 4400 2100
Connection ~ 4100 1600
Wire Wire Line
	4100 2200 4100 2500
Wire Wire Line
	4000 2300 4100 2300
Connection ~ 4100 2300
Wire Wire Line
	4500 3700 4500 3600
Wire Wire Line
	1500 3700 10100 3700
Wire Wire Line
	1500 3700 1500 3600
Wire Wire Line
	1500 2400 1500 3300
Wire Wire Line
	1500 3100 2000 3100
Connection ~ 1500 3100
Wire Wire Line
	1700 3300 1700 3100
Connection ~ 1700 3100
Wire Wire Line
	1900 3100 1900 3200
Wire Wire Line
	1900 3600 1900 3700
Connection ~ 1900 3700
Wire Wire Line
	1700 3600 1700 3700
Connection ~ 1700 3700
Wire Wire Line
	4100 2800 4100 3000
Wire Wire Line
	4100 3400 4100 3700
Connection ~ 4100 3700
Wire Wire Line
	4400 3200 4600 3200
Wire Wire Line
	4500 3300 4500 3200
Connection ~ 4500 3200
Wire Wire Line
	3700 2900 5400 2900
Wire Wire Line
	4400 2900 4400 2400
Connection ~ 4100 2900
$Comp
L C C6
U 1 1 5807948F
P 5300 2650
F 0 "C6" H 5325 2750 50  0000 L CNN
F 1 "10u/16" H 5325 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5338 2500 30  0001 C CNN
F 3 "" H 5300 2650 60  0000 C CNN
	1    5300 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1600 5300 2500
Connection ~ 4400 1600
Wire Wire Line
	5300 3700 5300 2800
Connection ~ 4500 3700
Text Label 4900 3200 0    60   ~ 0
SDL_SW
$Comp
L Q_PNP_BEC Q5
U 1 1 5807AA13
P 10000 2200
F 0 "Q5" H 10300 2250 50  0000 R CNN
F 1 "BC857C" H 10600 2150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10200 2300 29  0001 C CNN
F 3 "" H 10000 2200 60  0000 C CNN
	1    10000 2200
	1    0    0    1   
$EndComp
$Comp
L R R12
U 1 1 5807AA19
P 8250 1600
F 0 "R12" V 8330 1600 50  0000 C CNN
F 1 "680" V 8250 1600 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 8180 1600 30  0001 C CNN
F 3 "" H 8250 1600 30  0000 C CNN
	1    8250 1600
	0    -1   1    0   
$EndComp
$Comp
L C C11
U 1 1 5807AA1F
P 8000 1950
F 0 "C11" H 8025 2050 50  0000 L CNN
F 1 "100n" H 8025 1850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 8038 1800 30  0001 C CNN
F 3 "" H 8000 1950 60  0000 C CNN
	1    8000 1950
	-1   0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 5807AA25
P 7750 2300
F 0 "R11" V 7830 2300 50  0000 C CNN
F 1 "1k5" V 7750 2300 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 7680 2300 30  0001 C CNN
F 3 "" H 7750 2300 30  0000 C CNN
	1    7750 2300
	0    -1   1    0   
$EndComp
$Comp
L C C13
U 1 1 5807AA2B
P 9700 1850
F 0 "C13" H 9725 1950 50  0000 L CNN
F 1 "100n" H 9725 1750 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 9738 1700 30  0001 C CNN
F 3 "" H 9700 1850 60  0000 C CNN
	1    9700 1850
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR010
U 1 1 5807AA31
P 9000 2600
F 0 "#PWR010" H 9000 2450 50  0001 C CNN
F 1 "VPP" H 9000 2750 50  0000 C CNN
F 2 "" H 9000 2600 60  0000 C CNN
F 3 "" H 9000 2600 60  0000 C CNN
	1    9000 2600
	1    0    0    1   
$EndComp
Wire Wire Line
	8400 1600 10100 1600
Wire Wire Line
	10100 1600 10100 2000
Wire Wire Line
	9700 1700 9700 1600
Connection ~ 9700 1600
Wire Wire Line
	9600 2200 9800 2200
Wire Wire Line
	9700 2000 9700 2200
Connection ~ 9700 2200
Wire Wire Line
	8000 1600 8000 1800
Wire Wire Line
	7900 2300 8600 2300
Wire Wire Line
	8000 2300 8000 2100
Connection ~ 8000 2300
Wire Wire Line
	8600 2100 8500 2100
Wire Wire Line
	8500 2100 8500 1600
Connection ~ 8500 1600
$Comp
L INDUCTOR_SMALL L2
U 1 1 5807AA4D
P 7500 1950
F 0 "L2" H 7500 2050 50  0000 C CNN
F 1 "3u-18" H 7500 1900 50  0000 C CNN
F 2 "rt_chokes:Choke_SMD_Fastron_242418_FPS" H 7500 1950 60  0001 C CNN
F 3 "" H 7500 1950 60  0000 C CNN
	1    7500 1950
	0    -1   1    0   
$EndComp
$Comp
L C C9
U 1 1 5807AA53
P 7500 2650
F 0 "C9" H 7525 2750 50  0000 L CNN
F 1 "10u/16" H 7525 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 7538 2500 30  0001 C CNN
F 3 "" H 7500 2650 60  0000 C CNN
	1    7500 2650
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q4
U 1 1 5807AA59
P 7400 3200
F 0 "Q4" H 7700 3250 50  0000 R CNN
F 1 "IRLML0030" H 8050 3150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7600 3300 29  0001 C CNN
F 3 "" H 7400 3200 60  0000 C CNN
	1    7400 3200
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D6
U 1 1 5807AA5F
P 7200 2250
F 0 "D6" H 7200 2350 50  0000 C CNN
F 1 "SK24A" H 7200 2150 50  0000 C CNN
F 2 "rt_diodes:SMA" H 7200 2250 60  0001 C CNN
F 3 "" H 7200 2250 60  0000 C CNN
	1    7200 2250
	0    -1   1    0   
$EndComp
$Comp
L R R10
U 1 1 5807AA65
P 7100 3450
F 0 "R10" V 7180 3450 50  0000 C CNN
F 1 "10k" V 7100 3450 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 7030 3450 30  0001 C CNN
F 3 "" H 7100 3450 30  0000 C CNN
	1    7100 3450
	1    0    0    1   
$EndComp
$Comp
L R R9
U 1 1 5807AA6B
P 6850 3200
F 0 "R9" V 6930 3200 50  0000 C CNN
F 1 "10" V 6850 3200 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 6780 3200 30  0001 C CNN
F 3 "" H 6850 3200 30  0000 C CNN
	1    6850 3200
	0    1    -1   0   
$EndComp
$Comp
L R R13
U 1 1 5807AA71
P 10100 3450
F 0 "R13" V 10180 3450 50  0000 C CNN
F 1 "10k" V 10100 3450 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 10030 3450 30  0001 C CNN
F 3 "" H 10100 3450 30  0000 C CNN
	1    10100 3450
	1    0    0    1   
$EndComp
$Comp
L C C14
U 1 1 5807AA77
P 9900 3450
F 0 "C14" H 9925 3550 50  0000 L CNN
F 1 "22n" H 9925 3350 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 9938 3300 30  0001 C CNN
F 3 "" H 9900 3450 60  0000 C CNN
	1    9900 3450
	1    0    0    -1  
$EndComp
$Comp
L ZENER_3pin_KNCA D7
U 1 1 5807AA7D
P 9700 3400
F 0 "D7" H 9700 3500 50  0000 C CNN
F 1 "3V3" H 9700 3300 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 9700 3400 60  0001 C CNN
F 3 "" H 9700 3400 60  0000 C CNN
	1    9700 3400
	0    -1   1    0   
$EndComp
Wire Wire Line
	7500 1600 7500 1700
Connection ~ 8000 1600
Wire Wire Line
	7200 1600 7200 2100
Connection ~ 7500 1600
Wire Wire Line
	7500 2200 7500 2500
Wire Wire Line
	7600 2300 7500 2300
Connection ~ 7500 2300
Wire Wire Line
	7100 3600 7100 3700
Wire Wire Line
	10100 3700 10100 3600
Wire Wire Line
	10100 2400 10100 3300
Wire Wire Line
	9600 3100 10100 3100
Connection ~ 10100 3100
Wire Wire Line
	9900 3300 9900 3100
Connection ~ 9900 3100
Wire Wire Line
	9700 3100 9700 3200
Wire Wire Line
	9700 3600 9700 3700
Connection ~ 9700 3700
Wire Wire Line
	9900 3600 9900 3700
Connection ~ 9900 3700
Wire Wire Line
	7500 2800 7500 3000
Wire Wire Line
	7500 3400 7500 3700
Connection ~ 7500 3700
Wire Wire Line
	7000 3200 7200 3200
Wire Wire Line
	7100 3300 7100 3200
Connection ~ 7100 3200
Wire Wire Line
	6000 2900 8000 2900
Wire Wire Line
	7200 2900 7200 2400
Connection ~ 7500 2900
$Comp
L C C8
U 1 1 5807AAA9
P 6300 2650
F 0 "C8" H 6325 2750 50  0000 L CNN
F 1 "10u/16" H 6325 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6338 2500 30  0001 C CNN
F 3 "" H 6300 2650 60  0000 C CNN
	1    6300 2650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6300 1600 6300 2500
Connection ~ 7200 1600
Wire Wire Line
	6300 3700 6300 2800
Connection ~ 7100 3700
Text Label 6700 3200 2    60   ~ 0
SDR_SW
$Comp
L ZENER_3pin_KNCA D4
U 1 1 5807D3F5
P 5600 2000
F 0 "D4" H 5600 2100 50  0000 C CNN
F 1 "3V3" H 5600 1900 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5600 2000 60  0001 C CNN
F 3 "" H 5600 2000 60  0000 C CNN
	1    5600 2000
	0    1    1    0   
$EndComp
$Comp
L C C7
U 1 1 5807D903
P 5800 2050
F 0 "C7" H 5825 2150 50  0000 L CNN
F 1 "100n" H 5825 1950 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 5838 1900 30  0001 C CNN
F 3 "" H 5800 2050 60  0000 C CNN
	1    5800 2050
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5807DA39
P 5700 2550
F 0 "R8" V 5780 2550 50  0000 C CNN
F 1 "330" V 5700 2550 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 5630 2550 30  0001 C CNN
F 3 "" H 5700 2550 30  0000 C CNN
	1    5700 2550
	-1   0    0    1   
$EndComp
Connection ~ 4400 2900
Wire Wire Line
	5600 2200 5600 2300
Wire Wire Line
	5600 2300 5900 2300
Wire Wire Line
	5800 2300 5800 2200
Wire Wire Line
	5700 2300 5700 2400
Connection ~ 5700 2300
Connection ~ 6300 1600
Connection ~ 5300 1600
Wire Wire Line
	5600 1600 5600 1800
Wire Wire Line
	5800 1600 5800 1900
Connection ~ 5600 1600
Connection ~ 5800 1600
Connection ~ 5800 2300
Text Label 5900 2300 0    60   ~ 0
OPAMP-
Connection ~ 7200 2900
Connection ~ 6300 3700
Connection ~ 5300 3700
Wire Wire Line
	5700 3700 5700 3800
Connection ~ 5700 3700
Wire Wire Line
	5700 1600 5700 1500
Connection ~ 5700 1600
$Comp
L VPP #PWR011
U 1 1 580813F6
P 5700 1500
F 0 "#PWR011" H 5700 1350 50  0001 C CNN
F 1 "VPP" H 5700 1650 50  0000 C CNN
F 2 "" H 5700 1500 60  0000 C CNN
F 3 "" H 5700 1500 60  0000 C CNN
	1    5700 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 58081497
P 5700 3800
F 0 "#PWR012" H 5700 3550 50  0001 C CNN
F 1 "GND" H 5700 3650 50  0000 C CNN
F 2 "" H 5700 3800 60  0000 C CNN
F 3 "" H 5700 3800 60  0000 C CNN
	1    5700 3800
	1    0    0    -1  
$EndComp
Connection ~ 1900 3100
Text Label 2000 3100 0    60   ~ 0
SDL_CURR
Text Label 9600 3100 2    60   ~ 0
SDR_CURR
Connection ~ 9700 3100
Text Label 9000 1800 1    60   ~ 0
OPAMP-
Text Label 2600 1800 1    60   ~ 0
OPAMP-
Text Label 7300 5700 0    60   ~ 0
SDL_CURR
Text Label 7300 5600 0    60   ~ 0
SDL_SW
Text Label 7300 5500 0    60   ~ 0
SDR_CURR
Text Label 7300 5400 0    60   ~ 0
SCL
Text Label 7300 5300 0    60   ~ 0
SDR_SW
Text Label 7300 5200 0    60   ~ 0
SDA
$Comp
L CONN_01X01 P5
U 1 1 5808DD11
P 3500 2400
F 0 "P5" H 3500 2500 50  0000 C CNN
F 1 "CONN_01X01" V 3600 2400 50  0000 C CNN
F 2 "rt_pins:MILLMAX_5660" H 3500 2400 60  0001 C CNN
F 3 "" H 3500 2400 60  0000 C CNN
	1    3500 2400
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X01 P6
U 1 1 5808E013
P 3500 2900
F 0 "P6" H 3500 3000 50  0000 C CNN
F 1 "CONN_01X01" V 3600 2900 50  0000 C CNN
F 2 "rt_pins:MILLMAX_5660" H 3500 2900 60  0001 C CNN
F 3 "" H 3500 2900 60  0000 C CNN
	1    3500 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 2400 4100 2400
Connection ~ 4100 2400
$Comp
L CONN_01X01 P7
U 1 1 5808E595
P 8200 2400
F 0 "P7" H 8200 2500 50  0000 C CNN
F 1 "CONN_01X01" V 8300 2400 50  0000 C CNN
F 2 "rt_pins:MILLMAX_5660" H 8200 2400 60  0001 C CNN
F 3 "" H 8200 2400 60  0000 C CNN
	1    8200 2400
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P8
U 1 1 5808E66B
P 8200 2900
F 0 "P8" H 8200 3000 50  0000 C CNN
F 1 "CONN_01X01" V 8300 2900 50  0000 C CNN
F 2 "rt_pins:MILLMAX_5660" H 8200 2900 60  0001 C CNN
F 3 "" H 8200 2900 60  0000 C CNN
	1    8200 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2400 7500 2400
Connection ~ 7500 2400
$Comp
L D_Schottky_x2_ACom_KKA D5
U 1 1 5809D058
P 5700 2900
F 0 "D5" H 5750 2800 50  0000 C CNN
F 1 "BAT54A" H 5700 3000 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 5700 2900 60  0001 C CNN
F 3 "" H 5700 2900 60  0000 C CNN
	1    5700 2900
	1    0    0    1   
$EndComp
$Comp
L CONN_01X06 P4
U 1 1 580B8EB0
P 10300 5450
F 0 "P4" H 10300 5800 50  0000 C CNN
F 1 "CONN_01X06" V 10400 5450 50  0000 C CNN
F 2 "rt_Pin_Header:Pin_Header_Straight_1x06_2.00_Pitch" H 10300 5450 60  0001 C CNN
F 3 "" H 10300 5450 60  0000 C CNN
	1    10300 5450
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR013
U 1 1 580B9257
P 9900 5100
F 0 "#PWR013" H 9900 4950 50  0001 C CNN
F 1 "VPP" H 9900 5250 50  0000 C CNN
F 2 "" H 9900 5100 60  0000 C CNN
F 3 "" H 9900 5100 60  0000 C CNN
	1    9900 5100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR014
U 1 1 580B9307
P 10000 5100
F 0 "#PWR014" H 10000 4950 50  0001 C CNN
F 1 "+3.3V" H 10000 5240 50  0000 C CNN
F 2 "" H 10000 5100 60  0000 C CNN
F 3 "" H 10000 5100 60  0000 C CNN
	1    10000 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 580B93B7
P 10000 5800
F 0 "#PWR015" H 10000 5550 50  0001 C CNN
F 1 "GND" H 10000 5650 50  0000 C CNN
F 2 "" H 10000 5800 60  0000 C CNN
F 3 "" H 10000 5800 60  0000 C CNN
	1    10000 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 5100 10000 5500
Wire Wire Line
	10000 5500 10100 5500
Wire Wire Line
	9900 5100 9900 5600
Wire Wire Line
	9900 5600 10100 5600
Wire Wire Line
	10100 5700 10000 5700
Wire Wire Line
	10000 5700 10000 5800
Wire Wire Line
	10100 5400 9800 5400
Wire Wire Line
	10100 5300 9800 5300
Wire Wire Line
	10100 5200 9800 5200
Text Label 9800 5200 2    60   ~ 0
SDA
Text Label 9800 5300 2    60   ~ 0
SCL
Text Label 9800 5400 2    60   ~ 0
BDEP_RX
$Comp
L Q_NMOS_GSD Q6
U 1 1 58B40773
P 1600 5700
F 0 "Q6" H 1900 5750 50  0000 R CNN
F 1 "IRLML0030" H 2250 5650 50  0000 R CNN
F 2 "footprints:SOT-23" H 1800 5800 29  0001 C CNN
F 3 "" H 1600 5700 60  0000 C CNN
	1    1600 5700
	-1   0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q7
U 1 1 58B41283
P 2100 5700
F 0 "Q7" H 2400 5750 50  0000 R CNN
F 1 "IRLML0030" H 2750 5650 50  0000 R CNN
F 2 "footprints:SOT-23" H 2300 5800 29  0001 C CNN
F 3 "" H 2100 5700 60  0000 C CNN
	1    2100 5700
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 58B414CA
P 1850 4950
F 0 "R14" V 1930 4950 50  0000 C CNN
F 1 "10k" V 1850 4950 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 1780 4950 30  0001 C CNN
F 3 "" H 1850 4950 30  0000 C CNN
	1    1850 4950
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 5400 2200 5400
Wire Wire Line
	1500 5400 1500 5500
Wire Wire Line
	1500 5900 1500 6000
Wire Wire Line
	1500 6000 2900 6000
Connection ~ 2900 6000
Connection ~ 1500 4700
Connection ~ 1500 5400
Wire Wire Line
	1800 5700 1900 5700
Wire Wire Line
	2200 5900 2200 6000
Connection ~ 2200 6000
Wire Wire Line
	2200 5400 2200 5500
Wire Wire Line
	1500 4700 1850 4700
Wire Wire Line
	1850 4700 1850 4800
Wire Wire Line
	1850 5100 1850 5700
Connection ~ 1850 5700
$EndSCHEMATC
