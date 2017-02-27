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
LIBS:Helena_ctrl-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Helena Wireless Control Board"
Date "2016-12-15"
Rev "2.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L nrf51822-qfax U2
U 1 1 580DE3D6
P 4300 3600
F 0 "U2" H 4250 3350 60  0000 C CNN
F 1 "nrf51822-qfax" H 4250 3500 60  0000 C CNN
F 2 "Housings_DFN_QFN:UQFN-48-1EP_6x6mm_Pitch0.4mm" H 4200 3750 60  0001 C CNN
F 3 "" H 4200 3750 60  0000 C CNN
	1    4300 3600
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR01
U 1 1 580DE493
P 1700 3400
F 0 "#PWR01" H 1700 3250 50  0001 C CNN
F 1 "+3.3V" H 1700 3540 50  0000 C CNN
F 2 "" H 1700 3400 60  0000 C CNN
F 3 "" H 1700 3400 60  0000 C CNN
	1    1700 3400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 580DE4A9
P 3900 5600
F 0 "#PWR02" H 3900 5350 50  0001 C CNN
F 1 "GND" H 3900 5450 50  0000 C CNN
F 2 "" H 3900 5600 60  0000 C CNN
F 3 "" H 3900 5600 60  0000 C CNN
	1    3900 5600
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 580DE4BF
P 3200 5250
F 0 "C12" H 3225 5350 50  0000 L CNN
F 1 "100n" H 3225 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 3238 5100 30  0001 C CNN
F 3 "" H 3200 5250 60  0000 C CNN
	1    3200 5250
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 580DE604
P 3000 5250
F 0 "C11" H 3025 5350 50  0000 L CNN
F 1 "100n" H 3025 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 3038 5100 30  0001 C CNN
F 3 "" H 3000 5250 60  0000 C CNN
	1    3000 5250
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 580DE621
P 2800 5250
F 0 "C10" H 2825 5350 50  0000 L CNN
F 1 "22p" H 2825 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2838 5100 30  0001 C CNN
F 3 "" H 2800 5250 60  0000 C CNN
	1    2800 5250
	1    0    0    -1  
$EndComp
$Comp
L Crystal_4pin Y1
U 1 1 580DE63F
P 2550 4500
F 0 "Y1" H 2550 4750 50  0000 C CNN
F 1 "16MHz" H 2550 4650 50  0000 C CNN
F 2 "rt_XTAL:XTAL_MT_3.2mm_x_2.5mm" H 2550 4500 60  0001 C CNN
F 3 "" H 2550 4500 60  0000 C CNN
	1    2550 4500
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 580DE6E3
P 2300 5250
F 0 "C7" H 2325 5350 50  0000 L CNN
F 1 "22p" H 2325 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2338 5100 30  0001 C CNN
F 3 "" H 2300 5250 60  0000 C CNN
	1    2300 5250
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 580DE71D
P 2100 5250
F 0 "C6" H 2125 5350 50  0000 L CNN
F 1 "1n" H 2125 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2138 5100 30  0001 C CNN
F 3 "" H 2100 5250 60  0000 C CNN
	1    2100 5250
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 580DE74A
P 1900 5250
F 0 "C4" H 1925 5350 50  0000 L CNN
F 1 "100n" H 1925 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1938 5100 30  0001 C CNN
F 3 "" H 1900 5250 60  0000 C CNN
	1    1900 5250
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 580DE776
P 1700 5250
F 0 "C3" H 1725 5350 50  0000 L CNN
F 1 "100n" H 1725 5150 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1738 5100 30  0001 C CNN
F 3 "" H 1700 5250 60  0000 C CNN
	1    1700 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5400 3900 5600
Wire Wire Line
	1700 5500 3900 5500
Wire Wire Line
	3200 5500 3200 5400
Connection ~ 3900 5500
Wire Wire Line
	3000 5500 3000 5400
Connection ~ 3200 5500
Wire Wire Line
	2800 5500 2800 5400
Connection ~ 3000 5500
Wire Wire Line
	2300 5500 2300 5400
Connection ~ 2800 5500
Wire Wire Line
	2100 5500 2100 5400
Connection ~ 2300 5500
Wire Wire Line
	1900 5500 1900 5400
Connection ~ 2100 5500
Wire Wire Line
	1700 5500 1700 5400
Connection ~ 1900 5500
Wire Wire Line
	2500 4700 2500 5500
Connection ~ 2500 5500
Wire Wire Line
	2600 4700 2600 5500
Connection ~ 2600 5500
Wire Wire Line
	3500 5000 3400 5000
Wire Wire Line
	3400 5000 3400 5500
Connection ~ 3400 5500
Wire Wire Line
	3500 5100 3400 5100
Connection ~ 3400 5100
Wire Wire Line
	3500 5200 3400 5200
Connection ~ 3400 5200
Wire Wire Line
	3500 4700 3200 4700
Wire Wire Line
	3200 4700 3200 5100
Wire Wire Line
	3500 4600 3000 4600
Wire Wire Line
	3000 4600 3000 5100
Wire Wire Line
	3500 4400 2800 4400
Wire Wire Line
	2800 4400 2800 5100
Wire Wire Line
	2700 4500 2800 4500
Connection ~ 2800 4500
Wire Wire Line
	3500 4300 2300 4300
Wire Wire Line
	2300 4300 2300 5100
Wire Wire Line
	2400 4500 2300 4500
Connection ~ 2300 4500
Wire Wire Line
	2100 4100 3500 4100
Wire Wire Line
	2100 4000 2100 5100
Wire Wire Line
	3500 4000 3400 4000
Wire Wire Line
	3400 4000 3400 4100
Connection ~ 3400 4100
Wire Wire Line
	3500 3600 1900 3600
Wire Wire Line
	1900 3400 1900 5100
Wire Wire Line
	1700 3400 1700 5100
NoConn ~ 3500 3800
$Comp
L +3.3V #PWR03
U 1 1 580DEC16
P 1900 3400
F 0 "#PWR03" H 1900 3250 50  0001 C CNN
F 1 "+3.3V" H 1900 3540 50  0000 C CNN
F 2 "" H 1900 3400 60  0000 C CNN
F 3 "" H 1900 3400 60  0000 C CNN
	1    1900 3400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 580DEC3C
P 2100 4000
F 0 "#PWR04" H 2100 3850 50  0001 C CNN
F 1 "+3.3V" H 2100 4140 50  0000 C CNN
F 2 "" H 2100 4000 60  0000 C CNN
F 3 "" H 2100 4000 60  0000 C CNN
	1    2100 4000
	1    0    0    -1  
$EndComp
Connection ~ 1700 3500
Connection ~ 2100 4100
Wire Wire Line
	1700 3500 3500 3500
Connection ~ 1900 3600
$Comp
L INDUCTOR_SMALL L2
U 1 1 580DEFA0
P 2950 2300
F 0 "L2" H 2950 2400 50  0000 C CNN
F 1 "4n7" H 2950 2250 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2950 2300 60  0001 C CNN
F 3 "" H 2950 2300 60  0000 C CNN
	1    2950 2300
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L3
U 1 1 580DF0CD
P 2950 2500
F 0 "L3" H 2950 2600 50  0000 C CNN
F 1 "10n" H 2950 2450 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2950 2500 60  0001 C CNN
F 3 "" H 2950 2500 60  0000 C CNN
	1    2950 2500
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 580DF102
P 2600 2950
F 0 "C9" H 2625 3050 50  0000 L CNN
F 1 "2n2" H 2625 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2638 2800 30  0001 C CNN
F 3 "" H 2600 2950 60  0000 C CNN
	1    2600 2950
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 580DF192
P 2350 2100
F 0 "C8" H 2375 2200 50  0000 L CNN
F 1 "2p2" H 2375 2000 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2388 1950 30  0001 C CNN
F 3 "" H 2350 2100 60  0000 C CNN
	1    2350 2100
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 580DF281
P 2100 2950
F 0 "C5" H 2125 3050 50  0000 L CNN
F 1 "1p0" H 2125 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 2138 2800 30  0001 C CNN
F 3 "" H 2100 2950 60  0000 C CNN
	1    2100 2950
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L1
U 1 1 580DF2F1
P 1750 2100
F 0 "L1" H 1750 2200 50  0000 C CNN
F 1 "3n3" H 1750 2050 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1750 2100 60  0001 C CNN
F 3 "" H 1750 2100 60  0000 C CNN
	1    1750 2100
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 580DF362
P 1400 2950
F 0 "C2" H 1425 3050 50  0000 L CNN
F 1 "1p5" H 1425 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1438 2800 30  0001 C CNN
F 3 "" H 1400 2950 60  0000 C CNN
	1    1400 2950
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 580DF442
P 1200 2950
F 0 "C1" H 1225 3050 50  0000 L CNN
F 1 "dnp" H 1225 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 1238 2800 30  0001 C CNN
F 3 "" H 1200 2950 60  0000 C CNN
	1    1200 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2300 3500 2300
Wire Wire Line
	3200 2500 3300 2500
Wire Wire Line
	3300 2500 3300 2300
Connection ~ 3300 2300
Wire Wire Line
	3500 2400 3400 2400
Wire Wire Line
	3400 2400 3400 2700
Wire Wire Line
	3400 2700 2600 2700
Wire Wire Line
	2600 2500 2600 2800
Wire Wire Line
	2700 2500 2600 2500
Connection ~ 2600 2700
Wire Wire Line
	3500 2200 3400 2200
Wire Wire Line
	3400 2200 3400 2100
Wire Wire Line
	3400 2100 2500 2100
Wire Wire Line
	2600 2100 2600 2300
Wire Wire Line
	2600 2300 2700 2300
Connection ~ 2600 2100
Wire Wire Line
	2000 2100 2200 2100
Wire Wire Line
	2100 2800 2100 2100
Connection ~ 2100 2100
Wire Wire Line
	1400 2800 1400 2100
Wire Wire Line
	1200 2100 1500 2100
Wire Wire Line
	1200 2000 1200 2800
Connection ~ 1400 2100
Wire Wire Line
	1200 3100 1200 3200
Wire Wire Line
	1000 3200 2600 3200
Wire Wire Line
	2600 3100 2600 3300
Connection ~ 2600 3200
Wire Wire Line
	2100 3100 2100 3200
Connection ~ 2100 3200
Wire Wire Line
	1400 3100 1400 3200
Connection ~ 1400 3200
$Comp
L GND #PWR05
U 1 1 580DF943
P 2600 3300
F 0 "#PWR05" H 2600 3050 50  0001 C CNN
F 1 "GND" H 2600 3150 50  0000 C CNN
F 2 "" H 2600 3300 60  0000 C CNN
F 3 "" H 2600 3300 60  0000 C CNN
	1    2600 3300
	1    0    0    -1  
$EndComp
$Comp
L antenna-gnd U1
U 1 1 580DFC33
P 1200 2000
F 0 "U1" H 1200 2200 60  0000 C CNN
F 1 "antenna-gnd" H 1200 2300 60  0000 C CNN
F 2 "rt_antennas:MML_R302.302.000" H 1200 2100 60  0001 C CNN
F 3 "" H 1200 2100 60  0000 C CNN
	1    1200 2000
	-1   0    0    -1  
$EndComp
Connection ~ 1200 2100
Wire Wire Line
	1100 1900 1000 1900
Wire Wire Line
	1000 1900 1000 3200
Connection ~ 1200 3200
$Comp
L MPU-6050 U3
U 1 1 580DFEC4
P 8500 2600
F 0 "U3" H 8500 3250 60  0000 C CNN
F 1 "MPU-6050" H 8500 1950 60  0000 C CNN
F 2 "Housings_DFN_QFN:QFN-24_4x4mm_Pitch0.5mm" H 8500 2050 60  0001 C CNN
F 3 "" H 8500 2050 60  0000 C CNN
	1    8500 2600
	1    0    0    -1  
$EndComp
$Comp
L FB FB1
U 1 1 580DFFE8
P 7200 1850
F 0 "FB1" V 7280 1850 50  0000 C CNN
F 1 "FB" V 7125 1850 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 7130 1850 30  0001 C CNN
F 3 "" H 7190 1850 30  0000 C CNN
	1    7200 1850
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 580E009D
P 7800 2950
F 0 "C17" H 7825 3050 50  0000 L CNN
F 1 "2n2" H 7825 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 7838 2800 30  0001 C CNN
F 3 "" H 7800 2950 60  0000 C CNN
	1    7800 2950
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 580E03EA
P 7600 2950
F 0 "C15" H 7625 3050 50  0000 L CNN
F 1 "100n" H 7625 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 7638 2800 30  0001 C CNN
F 3 "" H 7600 2950 60  0000 C CNN
	1    7600 2950
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 580E0444
P 7400 2950
F 0 "C14" H 7425 3050 50  0000 L CNN
F 1 "10n" H 7425 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 7438 2800 30  0001 C CNN
F 3 "" H 7400 2950 60  0000 C CNN
	1    7400 2950
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 580E049D
P 7200 2950
F 0 "C13" H 7225 3050 50  0000 L CNN
F 1 "100n" H 7225 2850 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 7238 2800 30  0001 C CNN
F 3 "" H 7200 2950 60  0000 C CNN
	1    7200 2950
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 580E052F
P 9200 1850
F 0 "R3" V 9280 1850 50  0000 C CNN
F 1 "1k5" V 9200 1850 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 9130 1850 30  0001 C CNN
F 3 "" H 9200 1850 30  0000 C CNN
	1    9200 1850
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 580E05D9
P 9400 1850
F 0 "R5" V 9480 1850 50  0000 C CNN
F 1 "1k5" V 9400 1850 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 9330 1850 30  0001 C CNN
F 3 "" H 9400 1850 30  0000 C CNN
	1    9400 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 2100 8000 2100
Wire Wire Line
	7200 2000 7200 2800
Wire Wire Line
	7400 2300 8000 2300
Wire Wire Line
	7400 1900 7400 2800
Wire Wire Line
	8000 2500 7600 2500
Wire Wire Line
	7600 2500 7600 2800
Wire Wire Line
	8000 2700 7800 2700
Wire Wire Line
	7800 2700 7800 2800
$Comp
L GND #PWR06
U 1 1 580E080F
P 7900 3500
F 0 "#PWR06" H 7900 3250 50  0001 C CNN
F 1 "GND" H 7900 3350 50  0000 C CNN
F 2 "" H 7900 3500 60  0000 C CNN
F 3 "" H 7900 3500 60  0000 C CNN
	1    7900 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 3100 7900 3100
Wire Wire Line
	7900 3100 7900 3500
Wire Wire Line
	9100 3400 7200 3400
Wire Wire Line
	7800 3400 7800 3100
Connection ~ 7900 3400
Wire Wire Line
	7600 3400 7600 3100
Connection ~ 7800 3400
Wire Wire Line
	7400 3400 7400 3100
Connection ~ 7600 3400
Wire Wire Line
	7200 3400 7200 3100
Connection ~ 7400 3400
$Comp
L +3.3V #PWR07
U 1 1 580E0C37
P 7200 1500
F 0 "#PWR07" H 7200 1350 50  0001 C CNN
F 1 "+3.3V" H 7200 1640 50  0000 C CNN
F 2 "" H 7200 1500 60  0000 C CNN
F 3 "" H 7200 1500 60  0000 C CNN
	1    7200 1500
	1    0    0    -1  
$EndComp
Connection ~ 7200 2100
Connection ~ 7400 2100
Connection ~ 7400 2300
Wire Wire Line
	7200 1500 7200 1700
Wire Wire Line
	7200 1600 9400 1600
Wire Wire Line
	9200 1600 9200 1700
Connection ~ 7200 1600
Wire Wire Line
	9400 1600 9400 1700
Connection ~ 9200 1600
Wire Wire Line
	9000 2100 9500 2100
Wire Wire Line
	9200 2100 9200 2000
Wire Wire Line
	9000 2200 9500 2200
Wire Wire Line
	9400 2200 9400 2000
Wire Wire Line
	9100 2500 9100 3400
Wire Wire Line
	9100 2700 9000 2700
Wire Wire Line
	9000 2500 9100 2500
Connection ~ 9100 2700
Wire Wire Line
	9000 2600 9400 2600
NoConn ~ 9000 2800
NoConn ~ 9000 3000
NoConn ~ 9000 3100
$Comp
L R R1
U 1 1 580E1AF6
P 7900 5350
F 0 "R1" V 7980 5350 50  0000 C CNN
F 1 "470k" V 7900 5350 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 7830 5350 30  0001 C CNN
F 3 "" H 7900 5350 30  0000 C CNN
	1    7900 5350
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 580E1D3D
P 7900 5850
F 0 "R2" V 7980 5850 50  0000 C CNN
F 1 "100k" V 7900 5850 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 7830 5850 30  0001 C CNN
F 3 "" H 7900 5850 30  0000 C CNN
	1    7900 5850
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 580E1DB9
P 7700 5850
F 0 "C16" H 7725 5950 50  0000 L CNN
F 1 "100n" H 7725 5750 50  0000 L CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" H 7738 5700 30  0001 C CNN
F 3 "" H 7700 5850 60  0000 C CNN
	1    7700 5850
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR08
U 1 1 580E1E7D
P 7900 5100
F 0 "#PWR08" H 7900 4950 50  0001 C CNN
F 1 "VPP" H 7900 5250 50  0000 C CNN
F 2 "" H 7900 5100 60  0000 C CNN
F 3 "" H 7900 5100 60  0000 C CNN
	1    7900 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5100 7900 5200
Wire Wire Line
	7900 5500 7900 5700
Wire Wire Line
	7900 6000 7900 6200
Wire Wire Line
	7700 6000 7700 6100
Wire Wire Line
	7700 6100 7900 6100
Connection ~ 7900 6100
Wire Wire Line
	7700 5700 7700 5600
Wire Wire Line
	7600 5600 7900 5600
Connection ~ 7900 5600
Connection ~ 7700 5600
$Comp
L GND #PWR09
U 1 1 580E22A2
P 7900 6200
F 0 "#PWR09" H 7900 5950 50  0001 C CNN
F 1 "GND" H 7900 6050 50  0000 C CNN
F 2 "" H 7900 6200 60  0000 C CNN
F 3 "" H 7900 6200 60  0000 C CNN
	1    7900 6200
	1    0    0    -1  
$EndComp
Text Label 7600 5600 2    60   ~ 0
VIN_ADC
$Comp
L SW_PUSH SW1
U 1 1 580E2480
P 8500 5700
F 0 "SW1" H 8650 5810 50  0000 C CNN
F 1 "SW_PUSH" H 8500 5620 50  0000 C CNN
F 2 "rt_button:JTP-1138EM" H 8500 5700 60  0001 C CNN
F 3 "" H 8500 5700 60  0000 C CNN
	1    8500 5700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR010
U 1 1 580E25AB
P 8500 6200
F 0 "#PWR010" H 8500 5950 50  0001 C CNN
F 1 "GND" H 8500 6050 50  0000 C CNN
F 2 "" H 8500 6200 60  0000 C CNN
F 3 "" H 8500 6200 60  0000 C CNN
	1    8500 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 6000 8500 6200
Wire Wire Line
	8400 5300 8500 5300
Wire Wire Line
	8500 5300 8500 5400
Text Label 8400 5300 2    60   ~ 0
BUTTON
$Comp
L GND #PWR011
U 1 1 580E28BE
P 9300 6200
F 0 "#PWR011" H 9300 5950 50  0001 C CNN
F 1 "GND" H 9300 6050 50  0000 C CNN
F 2 "" H 9300 6200 60  0000 C CNN
F 3 "" H 9300 6200 60  0000 C CNN
	1    9300 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 6000 9100 6100
Wire Wire Line
	9100 6100 9500 6100
Wire Wire Line
	9300 6000 9300 6200
Wire Wire Line
	9500 6100 9500 6000
Connection ~ 9300 6100
Wire Wire Line
	9100 5000 9100 5200
Wire Wire Line
	9100 5000 9000 5000
Wire Wire Line
	9000 4800 9500 4800
Wire Wire Line
	9500 4800 9500 5200
Text Label 9000 5000 2    60   ~ 0
LED_RED
Text Label 9000 4800 2    60   ~ 0
LED_BLUE
$Comp
L R R4
U 1 1 580E2EDB
P 9100 5350
F 0 "R4" V 9180 5350 50  0000 C CNN
F 1 "680" V 9100 5350 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 9030 5350 30  0001 C CNN
F 3 "" H 9100 5350 30  0000 C CNN
	1    9100 5350
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 580E3035
P 9500 5350
F 0 "R7" V 9580 5350 50  0000 C CNN
F 1 "120" V 9500 5350 50  0000 C CNN
F 2 "rt_device:C_0603_THERMaL_RELIEF" V 9430 5350 30  0001 C CNN
F 3 "" H 9500 5350 30  0000 C CNN
	1    9500 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 5500 9500 5600
Wire Wire Line
	9100 5600 9100 5500
$Comp
L Q_PNP_BEC Q1
U 1 1 580E3666
P 10500 5900
F 0 "Q1" H 10800 5950 50  0000 R CNN
F 1 "BC857C" H 11100 5850 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 10700 6000 29  0001 C CNN
F 3 "" H 10500 5900 60  0000 C CNN
	1    10500 5900
	1    0    0    1   
$EndComp
$Comp
L GND #PWR012
U 1 1 580E396D
P 10600 6200
F 0 "#PWR012" H 10600 5950 50  0001 C CNN
F 1 "GND" H 10600 6050 50  0000 C CNN
F 2 "" H 10600 6200 60  0000 C CNN
F 3 "" H 10600 6200 60  0000 C CNN
	1    10600 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 6100 10600 6200
Wire Wire Line
	10300 5900 10200 5900
Wire Wire Line
	10600 5700 10600 5600
Wire Wire Line
	10600 5600 10200 5600
Text Label 10200 5600 2    60   ~ 0
BDEP_RX
Text Label 10200 5900 2    60   ~ 0
BDEP_TX
$Comp
L CONN_01X06 P1
U 1 1 580E4135
P 1100 6850
F 0 "P1" H 1100 7200 50  0000 C CNN
F 1 "CONN_01X06" V 1200 6850 50  0000 C CNN
F 2 "rt_Pin_Header:Pin_Header_Straight_1x06_2.00_Pitch" H 1100 6850 60  0001 C CNN
F 3 "" H 1100 6850 60  0000 C CNN
	1    1100 6850
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 580E471F
P 4000 7100
F 0 "#PWR013" H 4000 6850 50  0001 C CNN
F 1 "GND" H 4000 6950 50  0000 C CNN
F 2 "" H 4000 7100 60  0000 C CNN
F 3 "" H 4000 7100 60  0000 C CNN
	1    4000 7100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR014
U 1 1 580E48BA
P 4000 6600
F 0 "#PWR014" H 4000 6450 50  0001 C CNN
F 1 "+3.3V" H 4000 6740 50  0000 C CNN
F 2 "" H 4000 6600 60  0000 C CNN
F 3 "" H 4000 6600 60  0000 C CNN
	1    4000 6600
	1    0    0    -1  
$EndComp
Text Label 3900 7000 2    60   ~ 0
SWDIO
Text Label 3900 6800 2    60   ~ 0
SWDCLK
Text Label 3500 2000 2    60   ~ 0
SWDCLK
Text Label 3500 1900 2    60   ~ 0
SWDIO
Connection ~ 9200 2100
Connection ~ 9400 2200
Text Label 9500 2100 0    60   ~ 0
SDA
Text Label 9500 2200 0    60   ~ 0
SCL
Text Label 1300 6600 0    60   ~ 0
SDA
Text Label 1300 6700 0    60   ~ 0
SCL
Text Label 1300 6800 0    60   ~ 0
BDEP_RX
$Comp
L +3.3V #PWR015
U 1 1 580E5DFE
P 1800 6500
F 0 "#PWR015" H 1800 6350 50  0001 C CNN
F 1 "+3.3V" H 1800 6640 50  0000 C CNN
F 2 "" H 1800 6500 60  0000 C CNN
F 3 "" H 1800 6500 60  0000 C CNN
	1    1800 6500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 580E5E78
P 1400 7200
F 0 "#PWR016" H 1400 6950 50  0001 C CNN
F 1 "GND" H 1400 7050 50  0000 C CNN
F 2 "" H 1400 7200 60  0000 C CNN
F 3 "" H 1400 7200 60  0000 C CNN
	1    1400 7200
	1    0    0    -1  
$EndComp
$Comp
L VPP #PWR017
U 1 1 580E5F61
P 2000 6500
F 0 "#PWR017" H 2000 6350 50  0001 C CNN
F 1 "VPP" H 2000 6650 50  0000 C CNN
F 2 "" H 2000 6500 60  0000 C CNN
F 3 "" H 2000 6500 60  0000 C CNN
	1    2000 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6900 1800 6900
Wire Wire Line
	1800 6900 1800 6500
Wire Wire Line
	1300 7000 2000 7000
Wire Wire Line
	2000 7000 2000 6500
Wire Wire Line
	1300 7100 1400 7100
Wire Wire Line
	1400 7100 1400 7200
$Comp
L CONN_01X04 P2
U 1 1 580F1A5F
P 4300 6850
F 0 "P2" H 4300 7100 50  0000 C CNN
F 1 "CONN_01X04" V 4400 6850 50  0000 C CNN
F 2 "rt_Pin_Header:Pin_Header_Straight_1x04_Pitch1.27mm" H 4300 6850 60  0001 C CNN
F 3 "" H 4300 6850 60  0000 C CNN
	1    4300 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 6600 4000 6700
Wire Wire Line
	4000 6700 4100 6700
Wire Wire Line
	4100 6900 4000 6900
Wire Wire Line
	4000 6900 4000 7100
Wire Wire Line
	4100 7000 3900 7000
Wire Wire Line
	4100 6800 3900 6800
Text Label 5100 3700 0    60   ~ 0
BDEP_TX
Text Label 5100 3500 0    60   ~ 0
BDEP_RX
Text Label 5100 3400 0    60   ~ 0
SCL
Text Label 5100 3300 0    60   ~ 0
SDA
Text Label 5100 3100 0    60   ~ 0
BUTTON
Text Label 9400 2600 0    60   ~ 0
MPU_INT
Text Label 5100 3200 0    60   ~ 0
MPU_INT
Wire Wire Line
	9000 2300 9100 2300
Wire Wire Line
	9100 2300 9100 1900
Wire Wire Line
	9100 1900 7400 1900
Text Label 5100 2500 0    60   ~ 0
VIN_ADC
$Comp
L Double_LED D1
U 2 1 580F8953
P 9100 5800
F 0 "D1" H 9100 5900 50  0000 C CNN
F 1 "Double_LED" H 9100 5700 50  0000 C CNN
F 2 "rt_leds:Duo-Led_0603" H 9100 5800 60  0001 C CNN
F 3 "" H 9100 5800 60  0000 C CNN
	2    9100 5800
	0    -1   -1   0   
$EndComp
$Comp
L Double_LED D1
U 1 1 580F8A8E
P 9500 5800
F 0 "D1" H 9500 5900 50  0000 C CNN
F 1 "Double_LED" H 9500 5700 50  0000 C CNN
F 2 "" H 9500 5800 60  0000 C CNN
F 3 "" H 9500 5800 60  0000 C CNN
	1    9500 5800
	0    -1   -1   0   
$EndComp
Text Label 5100 4900 0    60   ~ 0
LED_RED
Text Label 5100 3000 0    60   ~ 0
LED_BLUE
NoConn ~ 5100 1900
NoConn ~ 5100 2000
NoConn ~ 5100 2100
NoConn ~ 5100 2200
NoConn ~ 5100 2300
NoConn ~ 5100 2400
NoConn ~ 5100 2600
NoConn ~ 5100 2800
NoConn ~ 5100 2900
NoConn ~ 5100 3800
NoConn ~ 5100 3900
NoConn ~ 5100 4000
NoConn ~ 5100 4100
NoConn ~ 5100 4200
NoConn ~ 5100 4300
NoConn ~ 5100 4400
NoConn ~ 5100 4600
NoConn ~ 5100 4700
NoConn ~ 5100 4800
NoConn ~ 5100 5000
NoConn ~ 5100 5100
NoConn ~ 5100 5200
$EndSCHEMATC
