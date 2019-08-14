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
LIBS:rt_transistors
LIBS:rt_74xx
LIBS:rt_device
LIBS:rt_logos
LIBS:rt_maxim
LIBS:rt_microchip
LIBS:rt_nrf
LIBS:rt_stm32
LIBS:rt_ti
LIBS:Helena_LED_Board_sd-cache
EELAYER 25 0
EELAYER END
$Descr User 8268 5827
encoding utf-8
Sheet 1 1
Title "Helena XM-L2 & XHP50 LED Board"
Date "2017-02-27"
Rev "2.0"
Comp ""
Comment1 "for use with dual step down converter"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_02X02 P1
U 1 1 58B41DE0
P 3950 2650
F 0 "P1" H 3950 2800 50  0000 C CNN
F 1 "CONN_02X02" H 3950 2500 50  0000 C CNN
F 2 "rt_Pin_Header:Pin_Header_Straight_2x02_2.00_Pitch" H 3950 1450 50  0001 C CNN
F 3 "" H 3950 1450 50  0000 C CNN
	1    3950 2650
	1    0    0    1   
$EndComp
$Comp
L C C2
U 1 1 58B41E62
P 4500 2650
F 0 "C2" H 4525 2750 50  0000 L CNN
F 1 "10u/16" H 4525 2550 50  0000 L CNN
F 2 "rt_device:C_0805" H 4538 2500 50  0001 C CNN
F 3 "" H 4500 2650 50  0000 C CNN
	1    4500 2650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 58B41EE1
P 3400 2650
F 0 "C1" H 3425 2750 50  0000 L CNN
F 1 "10u/16" H 3425 2550 50  0000 L CNN
F 2 "rt_device:C_0805" H 3438 2500 50  0001 C CNN
F 3 "" H 3400 2650 50  0000 C CNN
	1    3400 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2600 4200 2600
Wire Wire Line
	4300 2200 4300 2600
Wire Wire Line
	4200 2700 4300 2700
Wire Wire Line
	4300 2700 4300 3100
Wire Wire Line
	3700 2600 3600 2600
Wire Wire Line
	3700 2700 3600 2700
Wire Wire Line
	4300 2200 5300 2200
Wire Wire Line
	4900 2350 4900 2200
Connection ~ 4900 2200
Wire Wire Line
	4500 2500 4500 2200
Connection ~ 4500 2200
Wire Wire Line
	5300 2200 5300 2350
Wire Wire Line
	5300 2950 5300 3100
Wire Wire Line
	5300 3100 4300 3100
Wire Wire Line
	4500 2800 4500 3100
Connection ~ 4500 3100
Wire Wire Line
	4900 2950 4900 3100
Connection ~ 4900 3100
Wire Wire Line
	3600 2600 3600 2200
Wire Wire Line
	3600 2200 3000 2200
Wire Wire Line
	3000 2200 3000 2500
Wire Wire Line
	3000 2800 3000 3100
Wire Wire Line
	3000 3100 3600 3100
Wire Wire Line
	3600 3100 3600 2700
Wire Wire Line
	3400 2800 3400 3100
Connection ~ 3400 3100
Wire Wire Line
	3400 2500 3400 2200
Connection ~ 3400 2200
$Comp
L XHP50 D2
U 1 1 58B435CF
P 4900 2650
F 0 "D2" H 4925 2750 50  0000 C CNN
F 1 "XHP50" H 4925 2525 50  0000 C CNN
F 2 "rt_leds:LED_CREE-XHP50" H 4750 2650 50  0001 C CNN
F 3 "" H 4750 2650 50  0000 C CNN
	1    4900 2650
	0    -1   -1   0   
$EndComp
$Comp
L XHP50 D2
U 2 1 58B43634
P 5300 2650
F 0 "D2" H 5325 2750 50  0000 C CNN
F 1 "XHP50" H 5325 2525 50  0000 C CNN
F 2 "rt_leds:LED_CREE-XHP50" H 5150 2650 50  0001 C CNN
F 3 "" H 5150 2650 50  0000 C CNN
	2    5300 2650
	0    -1   -1   0   
$EndComp
$Comp
L LED_POWER D1
U 1 1 58B438EC
P 3000 2650
F 0 "D1" H 3000 2750 50  0000 C CNN
F 1 "LED_POWER" H 3000 2550 50  0000 C CNN
F 2 "rt_leds:LED_CREE-XML" H 3000 2650 50  0001 C CNN
F 3 "" H 3000 2650 50  0000 C CNN
	1    3000 2650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR01
U 1 1 58B43B79
P 2900 2800
F 0 "#PWR01" H 2900 2550 50  0001 C CNN
F 1 "GND" H 2900 2650 50  0000 C CNN
F 2 "" H 2900 2800 50  0000 C CNN
F 3 "" H 2900 2800 50  0000 C CNN
	1    2900 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58B43B9D
P 4800 2950
F 0 "#PWR02" H 4800 2700 50  0001 C CNN
F 1 "GND" H 4800 2800 50  0000 C CNN
F 2 "" H 4800 2950 50  0000 C CNN
F 3 "" H 4800 2950 50  0000 C CNN
	1    4800 2950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 58B43BBA
P 5200 2950
F 0 "#PWR03" H 5200 2700 50  0001 C CNN
F 1 "GND" H 5200 2800 50  0000 C CNN
F 2 "" H 5200 2950 50  0000 C CNN
F 3 "" H 5200 2950 50  0000 C CNN
	1    5200 2950
	1    0    0    -1  
$EndComp
$EndSCHEMATC
