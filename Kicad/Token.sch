EESchema Schematic File Version 2
LIBS:Token-rescue
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
LIBS:stm32
LIBS:Token-cache
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
L STM32F103C8 U?
U 1 1 581C61B8
P 4900 2800
F 0 "U?" H 3600 4450 50  0000 C CNN
F 1 "STM32F103C8" H 5950 1150 50  0000 C CNN
F 2 "LQFP48" H 4900 2800 50  0000 C CNN
F 3 "" H 4900 2800 50  0000 C CNN
	1    4900 2800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 581C6223
P 4600 800
F 0 "#PWR?" H 4600 650 50  0001 C CNN
F 1 "+3V3" H 4600 940 50  0000 C CNN
F 2 "" H 4600 800 50  0000 C CNN
F 3 "" H 4600 800 50  0000 C CNN
	1    4600 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1100 4600 800 
Wire Wire Line
	4700 1100 4700 1000
Wire Wire Line
	4600 1000 5200 1000
Connection ~ 4600 1000
Wire Wire Line
	4900 1000 4900 1100
Connection ~ 4700 1000
Connection ~ 4900 1000
Wire Wire Line
	5200 1000 5200 1100
$Comp
L GND #PWR?
U 1 1 581C635A
P 4600 4500
F 0 "#PWR?" H 4600 4250 50  0001 C CNN
F 1 "GND" H 4600 4350 50  0000 C CNN
F 2 "" H 4600 4500 50  0000 C CNN
F 3 "" H 4600 4500 50  0000 C CNN
	1    4600 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 581C636F
P 4700 4500
F 0 "#PWR?" H 4700 4250 50  0001 C CNN
F 1 "GND" H 4700 4350 50  0000 C CNN
F 2 "" H 4700 4500 50  0000 C CNN
F 3 "" H 4700 4500 50  0000 C CNN
	1    4700 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 581C637D
P 4900 4500
F 0 "#PWR?" H 4900 4250 50  0001 C CNN
F 1 "GND" H 4900 4350 50  0000 C CNN
F 2 "" H 4900 4500 50  0000 C CNN
F 3 "" H 4900 4500 50  0000 C CNN
	1    4900 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 581C638B
P 5200 4500
F 0 "#PWR?" H 5200 4250 50  0001 C CNN
F 1 "GND" H 5200 4350 50  0000 C CNN
F 2 "" H 5200 4500 50  0000 C CNN
F 3 "" H 5200 4500 50  0000 C CNN
	1    5200 4500
	1    0    0    -1  
$EndComp
$Comp
L LED D?
U 1 1 581C63B7
P 2650 4200
F 0 "D?" H 2650 4300 50  0000 C CNN
F 1 "LED" H 2650 4100 50  0000 C CNN
F 2 "" H 2650 4200 50  0000 C CNN
F 3 "" H 2650 4200 50  0000 C CNN
	1    2650 4200
	0    -1   -1   0   
$EndComp
$Comp
L LED D?
U 1 1 581C65D4
P 3000 4200
F 0 "D?" H 3000 4300 50  0000 C CNN
F 1 "LED" H 3000 4100 50  0000 C CNN
F 2 "" H 3000 4200 50  0000 C CNN
F 3 "" H 3000 4200 50  0000 C CNN
	1    3000 4200
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 581C6648
P 2650 4650
F 0 "R?" V 2730 4650 50  0000 C CNN
F 1 "R" V 2650 4650 50  0000 C CNN
F 2 "" V 2580 4650 50  0000 C CNN
F 3 "" H 2650 4650 50  0000 C CNN
	1    2650 4650
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 581C6827
P 3000 4650
F 0 "R?" V 3080 4650 50  0000 C CNN
F 1 "R" V 3000 4650 50  0000 C CNN
F 2 "" V 2930 4650 50  0000 C CNN
F 3 "" H 3000 4650 50  0000 C CNN
	1    3000 4650
	-1   0    0    1   
$EndComp
Wire Wire Line
	3000 4000 3400 4000
Wire Wire Line
	3400 3900 2650 3900
Wire Wire Line
	2650 3900 2650 4000
Wire Wire Line
	2650 4400 2650 4500
Wire Wire Line
	3000 4400 3000 4500
$Comp
L GND #PWR?
U 1 1 581C68D5
P 3000 4900
F 0 "#PWR?" H 3000 4650 50  0001 C CNN
F 1 "GND" H 3000 4750 50  0000 C CNN
F 2 "" H 3000 4900 50  0000 C CNN
F 3 "" H 3000 4900 50  0000 C CNN
	1    3000 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 581C68EF
P 2650 4900
F 0 "#PWR?" H 2650 4650 50  0001 C CNN
F 1 "GND" H 2650 4750 50  0000 C CNN
F 2 "" H 2650 4900 50  0000 C CNN
F 3 "" H 2650 4900 50  0000 C CNN
	1    2650 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 4800 2650 4900
Wire Wire Line
	3000 4800 3000 4900
$Comp
L SW_PUSH SW?
U 1 1 581C6ABF
P 3000 2700
F 0 "SW?" H 3150 2810 50  0000 C CNN
F 1 "SW_PUSH" H 3000 2620 50  0000 C CNN
F 2 "" H 3000 2700 50  0000 C CNN
F 3 "" H 3000 2700 50  0000 C CNN
	1    3000 2700
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 581C6D59
P 2700 2700
F 0 "#PWR?" H 2700 2550 50  0001 C CNN
F 1 "+3V3" H 2700 2840 50  0000 C CNN
F 2 "" H 2700 2700 50  0000 C CNN
F 3 "" H 2700 2700 50  0000 C CNN
	1    2700 2700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 2700 3300 2700
$Comp
L LCD-016N002L-RESCUE-Token DS?
U 1 1 581C7047
P 1450 2750
F 0 "DS?" H 650 3150 50  0000 C CNN
F 1 "LCD-016N002L" H 2150 3150 50  0000 C CNN
F 2 "WC1602A" H 1450 2700 50  0000 C CIN
F 3 "" H 1450 2750 50  0000 C CNN
	1    1450 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3300 1000 3300
Wire Wire Line
	1000 3300 1000 3250
Wire Wire Line
	3400 3400 900  3400
Wire Wire Line
	900  3400 900  3250
$Comp
L +5V #PWR?
U 1 1 581C7366
P 700 3400
F 0 "#PWR?" H 700 3250 50  0001 C CNN
F 1 "+5V" H 700 3540 50  0000 C CNN
F 2 "" H 700 3400 50  0000 C CNN
F 3 "" H 700 3400 50  0000 C CNN
	1    700  3400
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 581C73E8
P 800 3250
F 0 "#PWR?" H 800 3000 50  0001 C CNN
F 1 "GND" H 800 3100 50  0000 C CNN
F 2 "" H 800 3250 50  0000 C CNN
F 3 "" H 800 3250 50  0000 C CNN
	1    800  3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  3400 700  3250
$Comp
L CONN_01X04 P?
U 1 1 581C76CE
P 7400 1550
F 0 "P?" H 7400 1800 50  0000 C CNN
F 1 "RFID" V 7500 1550 50  0000 C CNN
F 2 "" H 7400 1550 50  0000 C CNN
F 3 "" H 7400 1550 50  0000 C CNN
	1    7400 1550
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 581C78A7
P 7050 1400
F 0 "#PWR?" H 7050 1250 50  0001 C CNN
F 1 "+5V" H 7050 1540 50  0000 C CNN
F 2 "" H 7050 1400 50  0000 C CNN
F 3 "" H 7050 1400 50  0000 C CNN
	1    7050 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6400 1600 7200 1600
Wire Wire Line
	7200 1700 6400 1700
$Comp
L GND #PWR?
U 1 1 581C7C89
P 7200 1500
F 0 "#PWR?" H 7200 1250 50  0001 C CNN
F 1 "GND" H 7200 1350 50  0000 C CNN
F 2 "" H 7200 1500 50  0000 C CNN
F 3 "" H 7200 1500 50  0000 C CNN
	1    7200 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 1400 7200 1400
$Comp
L CONN_01X03 P?
U 1 1 581C8097
P 7400 2000
F 0 "P?" H 7400 2200 50  0000 C CNN
F 1 "Servo" V 7500 2000 50  0000 C CNN
F 2 "" H 7400 2000 50  0000 C CNN
F 3 "" H 7400 2000 50  0000 C CNN
	1    7400 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 2100 7200 2100
$Comp
L +5V #PWR?
U 1 1 581C82C7
P 7050 1900
F 0 "#PWR?" H 7050 1750 50  0001 C CNN
F 1 "+5V" H 7050 2040 50  0000 C CNN
F 2 "" H 7050 1900 50  0000 C CNN
F 3 "" H 7050 1900 50  0000 C CNN
	1    7050 1900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 581C82ED
P 7200 2000
F 0 "#PWR?" H 7200 1750 50  0001 C CNN
F 1 "GND" H 7200 1850 50  0000 C CNN
F 2 "" H 7200 2000 50  0000 C CNN
F 3 "" H 7200 2000 50  0000 C CNN
	1    7200 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 1900 7200 1900
$Comp
L CONN_01X04 P?
U 1 1 581C8D98
P 1700 3850
F 0 "P?" H 1700 4100 50  0000 C CNN
F 1 "UART" V 1800 3850 50  0000 C CNN
F 2 "" H 1700 3850 50  0000 C CNN
F 3 "" H 1700 3850 50  0000 C CNN
	1    1700 3850
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 581C9114
P 1900 3800
F 0 "#PWR?" H 1900 3550 50  0001 C CNN
F 1 "GND" H 1900 3650 50  0000 C CNN
F 2 "" H 1900 3800 50  0000 C CNN
F 3 "" H 1900 3800 50  0000 C CNN
	1    1900 3800
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR?
U 1 1 581C92A5
P 2100 3700
F 0 "#PWR?" H 2100 3550 50  0001 C CNN
F 1 "+5V" H 2100 3840 50  0000 C CNN
F 2 "" H 2100 3700 50  0000 C CNN
F 3 "" H 2100 3700 50  0000 C CNN
	1    2100 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	2100 3700 1900 3700
Wire Wire Line
	3400 3800 2550 3800
Wire Wire Line
	2550 3800 2550 4000
Wire Wire Line
	2550 4000 1900 4000
Wire Wire Line
	1900 3900 2450 3900
Wire Wire Line
	2450 3900 2450 3700
Wire Wire Line
	2450 3700 3400 3700
Text Notes 7150 6850 0    60   ~ 0
Janne och Christoffer
Text Notes 1850 6450 0    394  ~ 0
Coffe Maniac
Text Notes 6850 3450 0    60   ~ 0
Tanken är att om du scannar ditt kort så ska du få ut en polett, ifall\ndu har pengar på kortet. 
Text Notes 6800 3200 0    197  ~ 0
Vad systemet gör
Text Notes 6850 3850 0    197  ~ 0
Triggervilkor\n
Text Notes 6850 4550 0    60   ~ 0
När man scannar sitt kort, släpps en semafor till processen för check_number_task.\n\nOm kortet är giltligt och har pengar, lägger vi in information till en kö till led_tasken, \nskiftar in bitar i eventregistret till displayen och släpper semafor till servo_tasken. \n\nEfter att displayen skrivit ut, startar vi en 4 sek timer som när den triggas, aktiverar\nen funktion. 
$EndSCHEMATC
