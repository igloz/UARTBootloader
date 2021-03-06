/*
 * default_definitions.inc
 *
 *  Created: 23.09.2016 14:15:13
 *  Author: Dmitry Pogrebnyak (http://aterlux.ru/)

 *  June 2019
 *  Added support for MCUs with more then 64k flash memory
 *  Added automatic baudrate detection and connnect timeout features
 *  Author: Ihor Lozovskyi (igloz@softick.com)
 */


 /* ���� ������ ���� ������������ � ������� Atmel Studio, �� ���� � ������������� ��������� ������
   (�� �������� ������� ���������� ��� �� � ��������� �������).
   � ��������� ������ ���������� ���������� ���� � �������������, ���������������� ��
   If this file is used as the part of the Atmel Studio project then definitions for the MCU are included implicitly
   (Don't forget to select the proper MCU type in the project properties).
   Otherwise the proper file with definitions for the MCU should be included explicitly */
// .include "m88PAdef.inc" // Atmega88PA
// .include "m328Pdef.inc" // Atmega328P
// .include "m8Adef.inc"   // Atmega8A
// etc.

/* HARDWARE_ID
   ����� 15 ��������, ������� ������������ � ������ blst � �������������� ������������.
   ��� ��� .db ��������� ����� ����� UARTBootloader.asm
   Exactly 15 symbols that will be returned with the blst reply to identify a hardware. Place your own here.
   Actual .db code is placed at the end of the UARTBootloader.asm file
*/
#define HARDWARE_ID "AterLuxDefault+"

/* ���������� ������� ����������
   define the CPU frequency */
.equ F_CPU = 16000000

/* If UART_AUTOBAUD is used then UART speed is detected automatically by the software. In this case UART_SPEED setting is ignored.
   Set UART_AUTOBAUD = 1 - to enable this option, UART_AUTOBAUD = 0 - to disable it.
*/
.equ UART_AUTOBAUD = 1

/* ���������� �������� �������� UART.
   define the desired UART speed.
   �������������� �������� ����� ����� ���������� �� ��������, � ����� ��������� �� �������� ����:
   Actual UART speed may differ from the defined one. It could be calculated by the formulas below:
   X = (UART_2X==1) ? F_CPU / 8 : F_CPU / 16
   acutal UART speed = X / round(X / UART_SPEED)

   �������/examples:

   F_CPU     UART_SPEED   UART_2X  Actual Speed  �������/difference
   8000000   38400        0/1      38462         +0.2%
   8000000   115200       0        125000        +8.5% (��������/critical)
   8000000   115200       1        111111        -3.5% (����� ���� ������ ��� ��������/may cause errors)
   12000000  115200       0/1      115384        +0.2%
   16000000  38400        0/1      38462         +0.2%
   16000000  115200       0        111111        -3.5% (����� ���� ������ ��� ��������/may cause errors)
   16000000  115200       1        117647        +2.1% (����������, �� �� ��������/high, but not critical)
   20000000  115200       1        113636        -1.3%
   */
.equ UART_SPEED = 9600

/* ���������� UART_2X = 1 ����� ������������ ����� ������� �������� ��� UART
   define UART_2X = 1 to enable the double speed option */
.equ UART_2X = 1

/* UART_PARITY - ��� ��������/parity mode:
   0 - none/���
   1 - odd/��������
   2 - even/������ */
.equ UART_PARITY = 0

/* UART_STOP_BITS  ���������� �������� ����� 1 ��� 2
   Number of stop bits 1 or 2 */
.equ UART_STOP_BITS = 1

/* BLINKER - �������� ���������, ��������������� � ������ ����������.
   ������� BLINKER = 1 - ����� �������� ��� �����, BLINKER = 0 - ����� ���������.
   BLINKER_PATTERN - ������ �������. NORMAL - ����� ����� ������� ����������, READY - ����� ���������� ������� BLST, FLASHING - ����� ������ ������ ������ ��������.
   ���������� �������, ���������� ���������� ��� �������� �� 0 �� 255. �������� ��� ��������� ��������������� ������� ��, � �������������� ����� 16 ���������� � ������� ��� ������� 16���.
   ��� ���� ��������� � �������� ����������� �������� ������� �. ���� ��������� �������� ����� ���� - ��������� ���������� (������� ������� �� ������), ����� - ������� (������ �������)

   BLINKER - flashing LED to signal the bootloader activity
   Set BLINKER = 1 - to enable this option, BLINKER = 0 - to disable it.
   BLINKER_PATTERN - pattern of LED blinking. NORMAL - just after the bootloader started, READY - after BLST command, FLASHING - after first page started to be written.
   There is a counter, which cyclically increments from 0 to 255. It's change speed is proportional to the CPU frequency, roughly at the level of 16 changes per second, when running at 16MHz
   Bitwise AND is perfomed over this number and the pattern. If the operation result is zero then the LED is being turned on (high level at output) otherwise turned off (low level)

   ������� ��������/examples of patterns:
   0x00 - ��������� ������� / constantly on
   0x01 - ������ ������ / blinks fast
   0x08 - �������� ������ (~1��� � ������� ��� 16���) / blinks slowly (~1 per second @16MHz)
   0x0A - ����� �� ���� ������� / series of two flashes
   0x12 - ����� �� ������ ������� / series of four flashes
   (����� ��������� �������� �������, ��������� ����� �� 2, ����� ��������� - �������� 
    divide the numbers by two to increase the blinking frequency, multiply them to decrease it)
*/
.equ BLINKER = 1
.equ BLINKER_PORT = PORTB
.equ BLINKER_DDR = DDRB
.equ BLINKER_PIN_NUM = 7

.equ BLINKER_PATTERN_NORMAL = 0x08
.equ BLINKER_PATTERN_READY = 0x0A
.equ BLINKER_PATTERN_FLASHING = 0x01

/* MAGIC - ��� ����������� ����� ������� ����������� � ����� �� ����� EEPROM.
   ��� ������� ��������� ������ �������� ���� ������. ���� �������� ���������� �� ���������, �� ���������� ������ ����������,
   � �������� � �� �� ����������. ����� ���������� ������� �������� � ��������� ������ ��������, � ��������� ������ ������������
   �������� 0xFF, ����� �������, ���� ������� �������� ����� �������, ��� ��������� ������ �� �� ����� ��������, � ����������
   ���������.
   ��� ������ (������� BLXT), ���� ������� �������� ��� ����� (��������� ���� �� ���� ��������), � ���� MAGIC_WRITE_WHEN_FLASHED == 1,
   �� � ��������� ������ ������������ ��������� ��������. ��� ��������� ������� ����� ���������� ������� ����� � ��.
   ���� MAGIC_WRITE_WHEN_FLASHED == 0, �� �� ���� ������ ���������� ��� ��������, ���� ��������� ������ ����������.
   ����� ��������� ��� ����� ������, ��������������� ������, ������������ MAGIC

   MAGIC - it is a special number value being stored in a one of the EEPROM cells.
   When started, the bootloader reads the value in this EEPROM cell, and if the value differs from the defined one, the bootloader is being
   activated and transfer of the control to the main firmware will not occur. When the flashing process is started and the first page is loaded,
   this EEPROM cell is filled with 0xFF. Therefore, in the case the flashing process will be interrupted, the next time firmware will not get
   the control, but the bootloader will be activated instead.
   During exit (command BLXT), if the flashing process was started (at least one page was loaded), and if MAGIC_WRITE_WHEN_FLASHED == 1,
   then this cell is filled with the specified value. On the next power-on the bootloader will not be activated but will pass the control
   directly to the main firmware.
   If MAGIC_WRITE_WHEN_FLASHED == 0, then the firmware itself is obligated to write this value at this cell to prevent the bootloader to activate.
   To disable this option, comment out the line defining MAGIC.
*/
.equ MAGIC = 0xAC
.equ MAGIC_WRITE_WHEN_FLASHED = 1
.equ MAGIC_EEPROM_ADDRESS = EEPROMEND

/* ����� ��������������� ������� ���������� ��� ��������� �������
   0 - ����� ��������������� ������� ���������
   1 - ������ ������� �� ��������� ����� (���������� ������������� ��������)
   2 - ������� ������� �� ��������� ����� (������� ���������� ��������)
   3 - ��� ��������� ������ �������� ����� ����� ����������
   4 - ������ ������� �� ����� ��������� ������ (���������� ������������� ���������)

   A method for the forced activation of the bootloader
   0 - forced activation is disabled
   1 - low level on the selected pin (pull-up is enabled)
   2 - high level on the selected pin (external pull-down)
   3 - two selected pins are coupled by a jumper
   4 - low level on both selected pins (pull-ups are enabled)
*/
.equ FORCE_METHOD = 3
.equ FORCE_PORT = PORTB
.equ FORCE_DDR = DDRB
.equ FORCE_PIN = PINB
.equ FORCE_PIN_NUM = 3
// ���� ��� ������� ���� ��� ������� 3 � 4, Below is for the second pin for methods 3 and 4
.equ FORCE_PORT2 = PORTB
.equ FORCE_DDR2 = DDRB
.equ FORCE_PIN2 = PINB
.equ FORCE_PIN_NUM2 = 5

/* A number of milliseconds to wait for the commands after power-on or reset before passing the control to the main firmware.
   Set to 0 to disable this feature.
*/
.equ CONNECT_TIMEOUT = 1000
