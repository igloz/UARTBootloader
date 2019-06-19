# UARTBootloader
This is a 1024-bytes UART bootloader for AVR-family MCUs (ATmega) with 8-256 kBytes of flash memory.
This work is based on the original work by AterLux, which you can find here: https://github.com/AterLux/UARTBootloader

Project for **Atmel Studio 6.2**

You will find **the flashing tool** here: https://github.com/igloz/UARTBootloaderGUI

Описание исходного проекта (не этого, а исходного) на русском языке здесь: https://www.drive2.ru/b/2878717/

## Main features

* **Magic byte control**. The bootloader can check the value of a predefined byte cell in EEPROM, and pass control to the main firmware only if this value is as expected.
The bootloader clears this EEPROM cell at the begining of a flashing process, and at the end of this process it can either set up the value by itself or leave this task to the main firmware.
The last option is useful to allow the firmware to complete some post-upgrade actions, and/or to ensure communication is still possible.   

* **Forced start**. If a condition for the forced start is met, the bootloader doesn't pass control to the main firmware, but activates and waits for the commands. The condition can be one of
  1. Low level on a particular input pin (pull-up enabled)
  2. High level on a particular input pin (external pull-down is required)
  3. A jumper between the two input pins
  4. Low level on the two input pins (pull-ups enabled)

* **Start on call**. The bootloader activates and waits for the commands if it is called directly from the firmware. 
After power-on or reset, if the condition from the list above is not met (or the corresponding check is disabled), the bootloader passes control to the main firmware.

* **Connect timeout**. After power-on or reset the bootloader activates and waits for the commands until a specified timeout elapses.
If the timeout elapses and there are no commands, the bootloader passes control to the main firmware.

* **Blinker**. The bootloader can display different blinking patterns on the led connected to a particular pin to indicate the mode of operation.

* **Automatic baudrate detection**. The bootloader can detect the baudrate of the incoming UART connection and automatically configure itself.

## How to use

1. Open the project in *Atmel Studio*.
2. Go to project properties, *Device* tab, and *Change Device* according to yours.
3. Open ***default_definitions.inc*** file and change the definitions as you need:
  * *HARDWARE_ID* - Exactly 15 characters that will be returned with the blst reply to identify a hardware. I recommend to use a combination of your nickname and the device name. For example: *"JhnSmithBlinker"*.
  * *F_CPU* - define the CPU frequency, as it is set by the hardware and the fuse settings.
  * *UART_AUTOBAUD* - detect UART speed automatically: 1 - enabled, 0 - disabled.
  * *UART_SPEED* - define the desired UART speed. Note: not all speeds are exactly supported. Refer to the MCU datasheet.
  * *UART_2X* - UART 2x mode (refer to the MCU datasheet): 1 - enabled, 0 - disabled.
  * *UART_PARITY* - UART parity mode: 0 - none, 1 - odd, 2 - even.
  * *UART_STOP_BITS* - the number of stop bits: 1 or 2.
  
  * *BLINKER* - enable blinker options: 1 - enable, 0 - disable.
  * *BLINKER_DDR*, *BLINKER_PORT*, *BLINKER_PIN_NUM* - I/O registers and bit number to access the blinker pin (if enabled).
  * *BLINKER_PATTERN_...* - different patterns of blinking (read description in the source).
  
  * *MAGIC* - a magic byte value, could not be 0xFF. Comment out this line to disable the magic byte control.
  * *MAGIC_WRITE_WHEN_FLASHED* - 1 - if the bootloader should write the value by itself, 0 - if it would be performed by the main firmware.
  * *MAGIC_EEPROM_ADDRESS* - the address in EEPROM where the magic value is stored.

  * *FORCE_METHOD* - a method for the forced start: 0 - disabled, 1 - low level, 2 - high level (requires external pull-down), 3 - a jumper between the two pins, 4 - low level on the two pins.
  * *FORCE_PORT*, *FORCE_DDR*, *FORCE_PIN*, *FORCE_PIN_NUM* - the I/O registers and the bit number to access the pin.
  * *FORCE_PORT2*, *FORCE_DDR2*, *FORCE_PIN2*, *FORCE_PIN_NUM2* - the same for the second pin, for the methods 3 and 4.

  * *CONNECT_TIMEOUT* - number of milliseconds to wait for the commands after power-on or reset before passing the control to the main firmware. Set to 0 to disable this feature.
4. Setup the MCU fuses:
  * Select BOOTSZ bits to match the 512 words (1024 bytes) bootloader size.
  * Set BOOTRST fuse bit (i.e. make it value equal to 0) - that makes the MCU to execute the bootloader first after reset.
5. Compile the project and flash it into the MCU.

## Protocol description. 

All requests start with the bytes *0x42 0x4C* (**BL**), and replies start with the bytes *0x62 0x6C* (**bl**)
Reply *0x62 0x6C 0x65 0x72* (**bler**) means error (e.g. wrong command params, etc)
   
### BLST

Request to start flashing process

**Request:** *0x42 0x4C 0x53 0x54* (**BLST**) 

**Reply:** *0x62 0x6C 0x73 0x74 psw np <fwid>* (**blst...**) 
* *psw* - 1 byte size of the flash page (in words)
* *np* - 2 bytes number of pages available (excluding area occupied by bootloader)
* *fwid* - 15 symbols to identify hardware

### BLPG

Flasing a page

Before loading the first page, *BLST* command must be performed.
It is allowed to repeately perform *BLST* command to start flashing process over again.

**Request:** *0x42 0x4C 0x50 0x47 pg `<data>` cc* (**BLPG...**) 
* *pg* - 2 bytes number of page `(0 <= *pg* < *np*)`
* *`<data>`* - *psw* * 2 bytes of page data 
* *cc* - 1 byte CRC with polinomial x^8 +x^7 +x^6 +x^3 +x^2 +x +1 over all bytes in the *`<data>`* field, MSB first, initialized with 0xFF:
```c         
    cc = 0xFF;
    for (i = 0; i < sizeof(data); i++) {
        cc ^= data[i];
        for (b = 0; b < 8; b++) {
            cc = ((cc & 0x80) ? 0xCF : 0) ^ (cc << 1);
        }
    }
```

**Reply:** *0x62 0x6C 0x70 0x77 pg* (**blpg.**) - the page is successfully flashed

*0x62 0x6C 0x65 0x72* (**bler**) - cc byte doesn't match, or invalid pg value

### BLXT

Finalize the flashing process and pass control to the firmware

**Request:** *0x42 0x4C 0x58 0x54* (**BLXT**) 

**Reply:**  *0x62 0x6C 0x78 0x74* (**blxt**) 

## Discrepancies with the original AterLux project

* Size of bootloader. This version occupies 1024 bytes of memory.
* Size of flash memory of supported MCUs. This version works on MCUs with more than 64 kBytes of flash memory.
* Features. This version supports **Automatic baudrate detection** and **Connect timeout** features.
* Number of bytes to encode page number/number of pages. Two bytes are used instead of one byte.
* Number of bytes in *HARDWARE_ID*. The size of *HARDWARE_ID* is 15 bytes instead of 14.
* Order of pages while flashing. The pages can be flashed in any order.

## Have any questions?

Write me: igloz@softick.com

You could also write to the author of the original project: dmitry@aterlux.ru
