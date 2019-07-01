/*
 * UARTBootloader.asm
 *
 *  Created: 13.03.2016
 *  Author: Dmitry Pogrebnyak (http://aterlux.ru/)

 *  June 2019
 *  Added support for MCUs with more then 64k flash memory
 *  Added automatic baudrate detection and connnect timeout features
 *  Author: Ihor Lozovskyi (igloz@softick.com)
 */

/* Описание протокола. Все запросы начинаются с байтов  0x42 0x4C (BL), а ответы с байтов 0x62 0x6C (bl)
   Ответ 0x62 0x6C 0x65 0x72 (bler) означает ошибку (напр, в параметрах команды)

   Запрос: 0x42 0x4C 0x53 0x54 (BLST) запрос на начало работы загрузчика
   Ответ: 0x62 0x6C 0x73 0x74 psw np <fwid> (blst...)
         psw - 1 байт размер страницы флеш-памяти в словах
         np - 2 байта количество доступных для программирования страниц (не считая загрузчика)
         fwid - 15 символов с идентифицирующим кодом

   Запрос: 0x42 0x4C 0x50 0x47 pgn <data> cc (BLPG...) загрузка и прошивка страницы с номером pg
         pg - 2 байта номер страницы (0 <= pg < np)
         <data> - psw * 2 байт данных страницы
         cc - CRC с полиномом x^8 +x^7 +x^6 +x^3 +x^2 +x +1 по всем байтам в поле data, сначала старший бит, инициализированное значением 0xFF:
              cc = 0xFF;
              for (i = 0; i < sizeof(data); i++) {
                cc ^= data[i];
                for (b = 0; b < 8; b++) {
                   cc = ((cc & 0x80) ? 0xCF : 0) ^ (cc << 1);
                }
              }

         процессу загрузки должен предшествовать вызов команды BLST
         допустимо повторно вызвать BLST и начать загрузку заново
   Ответ: 0x62 0x6C 0x70 0x67 pg (blpg.) - OK
          0x62 0x6C 0x65 0x72 (bler) - не совпал контрольный байт, или недопустимый номер страницы

   Запрос: 0x42 0x4C 0x58 0x54 (BLXT) - завершить процесс прошивки и перейти к программе
   Ответ:  0x62 0x6C 0x78 0x74 (blxt)


   Protocol description. All requests start with the bytes 0x42 0x4C (BL), and replies start with the bytes 0x62 0x6C (bl)
   Reply 0x62 0x6C 0x65 0x72 (bler) means error (e.g. wrong command params, etc)

   Request: 0x42 0x4C 0x53 0x54 (BLST) request to start the flashing process
   Reply: 0x62 0x6C 0x73 0x74 psw np <fwid> (blst...)
         psw - 1 byte size of the flash page (in words)
         np - 2 bytes number of pages available (excluding area occupied by bootloader)
         fwid - 15 symbols to identify the hardware

   Request: 0x42 0x4C 0x50 0x47 pg <data> cc (BLPG...) flasing a page with a number pg
         pg - 2 bytes page number (0 <= pg < np)
         <data> - psw * 2 bytes of page data
         cc - CRC with polinomial x^8 +x^7 +x^6 +x^3 +x^2 +x +1 over all bytes in the <data> field, MSB first, initialized with 0xFF:
              cc = 0xFF;
              for (i = 0; i < sizeof(data); i++) {
                cc ^= data[i];
                for (b = 0; b < 8; b++) {
                   cc = ((cc & 0x80) ? 0xCF : 0) ^ (cc << 1);
                }
              }

         before loading the first page, BLST command must be performed
         It is allowed to repeately perform BLST command to start flashing process over again
   Reply: 0x62 0x6C 0x70 0x67 pg (blpg.) - OK
          0x62 0x6C 0x65 0x72 (bler) - cc byte doesn't match, or invalid pg value

   Request: 0x42 0x4C 0x58 0x54 (BLXT) - finalize the flashing process and pass control to the firmware
   Reply:  0x62 0x6C 0x78 0x74 (blxt)
*/

.include "default_definitions.inc"

/**********************************************************************************************************************************/
// Проверки и предварительные вычисления / Checks and precalculations

.ifndef F_CPU 
   .equ F_CPU = 8000000
   .warning "F_CPU is not defined. Defaults to 8MHz"
.endif

.if (F_CPU < 1000000) || (F_CPU > 20000000) 
   .warning "Suspicious F_CPU setting. Please check"
.endif

.ifndef UART_PARITY
   .equ UART_PARITY = 0
   .warning "UART_PARITY is not defined. Defaults to 0 (disabled)"
.elif (UART_PARITY != 0) && (UART_PARITY != 1) && (UART_PARITY != 2)
   .error "UART_PARITY should be 0, 1 or 2"
.endif

.ifndef UART_STOP_BITS
   .equ UART_STOP_BITS = 1
   .warning "UART_STOP_BITS is not defined. Defaults to 1"
.elif (UART_STOP_BITS != 1) && (UART_STOP_BITS != 2)
   .error "UART_STOP_BITS should be either 1 or 2"
.endif

.ifndef UART_AUTOBAUD
   .equ UART_AUTOBAUD = 0
.endif

.if UART_AUTOBAUD
  .ifndef UART_2X
     .equ UART_2X = 0
  .endif

  .ifdef PORTE
    .equ UART_PORT = PORTE
    .equ UART_DDR = DDRE
    .equ UART_PIN = PINE
    .equ UART_RX_PIN = PINE0
    .equ UART_TX_PIN = PINE1
  .else
    .equ UART_PORT = PORTD
    .equ UART_DDR = DDRD
    .equ UART_PIN = PIND
    .equ UART_RX_PIN = PIND0
    .equ UART_TX_PIN = PIND1
  .endif

  .equ TIMEOUT_10_MSEC = 10 * F_CPU / 1000
  .equ TIMEOUT_200_MSEC = 200 * F_CPU / 1000

.else // !UART_AUTOBAUD

  .ifndef UART_2X
     .equ UART_2X = 0
     .warning "UART_2X is not defined. Defaults to 0 (disabled)"
  .elif ((UART_2X != 0) && (UART_2X != 1))
     .error "UART_2X should be either 1 or 0"
  .endif

  .ifndef UART_SPEED
     .equ UART_SPEED = 9600
     .warning "UART_SPEED is not defined. Defaults to 9600"
  .endif

  .if (UART_SPEED < 1200) || (UART_SPEED > 1000000)
     .warning "Suspicious UART_SPEED setting. Please check"
  .endif

  .equ UBRR_value = ((F_CPU / 8 / ((UART_2X == 1) ? 1 : 2) + UART_SPEED / 2) / UART_SPEED > 0) ?  ((F_CPU / 8 / ((UART_2X == 1) ? 1 : 2) + UART_SPEED / 2) / UART_SPEED) - 1 : 0

  .if UBRR_value > 4095
     .error "UART_SPEED too small. Try to increase and/or set UART_2X = 0"
  .endif

  .equ UART_SPEED_actual = (F_CPU / 8 / ((UART_2X == 1) ? 1 : 2) + (UBRR_VALUE + 1) / 2) / (UBRR_VALUE + 1)

  .equ UART_SPEED_difference_mil = (abs(UART_SPEED - UART_SPEED_actual) * 1000 + UART_SPEED / 2) / UART_SPEED

  .if UART_SPEED_difference_mil > ((!defined(UART_PARITY) || (UART_PARITY == 0)) ? 45 : 41)
    .error "The actual speed of UART critically differs from defined. Communication impossible. Try to set UART_2X = 1 and/or change the UART_SPEED value"
  .elif UART_SPEED_difference_mil > 30
    .warning "The actual speed of UART will differ more than 3 percents from defined! This may cause communication errors. Try to set UART_2X = 1 and/or change the UART_SPEED value"
  .elif UART_SPEED_difference_mil > 21
    .warning "The actual speed of UART will differ more than 2.1 percents from defined. Try to set UART_2X = 1"
  .endif
.endif // !UART_AUTOBAUD

.ifdef MAGIC
  .if ((MAGIC >= 0xFF) || (MAGIC < 0))
    .error "MAGIC should by a byte value less than 0xFF"
  .endif
.endif

.ifndef BLINKER
  .equ BLINKER = 0
.endif

.ifndef CONNECT_TIMEOUT
  .equ CONNECT_TIMEOUT = 0
.else
  .if (CONNECT_TIMEOUT < 400) || (CONNECT_TIMEOUT > 120000)
     .warning "Suspicious CONNECT_TIMEOUT setting. Please check"
  .endif
  .equ CONNECT_TIMEOUT_TICKS = CONNECT_TIMEOUT * F_CPU / 1000
  .equ CONNECT_TIMEOUT_COUNT = 65536 - CONNECT_TIMEOUT_TICKS / 65536
.endif

.equ USES_TIMER = (UART_AUTOBAUD || BLINKER || CONNECT_TIMEOUT)

.equ BIG = (FLASHEND > 0xFFFF)
.if BIG && (PAGESIZE != 128)
  .error "Unsupported PAGESIZE and flash memory size combination"
.endif

.equ BOOTLOADER_SIZE_WORDS = 0x200
.equ BOOTLOADER_OFFSET_WORDS = FLASHEND + 1 - BOOTLOADER_SIZE_WORDS

.equ PAGES_AVAILABLE = BOOTLOADER_OFFSET_WORDS / PAGESIZE

.if defined(SELFPRGEN) && !defined(SPMEN)
  .equ SPMEN = SELFPRGEN
.endif

.macro inp
.if @1 < 0x40
  in @0, @1
.else
  lds @0, @1
.endif
.endm

.macro outp
.if @0 < 0x40
  out @0, @1
.else
  sts @0, @1
.endif
.endm

.macro sbis_r
  .if @0 < 0x20
    sbis @0, @1
  .else
    inp @2, @0
    sbrs @2, @1
  .endif
.endm

.macro sbic_r
  .if @0 < 0x20
    sbic @0, @1
  .else
    inp @2, @0
    sbrc @2, @1
  .endif
.endm

.macro sbi_r // use with care!
  .if @0 < 0x20
    sbi @0, @1
  .else
    ldi @2, (1 << (@1))
    outp @0, @2
  .endif
.endm

.dseg
.org SRAM_START

.ifndef GPIOR0
  save_MCUSR:
    .byte 1
.endif

.if UART_AUTOBAUD
  counters:
    .byte 60
.endif


/**********************************************************************************************************************************/
// Основной код загрузчика / Bootloader body

// r16, r17, r22, r23 - временные/параметры
// r18 - текущее состояние
// r20, r19 - временные вне процедур
// r28 - при выполнении uart_read побитно выполняется "или" со значением регистра UCSR0A/UCSRA
// r31:r30 - адрес для прошивки
// r27 - паттерн блинкера,
// r25:r24 - круговой счётчик
// r3:r2 - текущая страница

// These are bit flags
.equ STATE_ENTER = 0 // Сразу после входа, BLPG запрещено
.equ STATE_READY = 1 // После BLST, программирование разрешено
.equ STATE_FLASHING = 2 // Была выполнена хотя бы одна BLPG
.equ STATE_FORCED = 3

.cseg
.org BOOTLOADER_OFFSET_WORDS
boot_loader:

cli
ldi r16, high(RAMEND)
outp SPH, r16
ldi r16, low(RAMEND)
outp SPL, r16

rcall wait_eeprom_finish
rcall clear_ports // на выходе r16 == 0

////////// Проверка условий на запуск бутлоадера
ldi r18, (1 << STATE_ENTER) | (1 << STATE_FORCED)

// 1. Проверка и сброс MCUSR (MCUCSR): если в нём нет установленных флагов, значит бутлоадер вызван переходом из программы
.ifdef MCUSR
  inp r17, MCUSR
  outp MCUSR, r16
.else
  inp r17, MCUCSR
  outp MCUCSR, r16
.endif

tst r17
breq enter_bootloader

.ifdef GPIOR0
  outp GPIOR0, r17
.else
  sts save_MCUSR, r17
.endif

// 2. Проверка маркера
.ifdef MAGIC 
  rcall load_magic_eear
  sbi EECR, EERE
  inp r17, EEDR
  cpi r17, MAGIC
  brne enter_bootloader
.endif

// 3. Проверка уровня на входах
.ifdef FORCE_METHOD
  .if (FORCE_METHOD == 1) // Первый метод: низкий уровень на пине (включается пулл-ап)
    sbi FORCE_PORT, FORCE_PIN_NUM
    ldi r17, 255
    test_loop1:
      sbic FORCE_PIN, FORCE_PIN_NUM
      rjmp not_forced
      dec r17
    brne test_loop1
    rjmp enter_bootloader

  .elif (FORCE_METHOD == 2) // Второй метод: высокий уровень на пине 
    ldi r17, 255
    test_loop1:
      sbis FORCE_PIN, FORCE_PIN_NUM
      rjmp not_forced
      dec r17
    brne test_loop1
    rjmp enter_bootloader

  .elif FORCE_METHOD == 3 // Третий метод: два пина замкнуты перемычкой
    // Включим подтяжку и проверим что на обоих пинах высокий уровень
    sbi FORCE_PORT, FORCE_PIN_NUM
    sbi FORCE_PORT2, FORCE_PIN_NUM2
    rjmp PC+1 // Пауза 2 такта
    sbic FORCE_PIN, FORCE_PIN_NUM
    sbis FORCE_PIN2, FORCE_PIN_NUM2
      rjmp not_forced

    // На первый пин выдадим низкий уровень и проверим что низкий уровень на втором пине
    cbi FORCE_PORT, FORCE_PIN_NUM
    sbi FORCE_DDR,  FORCE_PIN_NUM
    rjmp PC+1 // Пауза 2 такта
    sbic FORCE_PIN2, FORCE_PIN_NUM2
      rjmp not_forced

    // Включаем подтяжку обратно и проверяем что у нас по-прежнему читается высокий уровень
    cbi FORCE_DDR,  FORCE_PIN_NUM
    sbi FORCE_PORT, FORCE_PIN_NUM
    rjmp PC+1 // Пауза 2 такта
    sbic FORCE_PIN, FORCE_PIN_NUM
    sbis FORCE_PIN2, FORCE_PIN_NUM2
      rjmp not_forced

    // На второй пин выдадим низкий уровень и проверим что низкий уровень на первом пине
    cbi FORCE_PORT2, FORCE_PIN_NUM2
    sbi FORCE_DDR2,  FORCE_PIN_NUM2
    rjmp PC+1 // Пауза 2 такта
    sbis FORCE_PIN, FORCE_PIN_NUM
    // Очевидно, пины замкнуты меж собой. Переходим к бутлоадеру
    rjmp enter_bootloader

  .elif (FORCE_METHOD == 4) // Четвёртый метод: низкий уровень на двух пинах (включаются пулл-апы)
    sbi FORCE_PORT, FORCE_PIN_NUM
    sbi FORCE_PORT2, FORCE_PIN_NUM2
    ldi r17, 255
    test_loop1:
      sbis FORCE_PIN, FORCE_PIN_NUM
      sbic FORCE_PIN2, FORCE_PIN_NUM2
      rjmp not_forced
      dec r17
    brne test_loop1
    rjmp enter_bootloader
  .endif
  not_forced:
.endif // FORCE_METHOD

.if CONNECT_TIMEOUT
  cbr r18, (1 << STATE_FORCED)
  rjmp enter_bootloader
.endif

jump_to_firmware:
  rcall clear_ports // r16 == 0 на выходе
  // Stop UART
  .ifdef UCSR0B
     outp UCSR0B, r16
  .else
     outp UCSRB, r16
  .endif
  .if USES_TIMER
    // Stop timer
     outp TCCR1B, r16
  .endif
  .if FLASHEND < 0x1000 // у устройств с 8k отсутствует инструкция jmp
    rjmp 0x0000
  .else
    jmp 0x0000
  .endif

enter_bootloader: // r16 == 0 at this point
.if USES_TIMER
  // Init timer
  ldi r16, (1 << CS10)
  outp TCCR1B, r16
.endif

.if BLINKER
  sbi BLINKER_DDR, BLINKER_PIN_NUM
  ldi r27, BLINKER_PATTERN_NORMAL
.endif

.if CONNECT_TIMEOUT
  ldi r24, low(CONNECT_TIMEOUT_COUNT)
  ldi r25, high(CONNECT_TIMEOUT_COUNT)
.endif

.if UART_AUTOBAUD
  ldi r16, (1 << UART_TX_PIN) | (1 << UART_RX_PIN)
  out UART_PORT, r16

  do_autobaud:
    rcall autobaud
    .if CONNECT_TIMEOUT
      sbrs r18, STATE_FORCED
      breq jump_to_firmware
    .endif
  breq do_autobaud

  // Инициализация USART
  .ifdef UCSR0A
    ldi r16, ((UART_2X == 1) ? (1 << U2X0) : 0)
    outp UCSR0A, r16
    outp UBRR0L, r4
    outp UBRR0H, r5
    ldi r16, (1 << RXEN0) | (1 << TXEN0)
    outp UCSR0B, r16
    ldi r16, (1 << UCSZ01) | (1 << UCSZ00) | ((UART_PARITY == 1) ? ((1 << UPM01) | (1 << UPM00)) : ((UART_PARITY == 2) ? (1 << UPM01) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS0) : 0)
    outp UCSR0C, r16
  .else
    // вариант регистров ATmega8 
    ldi r16, ((UART_2X == 1) ? (1 << U2X) : 0)
    outp UCSRA, r16
    outp UBRRL, r4
    outp UBRRH, r5
    ldi r16, (1 << RXEN) | (1 << TXEN)
    outp UCSRB, r16
    .ifdef URSEL
      ldi r16, (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0) | ((UART_PARITY == 1)  ? ((1 << UPM1) | (1 << UPM0)) : ((UART_PARITY == 2) ? (1 << UPM1) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS) : 0)
    .else
      ldi r16, (1 << UCSZ1) | (1 << UCSZ0) | ((UART_PARITY == 1)  ? ((1 << UPM1) | (1 << UPM0)) : ((UART_PARITY == 2) ? (1 << UPM1) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS) : 0)
    .endif
    outp UCSRC, r16
  .endif

  ldi r16, 0
  out UART_PORT, r16

.else // !UART_AUTOBAUD

  // Инициализация USART
  .ifdef UCSR0A
    ldi r16, ((UART_2X == 1) ? (1 << U2X0) : 0)
    outp UCSR0A, r16
    ldi r16, low(UBRR_VALUE)
    outp UBRR0L, r16
    ldi r16, high(UBRR_VALUE)
    outp UBRR0H, r16
    ldi r16, (1 << RXEN0) | (1 << TXEN0)
    outp UCSR0B, r16
    ldi r16, (1 << UCSZ01) | (1 << UCSZ00) | ((UART_PARITY == 1) ? ((1 << UPM01) | (1 << UPM00)) : ((UART_PARITY == 2) ? (1 << UPM01) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS0) : 0)
    outp UCSR0C, r16
  .else
    // вариант регистров ATmega8 
    ldi r16, ((UART_2X == 1) ? (1 << U2X) : 0)
    outp UCSRA, r16
    ldi r16, low(UBRR_VALUE)
    outp UBRRL, r16
    ldi r16, high(UBRR_VALUE)
    outp UBRRH, r16
    ldi r16, (1 << RXEN) | (1 << TXEN)
    outp UCSRB, r16
    .ifdef URSEL
      ldi r16, (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0) | ((UART_PARITY == 1)  ? ((1 << UPM1) | (1 << UPM0)) : ((UART_PARITY == 2) ? (1 << UPM1) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS) : 0)
    .else
      ldi r16, (1 << UCSZ1) | (1 << UCSZ0) | ((UART_PARITY == 1)  ? ((1 << UPM1) | (1 << UPM0)) : ((UART_PARITY == 2) ? (1 << UPM1) : 0)) | ((UART_STOP_BITS == 2) ? (1 << USBS) : 0)
    .endif
    outp UCSRC, r16
  .endif
.endif // !UART_AUTOBAUD

/***************/
// Цикл ожидания команд

// BLST - вход в режим программирования
// BLPG - данные страницы (2 байта номер страницы + n байт страница + 1 байт - контроль)
// BLXT - завершение программирования
main_loop:
  rcall uart_read
  test_b:
    cpi r16, 'B'
  brne main_loop
  rcall uart_read
  cpi r16, 'L'
  brne test_b

  rcall uart_read
  cpi r16, 'S'
  breq test_blst
  cpi r16, 'P'
  breq test_blpg
  cpi r16, 'X'
  brne test_b
test_blxt:
  rcall uart_read
  cpi r16, 'T'
  brne test_b
  // выход из загрузчика
  sbrs r18, STATE_FLASHING
  rjmp blxt_exit
    ldi r16, (1 << RWWSRE) | (1 << SPMEN)
    rcall do_spm
    .ifdef MAGIC
      .if MAGIC_WRITE_WHEN_FLASHED == 1 
        ldi r16, MAGIC
        rcall write_magic
      .endif
    .endif
blxt_exit:
  .ifdef UCSR0A
    ldi r16, (1 << TXC0) | ((UART_2X == 1) ? (1 << U2X0) : 0)
    outp UCSR0A, r16
  .else
    sbi UCSRA, TXC
  .endif
  ldi r22, 'x'
  ldi r23, 't'
  rcall uart_blrep
  wait_transmission_complete:
    .ifdef UCSR0A
      inp r16, UCSR0A
      sbrs r16, TXC0
    .else
      sbis UCSRA, TXC
    .endif
    rjmp wait_transmission_complete
  rjmp jump_to_firmware

test_blst:
  rcall uart_read
  cpi r16, 'T'
  brne test_b

  sbrs r18, STATE_FLASHING
  rjmp blst_finish
    ldi r16, (1 << RWWSRE) | (1 << SPMEN)
    rcall do_spm
  blst_finish:
  sbr r18, (1 << STATE_READY)
  .if BLINKER
    ldi r27, BLINKER_PATTERN_READY
  .endif
  ldi r22, 's'
  ldi r23, 't'
  rcall uart_blrep

  ldi r30, low(bootloader_id * 2)
  ldi r31, high(bootloader_id * 2)
  .if BIG
    ldi r20, byte3(bootloader_id * 2)
    outp RAMPZ, r20
  .endif
  ldi r20, 18
  blst_info_loop:
    .if BIG
      elpm r16, Z+
    .else
      lpm r16, Z+
    .endif
    rcall uart_write
    dec r20 
    brne blst_info_loop
rjmp main_loop

bler_and_loop:
  ldi r22, 'e'
  ldi r23, 'r'
  rcall uart_blrep
rjmp main_loop

test_blpg:
  rcall uart_read 
  cpi r16, 'G'
  brne test_b

  rcall uart_read
  mov r2, r16 // low(page number)
  rcall uart_read
  mov r3, r16 // high(page number)

  sbrs r18, STATE_READY // Если не был выполнен BLST, то - ошибка
  rjmp blpg_skip

  ldi r16, low(PAGES_AVAILABLE)
  ldi r17, high(PAGES_AVAILABLE)
  cp  r2, r16
  cpc r3, r17
  brlo blpg_state_ok

  blpg_skip:
  ldi r19, PAGESIZE * 2 - 1 // скипается на 2 байта меньше, чем длина данных страницы. На случай если байт был потерян при доставке, к моменту следующего пакета парсер будет в состоянии ожидания
  blpg_skip_loop:
    rcall uart_read
    dec r19
  brne blpg_skip_loop
  rjmp bler_and_loop

  blpg_state_ok:
  .if PAGESIZE == 128
    .if BIG
      outp RAMPZ, r3
    .endif
    mov r31, r2
    clr r30
  .else
    ldi r19, (PAGESIZE * 2)
    mul r2, r19
    movw r30, r0
  .endif
  ldi r19, PAGESIZE

  ldi r20, 0xFF
  // Загрузка страницы в страничный буфер
  blpg_load_loop:
    clr r28
    rcall uart_read_crc
    mov r0, r16
    rcall uart_read_crc
    .ifdef UCSR0A
      andi r28, (1 << FE0) | (1 << DOR0) | (1 << UPE0)
    .else
      andi r28, (1 << FE) | (1 << DOR) | (1 << UPE)
    .endif
    brne bler_and_loop // Если нет стоп-бита, или ошибка бита чётности, или пропущен байт, прерываем чтение страницы
    mov r1, r16
    ldi r16, (1 << SPMEN)
    rcall do_spm
    adiw r30, 2
    dec r19
  brne blpg_load_loop

  // Проверка контрольной суммы
  rcall uart_read_crc
  tst r20
  brne bler_and_loop

  sbrc r18, STATE_FLASHING
  rjmp blpg_state_continue
    // Если только начали прошивку, то устанавливаем маркер
    sbr r18, (1 << STATE_FLASHING)
    .ifdef MAGIC
      ldi r16, 0xFF
      rcall write_magic
    .endif
  blpg_state_continue:
  .if BLINKER
    ldi r27, BLINKER_PATTERN_FLASHING
  .endif

  // Стираем страницу из флеш и загружаем новую
  subi r30, low(PAGESIZE * 2)
  sbci r31, high(PAGESIZE * 2)

  ldi r16, (1 << PGERS) | (1 << SPMEN)
  rcall do_spm
  
  ldi r16, (1 << PGWRT) | (1 << SPMEN)
  rcall do_spm

  ldi r22, 'p'
  ldi r23, 'g'
  rcall uart_blrep

  mov r16, r2 // low(page number)
  rcall uart_write
  mov r16, r3 // high(page number)
  rcall uart_write
rjmp main_loop

.ifdef MAGIC
  // Загружает в EEARH:EEARL адрес MAGIC
  // использует r17
  load_magic_eear:
    ldi r17, high(MAGIC_EEPROM_ADDRESS)
    outp EEARH, r17
    ldi r17, low(MAGIC_EEPROM_ADDRESS);
    outp EEARL, r17
  ret

  // r16 - сохраняемое значение
  // использует r17
  write_magic:
    rcall load_magic_eear
    outp EEDR, r16
    .ifdef EEMWE
      sbi EECR, EEMWE
      sbi EECR, EEWE
    .else
      sbi EECR, EEMPE
      sbi EECR, EEPE
    .endif
  // сразу проваливаемся на ожидание окончания записи
.endif // MAGIC
wait_eeprom_finish:
  .ifdef EEWE
    sbic EECR, EEWE
  .else
    sbic EECR, EEPE
  .endif
  rjmp wait_eeprom_finish
ret

.if BLINKER || CONNECT_TIMEOUT
// Увеличивает значение r25:r24 и применяет мигание (из регистра r27) 
  update_counter:
    .ifdef TIFR1
      sbis_r TIFR1, TOV1, r16
    .else
      sbis_r TIFR, TOV1, r16
    .endif
  update_counter_exit:
    ret
    .ifdef TIFR1
      sbi_r TIFR1, TOV1, r16
    .else
      sbi_r TIFR, TOV1, r16
    .endif
    adiw r24, 1
    .if CONNECT_TIMEOUT
      .if BLINKER
        brne update_blinker
        cpi r18, (1 << STATE_ENTER)
        brne update_blinker
      .else
        brne update_counter_exit
        cpi r18, (1 << STATE_ENTER)
        brne update_counter_exit
      .endif
      rjmp jump_to_firmware
    .endif // CONNECT_TIMEOUT
  .if BLINKER
    update_blinker:
      movw r16, r24
      andi r16, 0xF0
      andi r17, 0x0F
      or r16, r17
      swap r16
      and r16, r27
      brne clear_blinker
        sbi BLINKER_PORT, BLINKER_PIN_NUM
        ret
      clear_blinker:
        cbi BLINKER_PORT, BLINKER_PIN_NUM
        ret
  .endif // BLINKER
.endif // BLINKER || CONNECT_TIMEOUT

// Чтение из UART
// Ожидает байт и возвращает его в r16, увеличивает значение r25:r24 и применяет мигание (из регистра r27)
uart_read:
  .if BLINKER || CONNECT_TIMEOUT
    .ifdef UCSR0A
      inp r16, UCSR0A
      sbrc r16, RXC0
    .else
      sbic UCSRA, RXC
    .endif
    rjmp uart_do_read
    rcall update_counter
    rjmp uart_read
  .else // !(BLINKER || CONNECT_TIMEOUT)
    .ifdef UCSR0A
      inp r16, UCSR0A
      sbrs r16, RXC0
    .else
      sbis UCSRA, RXC
    .endif
    rjmp uart_read
  .endif // !(BLINKER || CONNECT_TIMEOUT)
  uart_do_read:
  .ifdef UCSR0A
    inp r16, UCSR0A
    or  r28, r16
    inp r16, UDR0
  .else
    inp r16, UCSRA
    or  r28, r16
    inp r16, UDR
  .endif
ret

// Чтение из UART и обновление контрольной суммы
// Вызывает uart_read, который ожидает байт и возвращает его в r16, увеличивает значение r25:r24 и применяет мигание (из регистра r27)
// Прочитанным значением обновляется контрольная сумма в регистре r20
// Использует r22, r23
uart_read_crc:
  rcall uart_read
  eor r20, r16
  ldi r22, 8
  ldi r23, 0xCF
  crc_loop:
    lsl r20
    brcc crc_no_xor
      eor r20, r23
    crc_no_xor:
    dec r22
  brne crc_loop
ret 

// Запись в UART
// r16 - передаваемый байт
// Использует r17
uart_write:
  .ifdef UCSR0A
    inp r17, UCSR0A
    sbrs r17, UDRE0
    rjmp uart_write
    outp UDR0, r16
  .else
    sbis UCSRA, UDRE
    rjmp uart_write
    outp UDR, r16
  .endif
ret

uart_blrep:
  ldi r16, 'b'
  rcall uart_write
  ldi r16, 'l'
  rcall uart_write
  mov r16, r22
  rcall uart_write
  mov r16, r23
rjmp uart_write

do_spm: // на входе в r16 - значение SPMCSR
  .ifdef SPMCR
    outp SPMCR, r16
    spm
    wait_spm:
      inp r17, SPMCR
      sbrc r17, SPMEN
    rjmp wait_spm
  .else
    outp SPMCSR, r16
    spm
    wait_spm:
      inp r17, SPMCSR
      sbrc r17, SPMEN
    rjmp wait_spm
  .endif
ret

.if UART_AUTOBAUD
wait_for_line_high:
  set
  rjmp wait_for_line_state

wait_for_line_low:
  clt
  // continue execution in wait_for_line_state

/*
 * Waits until rx line is in a desired state or a timeout elapses.
 * On entry:
 *   if flag T = 1 waits until the line is high, if flag T = 0 waits until the line is low
 *   r23:r22:r21:r20 = timeout in ticks, if r23 = 0xFF then infinite timeout
 *   r31:r30 = pointer to store the counters, can be null
 * On exit:
 *   r3:r2:r1:r0 = ticks elapsed
 *   flag Z = 0 if the line is in a desired state and the counters are stored
 *   flag Z = 1 if a timeout elapsed
 * Uses/modifies:
 *   r0, r1, r2, r3, r16, r17, r19, r28, 29, r30, r31
 */
wait_for_line_state:
  inp r28, TCNT1L
  inp r29, TCNT1H
  clr r0
  clr r1
  movw r2, r0
  clr r19
  .if BLINKER || CONNECT_TIMEOUT
    rjmp wait_for_line_state_loop_enter
  .endif
  wait_for_line_state_loop:
    .if BLINKER || CONNECT_TIMEOUT
      rcall update_counter
    .endif
    wait_for_line_state_loop_enter:
    inp r16, TCNT1L
    inp r17, TCNT1H
    sub r16, r28
    sbc r17, r29
    add r28, r16
    adc r29, r17
    add r0, r16
    adc r1, r17
    adc r2, r19
    adc r3, r19
    cpi r23, 0xFF
    breq wait_for_line_state_check
      cp  r0, r20
      cpc r1, r21
      cpc r2, r22
      cpc r3, r23
      brsh wait_for_line_state_timeout
    wait_for_line_state_check:
    brts wait_for_line_state_high
      sbic UART_PIN, UART_RX_PIN
      rjmp wait_for_line_state_loop
      rjmp PC+1
      sbic UART_PIN, UART_RX_PIN
      rjmp wait_for_line_state_loop
      rjmp wait_for_line_state_loop_exit
    wait_for_line_state_high:
      sbis UART_PIN, UART_RX_PIN
      rjmp wait_for_line_state_loop
      rjmp PC+1
      sbis UART_PIN, UART_RX_PIN
      rjmp wait_for_line_state_loop
  wait_for_line_state_loop_exit:
  mov r19, r30
  or  r19, r31
  breq wait_for_line_state_exit
    inp r16, TCNT1L
    inp r17, TCNT1H
    st  Z+, r16
    st  Z+, r17
    st  Z+, r0
    st  Z+, r1
    st  Z+, r2
    st  Z+, r3
  wait_for_line_state_exit:
  clz
  ret
  wait_for_line_state_timeout:
  sez
  ret

/*
 * Waits until rx line becomes high and remains high during a given period or a timeout elapses.
 * On entry:
 *   r23:r22:r21:r20 = period/timeout in ticks, cannot be infinite
 * On exit:
 *   flag Z = 0 if the line is high during a given period
 *   flag Z = 1 if a timeout elapsed
 * Uses/modifies:
 *   r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r16, r17, r19, r28, 29, r30, r31
 */
wait_for_silence:
  clr r4
  clr r5
  movw r6, r4
  movw r8, r20
  movw r10, r22
  movw r30, r4
  wait_for_silence_loop:
    movw r20, r8
    movw r22, r10
    rcall wait_for_line_low
    breq wait_for_silence_exit
    add r4, r0
    adc r5, r1
    adc r6, r2
    adc r7, r3
    cp  r4, r20
    cpc r5, r21
    cpc r6, r22
    cpc r7, r23
    brsh wait_for_silence_timeout
    sub r20, r4
    sbc r21, r5
    sbc r22, r6
    sbc r23, r7
    rcall wait_for_line_high
    breq wait_for_silence_timeout
    add r4, r0
    adc r5, r1
    adc r6, r2
    adc r7, r3
  rjmp wait_for_silence_loop
  wait_for_silence_exit:
  clz
  ret
  wait_for_silence_timeout:
  sez
  ret

/*
 * Code size reduction.
 * Loads 10 msec timeout expressed in ticks into r23:r22:r21:r20.
 */
load_timeout_10_msec:
  ldi r20, low(TIMEOUT_10_MSEC)
  ldi r21, high(TIMEOUT_10_MSEC)
  ldi r22, byte3(TIMEOUT_10_MSEC)
  ldi r23, byte4(TIMEOUT_10_MSEC)
ret

/*
 * Waits for the synchro pattern and recognizes it.
 * On entry:
 *   r23:r22:r21:r20 = timeout in ticks, if r23 = 0xFF then infinite timeout
 *   r5:r4 = previously computed ubrr value or 0 if not computed yet
 * On exit:
 *   r5:r4 = updated ubrr value, will become 0 if synchro pattern is invalid
 *   flag Z = 0 if the synchro pattern is recognized and ubrr value is successfully updated
 *   flag Z = 1 if a timeout elapsed (line remains idle) or the synchro pattern is invalid
 * Uses/modifies:
 *   r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r16, r17, r19, r20, r21, r22, r23, r26, r28, 29, r30, r31
 */
recognize_pattern:
  ldi r30, low(counters)
  ldi r31, high(counters)
  rcall wait_for_line_low // wait for start bit (=0) start
  breq recognize_pattern_timeout
  rcall load_timeout_10_msec
  ldi r26, 4
  recognize_pattern_loop:
    rcall wait_for_line_high // wait for bit 0, 2, 4, 6 (=1) start
    breq recognize_pattern_not_recognized
    rcall wait_for_line_low // wait for bit 1, 3, 5, 7 (=0) start
    breq recognize_pattern_not_recognized
    dec r26
  brne recognize_pattern_loop
  rcall wait_for_line_high // wait for stop or parity bit (=1) start
  breq recognize_pattern_not_recognized
  sbiw r30, 60 // "rewind" pointer to the beginning of the counters array
  rcall analyze_counters
  brne recognize_pattern_recognized
  recognize_pattern_not_recognized:
    clr r4
    clr r5
  recognize_pattern_recognized:
  recognize_pattern_timeout:
ret

/*
 * Analyzes counters and updates ubrr value.
 * On entry:
 *   r5:r4 = previously computed ubrr value or 0 if not computed yet
 *   r31:r30 = pointer to the counters
 * On exit:
 *   r5:r4 = updated ubrr value
 *   flag Z = 0 if the synchro pattern is recognized and ubrr value is successfully updated
 *   flag Z = 1 if the synchro pattern is invalid
 * Uses/modifies:
 *   r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, r11, r16, r17, r19, r20, r21, r22, r23, r26, r28, 29, r30, r31
 */
analyze_counters:
  ld  r20, Z
  ldd r21, Z+1
  ldd r28, Z+6
  ldd r29, Z+6+1
  sub r28, r20
  sbc r29, r21
  movw r6, r28
  clr r19
  clr r0
  clr r1
  clr r2
  ldi r26, 8
  analyze_counters_loop:
    ld  r20, Z
    ldd r21, Z+1
    ldd r22, Z+6
    ldd r23, Z+6+1
    sub r22, r20
    sbc r23, r21
    cp  r28, r22
    cpc r29, r23
    brcc analyze_counters_skip_change_max
      movw r28, r22
    analyze_counters_skip_change_max:
    cp  r22, r6
    cpc r23, r7
    brcc analyze_counters_skip_change_min
      movw r6, r22
    analyze_counters_skip_change_min:
    ldd r8, Z+6+2
    ldd r9, Z+6+3
    ldd r10, Z+6+4
    ldd r11, Z+6+5
    cp  r8, r22
    cpc r9, r23
    cpc r10, r19
    cpc r11, r19
    brcc analyze_counters_pattern_is_invalid
    add r0, r22
    adc r1, r23
    adc r2, r19
    adiw r30, 6
    dec r26
  brne analyze_counters_loop
  sub r28, r6
  sbc r29, r7
  cpi r28, 70
  cpc r29, r19
  brcc analyze_counters_pattern_is_invalid
  .if UART_2X == 1
    ldi r26, 32
  .else
    ldi r26, 64
  .endif
  add r0, r26
  adc r1, r19
  adc r2, r19
  add r0, r0
  adc r1, r1
  adc r2, r2
  .if UART_2X == 1
    add r0, r0
    adc r1, r1
    adc r2, r2
  .endif
  ldi r26, 1
  sub r1, r26
  sbc r2, r19
  cp  r4, r1
  cpc r5, r2
  brcc analyze_counters_skip_change_ubrr
    mov r4, r1
    mov r5, r2
  analyze_counters_skip_change_ubrr:
  clz
  ret
  analyze_counters_pattern_is_invalid:
  sez
  ret

/*
 * Waits for the synchro pattern and computes ubrr value.
 * On exit:
 *   r5:r4 = computed ubrr value (0 if not computed)
 *   flag Z = 0 if the synchro pattern is recognized and ubrr value is computed
 *   flag Z = 1 if the synchro pattern is not recognized (some other activity on the line or the line remains idle)
 * Uses/modifies:
 *   r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r16, r17, r19, r20, r21, r22, r23, r26, r28, 29, r30, r31
 */
autobaud:
  rcall load_timeout_10_msec
  rcall wait_for_silence
  breq autobaud_exit
  clr r4
  clr r5
  ldi r23, 0xFF // infinite timeout
  autobaud_loop:
    rcall recognize_pattern
    breq autobaud_exit_loop
    ldi r20, low(TIMEOUT_200_MSEC)
    ldi r21, high(TIMEOUT_200_MSEC)
    ldi r22, byte3(TIMEOUT_200_MSEC)
    ldi r23, byte4(TIMEOUT_200_MSEC)
  rjmp autobaud_loop
  autobaud_exit_loop:
  mov r16, r4
  or  r16, r5
  autobaud_exit:
ret
.endif // UART_AUTOBAUD

// Очищает регистры портов
// На выходе r16 == 0
clear_ports:
  clr r16
  .if (defined(PORTE) && (PC > (FLASHEND - 24))) // Если у нас слишком дофига портов, а памяти уже не осталось...
     // ...то обнулим только те порты, которые используются загрузчиком
    .if BLINKER 
      outp BLINKER_DDR, r16
      outp BLINKER_PORT, r16
    .endif
    .ifdef FORCE_METHOD
      .if FORCE_METHOD
         outp FORCE_DDR, r16
         outp FORCE_PORT, r16
        .if FORCE_METHOD >= 3 
           outp FORCE_DDR2, r16
           outp FORCE_PORT2, r16
        .endif
      .endif
    .endif
    .if UART_AUTOBAUD
      outp UART_DDR, r16
      outp UART_PORT, r16
    .endif
  .else 
    // Если есть куда плясать - обнулим все порты
    .ifdef PORTA
      outp DDRA, r16
      outp PORTA, r16
    .endif
    .ifdef PORTB
      outp DDRB, r16
      outp PORTB, r16
    .endif
    .ifdef PORTC
      outp DDRC, r16
      outp PORTC, r16
    .endif
    .ifdef PORTD
      outp DDRD, r16
      outp PORTD, r16
    .endif
    .ifdef PORTE
      outp DDRE, r16
      outp PORTE, r16
    .endif
    .ifdef PORTF
      outp DDRF, r16
      outp PORTF, r16
    .endif
    .ifdef PORTG
      outp DDRG, r16
      outp PORTG, r16
    .endif
  .endif
ret

// bootloader_id возвращается в ответе на команду BLST, содержит ровно 18 байт. Первые три байта - размер страницы (в словах) и количество доступных страниц
// bootloader_id is returned in reply to BLST, contains exactly 18 bytes. First three bytes are: page size (in words) and number of pages available
bootloader_id: 
  .db  PAGESIZE, low(PAGES_AVAILABLE), high(PAGES_AVAILABLE), HARDWARE_ID
