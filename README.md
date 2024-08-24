# CH32VX UART PIO Library
This repository contains a CH32Vx/CH32X0x PIO NoneOS library for serial communication over UART ports. It is supposed to work with CH32V10X, CH32V20X, CH32V30X, CH32X033 and CH32X035 MCUs.

# Warning
Due to issues with stdlib's ftoa and atof functions, this repository comes with an additional library ``CH32V_FTOA_ATOF`` to implement these conversions correctly.

# Installation
## Prequisites
You need to have PlatformIO VSCode Plugin with the [WCH CH32V](https://github.com/Community-PIO-CH32V/platform-ch32v) Platform installed.

## Setup
Simply clone this repository onto your computer and open the folder like a regular PlatformIO project.

Alternativly, you can also copy the library ```lib/CH32V_UART``` into any PIO project, modify your ```platformio.ini``` and import the library into your ```main.c``` with: 
```c
#include "ch32v_uart.h"
```
If needed, also copy over ``lib/CH32V_FTOA_ATOF`` into you PIO project and import it with
```c
#include "ch32v_ftoa_atof.h"
```


# Usage

Import ```ch32v_uart.h```, then modify the ```platform.ini``` environment by specifying the serial monitor speed.

Afterwards, edit the user config section (line 16 to 20) in ```lib/CH32V_UART/ch32v_uart.h``` to control the serial port settings. The following settings are available:

|       Option:       | Description:                                                           |
|:-----------------:|-----------------------------------------------------------------|
| UART_RX_HALT_FULL           |   Halt Rx when RxBuffer full if ``TRUE``, continue and overwrite if ``FALSE`` (continous read mode) Default = ``False``   |
| UART_RX_BUFSIZE         | Size of RxBuffer in Number of Bytes. Default = 512          |
| UART_SCANF_BUF_SIZE       | Size of temporary scanf buffer. Default = 512 |
| UART_TX_BUFSIZE     | Size of temporary printf buffer. Default = 512   |
| GETCH_CLI_FEEDBACK | Wether to print received char/string as feedback into UART TX. Default = ``TRUE``      |


For the ``CH32V_FTOA_ATOF`` library, configuration can be done in the same way (line 41 to 43 in ```lib/CH32V_FTOA_ATOF/ch32v_ftoa_atof.h```) with the following settings:

|       Option:       | Description:                                                           |
|:-----------------:|-----------------------------------------------------------------|
| TMP_FBUF_SIZE           |   HBuffer size to temporary store float strings after conversion in ftoa_s(). Default = 128   |
| TMP_FSTR_SIZE         |     Length of individial strings of floats (num of floats stored at once = TMP_FBUF_SIZE / TMP_FSTR_SIZE). Default = 16      |
| TMP_FSTR_NUM       | default is 8 slots for temporary float strings (automatically overwritten on overflow). Default = TMP_FBUF_SIZE / TMP_FSTR_SIZE |

Now you can compile and upload the project.

## Overview

The most important functions, exposed by the UART serial library, are:
```C
void uart_init(const UART_NUM fd, uint32_t baudrate, uint8_t stopbits = 0, uint8_t parity = 0)    /* UART serial port initialization */

void void uart_writeByte(const UART_NUM fd, uint8_t *buf)      /* Write a Byte into UART (Synchronously) */

void uart_putchar(const UART_NUM fd, char *buf)                                 /* putchar() */
uint16_t uart_writeBuffer(const UART_NUM fd, uint8_t *buf, uint16_t length)     /* write buffer with specified length */

uint16_t uart_printf(const UART_NUM fd, char *format, ...)   /* printf() */
uint8_t uart_readByte(const UART_NUM fd, uint8_t *buf)      /* read a single byte from UART (Asynchronously) */
char uart_getch_locked(const UART_NUM fd)                   /* read a single char from UART (Synchronously) */
uint8_t uart_getch_unlocked(const UART_NUM fd, char *buf)   /* read a single char form UART (Asynchronously) */
void uart_rx_flush(const UART_NUM fd)                       /* flush/reset RX buffer */
int int uart_scanf(const UART_NUM fd, char *format, ...)    /* scanf() */

// CH32V_FTOA_ATOF Library specific:

char * ftoa_s(double val, int precision)    /* compact ftoa() */
double ratof(char *arr)     /* atof() double */
float ratoff(char *arr)     /* atof() float */
```

# Notes

## Floating Point values
As of right now, the standard library for CH32V MCUs does not handle string-to-float and float-to-string conversion correctly. Therefore this library contains an adapted ftoa implementation by [Anton B. Gusev](https://github.com/antongus/stm32tpl/blob/master/ftoa.c).
The library additionally contains an atof implementation.

When using ```uart_scanf()``` and ```uart_printf()```, floating point values have to be passed as strings and then converted to double or float. To achive this in a more compact manner, ```char * ftoa_s(double val, int precision)``` is an inline version of ftoa for use in the same line as the printf statement. For an example, look at the proivded ```src/main.c```.

## 64-Bit Integers
As of right now, the standard library for CH32V MCUs does not handle string-to-(U)INT64 and (U)INT64-to-string conversion correctly. Therefore a custom ``atoi()`` and ``itoa()`` function needs to be supplied.

## Supported MCUs
This library should work on all CH32V or CH32X Series of MCUs.

# Disclaimer

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
