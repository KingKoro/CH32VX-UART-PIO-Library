/**
 *  CH32VX UART Library
 *
 *  Copyright (c) 2024 Florian Korotschenko aka KingKoro
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *
 *  file         : ch32v_uart.h
 *  description  : uart library main header
 *
 */

#ifndef CH32V_UART_H_
#define CH32V_UART_H_

#include "stdio.h"
#include "debug.h"
#include "stdarg.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define TRUE    1
#define FALSE   0

// ---------- UART LIB USER CONFIG START ----------
#define UART_RX_HALT_FULL       FALSE           // Halt Rx when RxBuffer full if TRUE, continue and overwrite if FALSE (continous read mode)
#define UART_RX_BUFSIZE         512             // Size of RxBuffer in Number of Bytes
#define UART_SCANF_BUF_SIZE     512             // Size of temporary scanf buffer
#define UART_TX_BUFSIZE         512             // Size of temporary printf buffer
#define GETCH_CLI_FEEDBACK      TRUE            // Wether to print received char/string as feedback into UART TX
// ---------- UART LIB USER CONFIG END ----------

typedef enum UART
{
    UART1 = 0,
    UART2 = 1,
    UART3 = 2,
    UART4 = 3,
} UART_NUM;

// Initializer function
void uart_init_base(const UART_NUM fd, uint32_t baudrate, uint8_t stopbits, uint8_t parity);
// input structure for variadic args
typedef struct
{
    const UART_NUM fd;
    uint32_t baudrate;
    uint8_t stopbits;
    uint8_t parity;
} uart_init_args;
// placeholder for default args
void uart_init_var(uart_init_args in);
/*********************************************************************
 * @fn      uart_init_base
 *
 * @brief   Initialize UART port for communication.
 * 
 * @param   fd          Which port to initialize, either: UART1, UART2, UART3, UART4
 * @param   baudrate    Baudrate
 * @param   stopbits    Default = 0 = USART_StopBits_1, 1 = USART_StopBits_1_5, 2 = USART_StopBits_2
 * @param   parity      Default = 0 = USART_Parity_No, 1 = USART_Parity_Odd, 2 = USART_Parity_Even
 *
 * @return  None
 */
#define uart_init(...) uart_init_var((uart_init_args){__VA_ARGS__});

// Write functionality
extern void uart_writeByte(const UART_NUM fd, uint8_t *buf);
extern void uart_putchar(const UART_NUM fd, char *buf);
extern uint16_t uart_writeBuffer(const UART_NUM fd, uint8_t *buf, uint16_t length);

extern uint16_t uart_printf(const UART_NUM fd, char *format, ...);

// Read functionality
extern uint8_t uart_readByte(const UART_NUM fd, uint8_t *buf);
extern char uart_getch_locked(const UART_NUM fd);
extern uint8_t uart_getch_unlocked(const UART_NUM fd, char *buf);
extern void uart_rx_flush(const UART_NUM fd);
extern int uart_scanf(const UART_NUM fd, char *format, ...);

#ifdef __cplusplus
}
#endif

#endif /* CH32V_UART_H_ */