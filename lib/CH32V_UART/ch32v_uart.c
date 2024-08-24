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
 *  file         : ch32v_uart.c
 *  description  : uart library main code
 *
 */

#include "ch32v_uart.h"

#define size(a)    (sizeof(a) / sizeof(*(a)))
// Translator functions: Translate local position inside USARTx RxBuffer to global position in RxBuffer for all USARTs
#define ADDR_RX_BUF(usart_num, a)    ((usart_num * UART_RX_BUFSIZE) + a)

#if defined(CH32V00X)
    uint8_t RxBuffer[1 * UART_RX_BUFSIZE] = {0};    // Partition RxBuffer into 1 Chunks, each for one USART port
#elif defined(CH32V10X)
    uint8_t RxBuffer[3 * UART_RX_BUFSIZE] = {0};    // Partition RxBuffer into 3 Chunks, each for one USART port
#else
    uint8_t RxBuffer[4 * UART_RX_BUFSIZE] = {0};    // Partition RxBuffer into 4 Chunks, each for one USART port
#endif
volatile uint16_t RxWritePtr[4] = {ADDR_RX_BUF(0,0),ADDR_RX_BUF(1,0),ADDR_RX_BUF(2,0),ADDR_RX_BUF(3,0)};    // Track where to write into RxBuffer1,2,3,4
volatile uint16_t RxReadPtr[4] = {ADDR_RX_BUF(0,0),ADDR_RX_BUF(1,0),ADDR_RX_BUF(2,0),ADDR_RX_BUF(3,0)};     // Track where to read from RxBuffer1,2,3,4

#if (UART_RX_HALT_FULL == TRUE)
volatile uint16_t RxFill[4] = {0,0,0,0};        // Track "fullness" of RxBuffer1,2,3,4
#endif

void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
#if !defined(CH32V00X)
    void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    #if !defined(CH32V10X)
        void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
    #endif
#endif

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
void uart_init_base(const UART_NUM fd, uint32_t baudrate, uint8_t stopbits, uint8_t parity)
{
    USART_InitTypeDef USART_InitStructure = {0};
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    /* Number of stop bits (0: 1 stop bit; 1: 1.5 stop bits; 2: 2 stop bits). */
    if( stopbits == 1 )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    }
    else if( stopbits == 2 )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }
    else
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }

    /* Check digit (0: None; 1: Odd; 2: Even; 3: Mark; 4: Space); */
    if( parity == 1 )
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
    else if( parity == 2 )
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
    else
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    switch (fd)
    {
        case UART1:
            #if defined(CH32X035) || defined(CH32X033)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
            #elif defined(CH32V20X) || defined(CH32V10X) || defined(CH32V30X)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
            #elif defined(CH32V00X)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
            #endif
            USART_Init(USART1, &USART_InitStructure);
            // Enable RXNE Interrupt
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
            NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            // Enable USART1
            USART_Cmd(USART1, ENABLE);
            break;
        case UART2:
            #if !defined(CH32V00X)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOA, &GPIO_InitStructure );
                USART_Init(USART2, &USART_InitStructure);
                // Enable RXNE Interrupt
                USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
                NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
                // Enable USART2
                USART_Cmd(USART2, ENABLE);
            #endif
            break;
        case UART3:
            #if (defined(CH32X035) || defined(CH32X033)) && !defined(CH32V00X)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                USART_Init(USART3, &USART_InitStructure);
                // Enable RXNE Interrupt
                USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
                NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
                // Enable USART3
                USART_Cmd(USART3, ENABLE);
            #elif (defined(CH32V20X) || defined(CH32V10X) || defined(CH32V30X)) && !defined(CH32V00X)
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                USART_Init(USART3, &USART_InitStructure);
                // Enable RXNE Interrupt
                USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
                NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
                // Enable USART3
                USART_Cmd(USART3, ENABLE);
            #endif
            break;
        case UART4:
            #if (defined(CH32X035) || defined(CH32X033) || defined(CH32V20X)) && (!defined(CH32V00X) && !defined(CH32V10X))
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOB, &GPIO_InitStructure );
                USART_Init(USART4, &USART_InitStructure);
                // Enable RXNE Interrupt
                USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
                NVIC_InitStructure.NVIC_IRQChannel = USART4_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
                // Enable USART4
                USART_Cmd(USART4, ENABLE);
            #elif defined(CH32V30X) && (!defined(CH32V00X) && !defined(CH32V10X))
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART4, ENABLE);
                // Enable RX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
                GPIO_Init( GPIOC, &GPIO_InitStructure );
                // Enable TX Pin
                GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
                GPIO_Init( GPIOC, &GPIO_InitStructure );
                USART_Init(USART4, &USART_InitStructure);
                // Enable RXNE Interrupt
                USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
                NVIC_InitStructure.NVIC_IRQChannel = USART4_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
                // Enable USART4
                USART_Cmd(USART4, ENABLE);
            #endif
            break;
    }
}

void uart_init_var(uart_init_args in)
{
    uint8_t stopbits_out = in.stopbits ? in.stopbits : 0;
    uint8_t parity_out = in.parity ? in.parity : 0;
    return uart_init_base(in.fd, in.baudrate, stopbits_out, parity_out);
}

/*********************************************************************
 * @fn      uart_writeByte
 *
 * @brief   Synchronously write a single byte into UART port
 * 
 * @param   fd          Which port to write into, either: UART1, UART2, UART3, UART4
 * @param   buf         Pointer to buffer of byte
 *
 * @return  None
 */
void uart_writeByte(const UART_NUM fd, uint8_t *buf)
{
    switch (fd)
    {
        case UART1:
            while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
            USART_SendData(USART1, *buf);
            break;
        #if !defined(CH32V00X)
        case UART2:
            while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
            USART_SendData(USART2, *buf);
            break;
        case UART3:
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
            USART_SendData(USART3, *buf);
            break;
        #if !defined(CH32V10X)
        case UART4:
            while(USART_GetFlagStatus(USART4, USART_FLAG_TC) == RESET);
            USART_SendData(USART4, *buf);
            break;
        #endif
        #endif
    }
}

/*********************************************************************
 * @fn      uart_putchar
 *
 * @brief   Synchronously write a single char into UART port
 * 
 * @param   fd          Which port to write into, either: UART1, UART2, UART3, UART4
 * @param   buf         Pointer to buffer of char
 *
 * @return  None
 */
void uart_putchar(const UART_NUM fd, char *buf)
{
    uart_writeByte(fd, (uint8_t*)buf);
}

/*********************************************************************
 * @fn      uart_writeBuffer
 *
 * @brief   Synchronously write a byte buffer into UART port
 * 
 * @param   fd          Which port to write into, either: UART1, UART2, UART3, UART4
 * @param   buf         Pointer to buffer of bytes
 * @param   length      Number of bytes to write
 *
 * @return  Number of bytes written
 */
uint16_t uart_writeBuffer(const UART_NUM fd, uint8_t *buf, uint16_t length)
{
    uint16_t i = 0;
    switch (fd)
    {
        case UART1:
            for(i = 0; i < length; i++)
            {
                while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
                USART_SendData(USART1, *buf++);
            }
            break;
        #if !defined(CH32V00X)
        case UART2:
            for(i = 0; i < length; i++)
            {
                while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
                USART_SendData(USART2, *buf++);
            }
            break;
        case UART3:
            for(i = 0; i < length; i++)
            {
                while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
                USART_SendData(USART3, *buf++);
            }
            break;
        #if !defined(CH32V10X)
        case UART4:
            for(i = 0; i < length; i++)
            {
                while(USART_GetFlagStatus(USART4, USART_FLAG_TC) == RESET);
                USART_SendData(USART4, *buf++);
            }
            break;
        #endif
        #endif
    }
    return i;
}

/*********************************************************************
 * @fn      uart_printf
 *
 * @brief   Synchronous printf into UART
 * 
 * @param   fd          Which port to write into, either: UART1, UART2, UART3, UART4
 * @param   format      Formatted string to write, additional arguments for variables
 *
 * @return  Number of characters parsed
 */
uint16_t uart_printf(const UART_NUM fd, char *format, ...)
{
    va_list args;
    char buffer[UART_TX_BUFSIZE];
    uint16_t count = 0;

    va_start(args, format);

    count += (uint16_t)vsnprintf(buffer, sizeof(buffer), format, args);     // convert to string and write into buffer
    uart_writeBuffer(fd, (uint8_t*)buffer, count);                          // write string to UART TX

    va_end(args);
    return count;
}

/*********************************************************************
 * @fn      uart_disable_irqRXNE
 *
 * @brief   Disable UART RXNE interrupt
 * 
 * @param   fd          Which port to disable interrupt for, either: UART1, UART2, UART3, UART4
 *
 * @return  None
 */
void uart_disable_irqRXNE(const UART_NUM fd)
{
    switch (fd)
    {
        case UART1:
            USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
            break;
        #if !defined(CH32V00X)
        case UART2:
            USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
            break;
        case UART3:
            USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
            break;
        #if !defined(CH32V10X)
        case UART4:
            USART_ITConfig(USART4, USART_IT_RXNE, DISABLE);
            break;
        #endif
        #endif
    }
}

/*********************************************************************
 * @fn      uart_enable_irqRXNE
 *
 * @brief   Enable UART RXNE interrupt
 * 
 * @param   fd          Which port to enable interrupt for, either: UART1, UART2, UART3, UART4
 *
 * @return  None
 */
void uart_enable_irqRXNE(const UART_NUM fd)
{
    switch (fd)
    {
        case UART1:
            USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
            break;
        #if !defined(CH32V00X)
        case UART2:
            USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
            break;
        case UART3:
            USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
            break;
        #if !defined(CH32V10X)
        case UART4:
            USART_ITConfig(USART4, USART_IT_RXNE, ENABLE);
            break;
        #endif
        #endif
    }
}

/*********************************************************************
 * @fn      uart_readByte
 *
 * @brief   Attempt to read a byte from UART port if new data available (Asynchronously).
 * 
 * @param   fd          Which port to read from, either: UART1, UART2, UART3, UART4
 * @param   buf         Buffer pointer to write into.
 *
 * @return  Return 1 if byte was read, return 0 if not
 */
uint8_t uart_readByte(const UART_NUM fd, uint8_t *buf)
{
    #if (UART_RX_HALT_FULL == FALSE)
        // As there is no "fullness" indicator in continous read mode,
        // there must be new data if read/write pointers do not align
        // (if writeptr makes lap around readptr position, discard previous UART_RX_BUFSIZE bytes)
        if (RxReadPtr[fd] != RxWritePtr[fd])
        {
            *buf = RxBuffer[RxReadPtr[fd]++];
            if (RxReadPtr[fd] >= ADDR_RX_BUF(fd, UART_RX_BUFSIZE))      // Handle Read Buffer overflow, use constants for best performance
            {
                RxReadPtr[fd] = ADDR_RX_BUF(fd, 0);
            }
            return 1;
        }
    #else
        // Fullness != 0 indicates new data available
        if (RxFill[fd])
        {
            *buf = RxBuffer[RxReadPtr[fd]++];
            RxFill[fd]--;
            if (RxReadPtr[fd] >= ADDR_RX_BUF(fd, UART_RX_BUFSIZE))      // Handle Read Buffer overflow, use constants for best performance
            {
                RxReadPtr[fd] = ADDR_RX_BUF(fd, 0);
            }
            return 1;
        }
    #endif
    return 0;
}

/*********************************************************************
 * @fn      uart_getch_locked
 *
 * @brief   Wait and read one char from UART port (Synchronously).
 * 
 * @param   fd          Which port to read from, either: UART1, UART2, UART3, UART4
 *
 * @return  Read character
 */
char uart_getch_locked(const UART_NUM fd)
{
    uint8_t read_byte;
    while(!uart_readByte(fd, &read_byte)) {};
    // Write read data into UART TX to give feedback to prompt
    #if(GETCH_CLI_FEEDBACK == TRUE)
        uart_writeByte(fd, &read_byte);
    #endif
    return (char)read_byte;
}

/*********************************************************************
 * @fn      uart_getch_unlocked
 *
 * @brief   Read one char from UART port if new data available (Asynchronously).
 * 
 * @param   fd          Which port to read from, either: UART1, UART2, UART3, UART4
 * @param   buf         Char buffer to read into
 *
 * @return  Return 1 if char was read, return 0 if not
 */
uint8_t uart_getch_unlocked(const UART_NUM fd, char *buf)
{
    return uart_readByte(fd, (uint8_t*)buf);
}

/*********************************************************************
 * @fn      uart_rx_flush
 *
 * @brief   Flush UART RX buffer, reset read/write pointer.
 * 
 * @param   fd          Which port to flush rx buffer for, either: UART1, UART2, UART3, UART4
 *
 * @return  None
 */
void uart_rx_flush(const UART_NUM fd)
{
    uart_disable_irqRXNE(fd);
    RxReadPtr[fd] = ADDR_RX_BUF(fd, 0);
    RxWritePtr[fd] = ADDR_RX_BUF(fd, 0);
    #if (UART_RX_HALT_FULL == TRUE)
        RxFill[fd] = 0;
    #endif
    uart_enable_irqRXNE(fd);
}

/*********************************************************************
 * @fn      uart_scanf
 *
 * @brief   Read UART data synchronously, read and wait till '\n' ends input.
 * 
 * @param   fd          Which port to read from, either: UART1, UART2, UART3, UART4
 * @param   format      Formatted string to read from, additional arguments for variables
 *
 * @return  Number of successfully parsed variables from input
 */
int uart_scanf(const UART_NUM fd, char *format, ...)
{
    va_list args;                               // For declaration of list or arguments
    char rx_buf[UART_SCANF_BUF_SIZE];            // Buffer for received data
    char current_char = 0;
    uint16_t readCnt = 0;

    uart_rx_flush(fd);

    while (current_char != '\n')
    {
        current_char = uart_getch_locked(fd);
        rx_buf[readCnt++] = current_char;
        if (readCnt >= UART_SCANF_BUF_SIZE)
        {
            break;          // If buffer full before delimiter reached, break anyway
        }
    }
    // Append 0 to tell parser where end of input buf string is (override delimiter '\n')
    rx_buf[readCnt - 1] = 0;

    // ---------- Parsing ----------
    va_start(args, format);                     // Here you are starting your list from the format
    int num_parsed = vsscanf(rx_buf, format, args);
    va_end(args);
    return num_parsed;
}

/*********************************************************************
 * @fn      USART1_IRQHandler
 *
 * @brief   This function handles USART1 global interrupt request.
 *
 * @return  none
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        #if (UART_RX_HALT_FULL == FALSE)
            RxBuffer[RxWritePtr[0]++] = (uint8_t)USART_ReceiveData(USART1);
        #else
            if (RxFill[0] < UART_RX_BUFSIZE)
            {
                RxBuffer[RxWritePtr[0]++] = (uint8_t)USART_ReceiveData(USART1);
                RxFill[0]++;
            }
        #endif

        if (RxWritePtr[0] >= ADDR_RX_BUF(0, UART_RX_BUFSIZE))      // Handle Write Buffer overflow, use constants for best performance
        {
            RxWritePtr[0] = ADDR_RX_BUF(0, 0);
        }
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

#if !defined(CH32V00X)
/*********************************************************************
 * @fn      USART2_IRQHandler
 *
 * @brief   This function handles USART2 global interrupt request.
 *
 * @return  none
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        #if (UART_RX_HALT_FULL == FALSE)
            RxBuffer[RxWritePtr[1]++] = (uint8_t)USART_ReceiveData(USART2);
        #else
            if (RxFill[1] < UART_RX_BUFSIZE)
            {
                RxBuffer[RxWritePtr[1]++] = (uint8_t)USART_ReceiveData(USART2);
                RxFill[1]++;
            }
        #endif

        if (RxWritePtr[1] >= ADDR_RX_BUF(1, UART_RX_BUFSIZE))      // Handle Write Buffer overflow, use constants for best performance
        {
            RxWritePtr[1] = ADDR_RX_BUF(1, 0);
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/*********************************************************************
 * @fn      USART3_IRQHandler
 *
 * @brief   This function handles USART3 global interrupt request.
 *
 * @return  none
 */
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        #if (UART_RX_HALT_FULL == FALSE)
            RxBuffer[RxWritePtr[2]++] = (uint8_t)USART_ReceiveData(USART3);
        #else
            if (RxFill[2] < UART_RX_BUFSIZE)
            {
                RxBuffer[RxWritePtr[2]++] = (uint8_t)USART_ReceiveData(USART3);
                RxFill[2]++;
            }
        #endif

        if (RxWritePtr[2] >= ADDR_RX_BUF(2, UART_RX_BUFSIZE))      // Handle Write Buffer overflow, use constants for best performance
        {
            RxWritePtr[2] = ADDR_RX_BUF(2, 0);
        }
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

#if !defined(CH32V10X)
/*********************************************************************
 * @fn      USART4_IRQHandler
 *
 * @brief   This function handles USART4 global interrupt request.
 *
 * @return  none
 */
void USART4_IRQHandler(void)
{
    if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET)
    {
        #if (UART_RX_HALT_FULL == FALSE)
            RxBuffer[RxWritePtr[3]++] = (uint8_t)USART_ReceiveData(USART4);
        #else
            if (RxFill[3] < UART_RX_BUFSIZE)
            {
                RxBuffer[RxWritePtr[3]++] = (uint8_t)USART_ReceiveData(USART4);
                RxFill[3]++;
            }
        #endif

        if (RxWritePtr[3] >= ADDR_RX_BUF(3, UART_RX_BUFSIZE))      // Handle Write Buffer overflow, use constants for best performance
        {
            RxWritePtr[3] = ADDR_RX_BUF(3, 0);
        }
        USART_ClearITPendingBit(USART4, USART_IT_RXNE);
    }
}

#endif
#endif