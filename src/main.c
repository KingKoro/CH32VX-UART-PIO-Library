#if defined(CH32V00X)
#include <ch32v00x.h>
#elif defined(CH32V10X)
#include <ch32v10x.h>
#elif defined(CH32V20X)
#include <ch32v20x.h>
#elif defined(CH32V30X)
#include <ch32v30x.h>
#elif defined(CH32X035) || defined(CH32X033)
#include <ch32x035.h>
#endif

#include "debug.h"
#include "ch32v_uart.h"
#include "ch32v_ftoa_atof.h"

#define TRUE 1
#define FALSE 0

int main()
{
    // ---------- Initialization Code ----------
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);
    uart_init(UART2, 921600);


    printf("CH32V UART CDC Test - Starting ...\r\n");
    // Setup Code USB Serial
    // Variables for testing CDC
    uint32_t counter1s = 0;
    char testchar = 'A';
    char teststring[] = "STRING";
    int testint = 187;
    double testfloatA = 456321.7651234;
    double testfloatB = -3.14159;
    char testfloatC[16];
    uint32_t testhex = 0xFDCA8421;
    // ---------- Endless Loop Code ----------
    while ( 1 )
    {
        if(counter1s >= 20000)
        {
            // Call roughly every ~second
            counter1s = 0;
            uart_printf(UART2, "1s Loop ... %c \r\n", testchar);
            //testchar = uart_getch_locked(UART2);
            //uart_getch_unlocked(UART2, &testchar);
            
            uart_printf(UART2, "This is a char: %c,\tThis is a string: %s,\tThis is an integer: %d,\tThis is a float: %s or %s,\tThis is a hex value (32bit): %x \tThis is a percent sign: %%\n", testchar, teststring, testint, ftoa_s(testfloatA, 6), ftoa_s(testfloatB, 5), testhex);
            uart_printf(UART2, "This is negative pi with 2 decimals: %s and now 4 decimals: %s \n", ftoa_s(testfloatB, 2), ftoa_s(testfloatB, 4));
            
            int hours = 0;
            int minutes = 0;
            int seconds = 0;
            uart_printf(UART2, "Please enter a time in format \"HH:MM:SS\" : ");
            int status_scanf = uart_scanf(UART2, "%d:%d:%d", &hours, &minutes, &seconds);
            uart_printf(UART2, "Received hours: %d !, statuscode: %d \n", hours, status_scanf);
            uart_printf(UART2, "Received minutes: %d !, statuscode: %d \n", minutes, status_scanf);
            uart_printf(UART2, "Received seconds: %d !, statuscode: %d \n", seconds, status_scanf);
            uart_printf(UART2, "Please enter a floating point value: ");
            status_scanf = uart_scanf(UART2, "%s", &testfloatC);
            double received_float = ratof(testfloatC);
            //double received_float = strtod(testfloatC, NULL);
            uart_printf(UART2, "Received float: %s !, statuscode: %d \n", ftoa_s(received_float, 6), status_scanf);
        }
        // Call on every loop
        counter1s++;
        //USB_Tx_runner();
    }
}
