# ATENCIÓN BUGS

La velocidad de la UART no puede ser de 921600 en ninguna de las 3 usart del lpc4337.
El problema es SOLO a ese baud rate, se probo con un br=960000 y funciona perfectamente la comunicación.
Es por esto que la frecuencia de muestreo del poncho va a ser de 1khz para usar un br = 460800.

# CORRECION DEL BUG
Hay un error en prescaler del clk de la uart, elige automaticamente mal la frecuencia cuando le pones el
valor 460800. Se corrige usando el FDR (fractional divider register) con la función:

```C
uint32_t Chip_UART_SetBaudFDR(LPC_USART_T *pUART, uint32_t baud);
```

Esta función ya permite usar baudrates de hasta 921600 (probados). Se probo usar el poncho de biopotenciales 
a este baud rate y funcionó perfectamente con un sample rate de 2kHz.