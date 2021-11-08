# ATENCIÓN BUGS

La velocidad de la UART no puede ser de 921600 en ninguna de las 3 usart del lpc4337.
El problema es SOLO a ese baud rate, se probo con un br=960000 y funciona perfectamente la comunicación.
Es por esto que la frecuencia de muestreo del poncho va a ser de 1khz para usar un br = 460800.