# I2C Driver/ UART Driver with freeRTOS and STM32F103C8T6.

### How to use?
#### I2C:

* Add **JS_I2C1_Init()** before read/transmit the data. 
  * Initialize the control register(CR), clock control register(CCR) 
  * Set the priority of interrupt and enable it.
  * Set the open drain and ouput for I2C GPIO.
  
* **I2C_Write(blocking)/I2C_Write_IT(with interrupt)**: transmit the data through I2C.
* **I2C_Read(blocking)/I2C_Read_IT(with interrupt)**: recieve the data through I2C.
  * Example: hi2c1.enableIT = 0 (blocking) or 1(interrupt) for I2C1. You can check the default setting in main file.
  
#### UART:

* Add **JS_UART1_Init()** before read/transmit the data. 
  * Set baud rate register(BRR).
  * Initialize the control register(CR).
  * Set the priority of interrupt and enable it.
  * Set the push pull for UART GPIO.
 
* **UART_Write(blocking)/UART_Write_IT(with interrupt)**: transmit the data through UART.
* **UART_Read(blocking)/UART_Read_IT(with interrupt)**: recieve the data through UART.
  * Example: huart1.enableIT = 0 (blocking) or 1(interrupt) for uart1. You can check the default setting in main file.
  
#### example file: Main.c
Take IMU(MPU9250) and GPS(NEO-6M) as example and schedule these two tasks by freeRTOS.
