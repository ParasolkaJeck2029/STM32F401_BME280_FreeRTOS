
# Project loger of temperature, presure and humidaty 

This project is a example of use FreeRTOS on STM32F401 (BlackPill board). UART1 is used for transive data.
BME280 good sensor for reading temperature, pressure and humidaty, and it is connected to I2C1 of board.
For transive all data used queues and semaphors.


## Features

- Queue for UART and I2C
- Conection check 
- Using onboard button
- WatchDog enable 
- LED indication of work
- ADC read to UART


## 🚀 About Me
I'm a engineer of hydraulic and pneumatic systems. However my hobby is microcontrollers, now I work in STM32CubeIDE with two chips: STM32f401 and STM32H750.
Both chips instaled in board from WeAct Studio.


## Appendix

This project will have a future with second boart, on STM32H750 chip and more improver version. 
I want make project with two board, one as host and one as a slave. Host sends a SPI request and slave respond data from BME.

