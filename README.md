# This is the source for UM7 driver for STM32

The communication to the UM7 IMU is through Serial RS-232 UART. The uart libraries used are on this git hub [Kugle embedded](https://github.com/mindThomas/Kugle-Embedded)

## Development board 

The board used for developing this library is [Nucleo STM32H743ZI2](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html#sample-buy) <br/>
![Screenshot](nucleo.jpg)

## The main code developed is found under 

[UM7 IMU Driver](https://redshiftlabs.com.au/product/um7-lt-orientation-sensor/)

## Installation 

STMCubeMX IDE is used to specify the clock source for the periphirial in the input-output map. 

System Workbench IDE based on eclipse is used for coding. 

## UM7 IMU Product

![Screenshot](UM7.jpeg)<br/>
[UM7 IMU](https://redshiftlabs.com.au/product/um7-lt-orientation-sensor/)
[Datasheet](https://redshiftlabs.com.au/wp-content/uploads/2018/02/um7_datasheet_v1-6_10.1.2016.pdf)
