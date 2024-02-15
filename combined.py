import os
import time
from machine import SPI,Pin
from machine import I2C,Pin

#temperature sensor 
spi_T=machine.SPI(1, baudrate=400000, polarity=0, phase=1, bits=8,
firstbit=SPI.MSB)

cs_T = Pin('PA4', mode=Pin.OUT, value=0) #output pin configuration
cs_T(0) #pin off
cs_T(1) #pin on

spi_T.write(b'\x80') # write register operation x80 adress byte
spi_T.write(b'\x14') # one shot 1, shut down 0 (wake up)
                     # converts into:0001 0100 which is OS=1 and SHDN=0
                     #--> that mean continous temperature conversion
cs_T(0)

i2c = I2C(1,freq=100000)
i2c.writeto(0x23, b'\x01') # power on
i2c.writeto(0x23, b'\x13') # cont. large range

while 1:
    #repeating the starting commands
    cs_T(1)
    spi_T.write(b'\x80') 
    spi_T.write(b'\x15')
    cs_T(0)
    time.sleep(0.5)
    
    #continous measurement
    cs_T(1)
    spi_T.write(b'\x03') # read in all registers
    rx1=spi_T.read(4,0x00) # read 4 bytes while continously writing the single byte
    cs_T(0)
    
    print("______________Temperature data:___________________")
    print("ID register: "+str(hex(rx1[0]))) # ID register
    print("MSB: "+ str(hex(rx1[1]))) # MSB
    print("LSB: "+ str(hex(rx1[2]))) # LSB
    print("Control reg: "+ str(hex(rx1[3]))) # Control reg
    
    #conversion into °C temperature data
    temp = 0.0
    flag = int(rx1[2] >> 6) #right shift by 6 bits
    print(rx1[1])
    print(flag)

    if rx1[1] >= 128: #cause 128 means -0
        temp = -(int(rx1[1]) - 128)-(flag * 0.25)
    else:
        temp = int(rx1[1])+(flag * 0.25)
        print("here")
    
    print(str(temp)+" °C")
    time.sleep(2)
    
    
    
    #light sens
    print("____________Light sensor data:_____________________")
    te=i2c.readfrom(0x23, 2)
    print(te[0]) #high 
    print(te[1]) #low
    
    #conversion into lux
    lux = te[0]*256
    print(lux)
    lux += te[1]
    lux /= 1.2
    print(str(lux)+" lx")
    time.sleep(2)
