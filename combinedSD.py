import os
import time
import sdcard
from machine import SPI,Pin
from machine import I2C,Pin

#initial time
init_time=time.ticks_ms()
T_o=init_time
#temperature sensor 
spi_T=machine.SPI(2, baudrate=400000, polarity=0, phase=1, bits=8,
firstbit=SPI.MSB)

cs_T = Pin('PB1', mode=Pin.OUT, value=0) #output pin configuration
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

# init SDcard SPI
sd = sdcard.SDCard(machine.SPI(1), machine.Pin('PA4'))
# mount sd card to uC system
os.mount(sd, '/sd')

file = open('/sd/measurement.txt', 'w')
file.write('Time; Temperature; Luminosity \n')
file.close()


while 1:
    #repeating the starting commands
    cs_T(1)
    spi_T.write(b'\x80') 
    spi_T.write(b'\x15')
    cs_T(0)
    time.sleep(0.15)
    
    #continous measurement
    cs_T(1)
    spi_T.write(b'\x03') # read in all registers
    rx1=spi_T.read(4,0x00) # read 4 bytes while continously writing the single byte
    cs_T(0)
    
    
    #conversion into °C temperature data
    temp = 0.0
    flag = int(rx1[2] >> 6) #right shift by 6 bits

    if rx1[1] >= 128: #cause 128 means -0
        temp = -(int(rx1[1]) - 128)-(flag * 0.25)
    else:
        temp = int(rx1[1])+(flag * 0.25)
    #time.sleep(2)
    
    
    
    #light sens
    te=i2c.readfrom(0x23, 2)
    
    #conversion into lux
    lux = te[0]*256
    lux += te[1]
    lux /= 1.2
    #time.sleep(2)
    
    T=time.ticks_ms()-init_time
    file = open('/sd/measurement.txt', 'a')
    write_msg =str(round(T/1000,3))+'; '+str(temp)+'; '+str(lux)+'\n'
    file.write(write_msg)
    file.close()
    timeDelta=str(T-T_o)
    T_o=T
    #file = open('/sd/measurement.txt', 'r')
    #data = file.read()
    #print(data)
    #file.close()
    
    print(write_msg.rstrip('\n')+" Time delta: "+timeDelta + '\n')
    
    """
    print("______________Temperature data:___________________")
    print("ID register: "+str(hex(rx1[0]))) # ID register
    print("MSB: "+ str(hex(rx1[1]))) # MSB
    print("LSB: "+ str(hex(rx1[2]))) # LSB
    print("Control reg: "+ str(hex(rx1[3]))) # Control reg
    print(str(temp)+" °C")
    
    
    print("____________Light sensor data:_____________________")
    print(te[0]) #high 
    print(te[1]) #low
    print(str(lux)+" lx")
    """
    
