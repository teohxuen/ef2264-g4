# Adapted from: https://toptechboy.com/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/

import serial # import Serial Library
import matplotlib.pyplot as plt #import matplotlib library
from drawnow import *
 
temp= []
luminosity=[]

templim = [0,30] # change the y limit of temperature
luxlim = [0,1000] # change the y limit of luminosity

# change serial port here
arduinoData = serial.Serial('COM3', 115200) # Creating our serial object named arduinoData
plt.ion() # Tell matplotlib you want interactive mode to plot live data
cnt=0
 
def makeFig(): #Create a function that makes our desired plot
    plt.ylim(templim[0],templim[1])                             # Set y min and max values
    plt.title('Live Temperature and Luminosity Graph')          # Plot the title
    plt.grid(True)                                              # Turn the grid on
    plt.ylabel('Temp C')                                        # Set ylabels
    #plot the temperature "ro-" r=red, o=circle marker, -=solid line
    plt.plot(temp, 'ro-', label='Degrees')       
    plt.legend(loc='upper left')                                # plot the legend
    plt2=plt.twinx()                                            # Create a second y axis
    plt.ylim(luxlim[0],luxlim[1])                               # Set limits of second y axis- adjust to readings you are getting
    #plot the luminosity "b^-" b=blue, ^=triangle up marker, -=solid line
    plt2.plot(luminosity, 'b^-', label='Lux')               
    plt2.set_ylabel('Luminosity (Lux)')                         # label second y axis
    plt2.ticklabel_format(useOffset=False)                      # Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')                              # plot the legend
    

while True: # While loop that loops forever
    try:
        while (arduinoData.inWaiting()==0):                     # Wait here until there is data
            pass #do nothing
        arduinoString = arduinoData.readline().rstrip()         # read the line of text from the serial port
        arduinoString = arduinoString.decode()                  # change to string from binary
        dataArray = arduinoString.split(",")                    # Split it into an array called dataArray
        if (dataArray[0]!="LIVE"):                              # ignore data that is not meant for plotting
            continue
        print(dataArray)
        tempVal = float(dataArray[1])                           # Convert first element to floating number and put in temp
        luxVal =  float(dataArray[2])                           # Convert second element to floating number and put in lux
        temp.append(tempVal)                                    # Build our temp array by appending temp readings
        luminosity.append(luxVal)                               # Building our luminosity array by appending lux readings
        drawnow(makeFig) 
        plt.pause(.000001)                                      # Pause Briefly. Important to keep drawnow from crashing
        cnt=cnt+1
        if(cnt>50):                                             # If you have 50 or more points, delete the first one from the array
            temp.pop(0)                       
            luminosity.pop(0)
    except KeyboardInterrupt:                                   # Stop the programme if Ctrl+C is pressed
        break