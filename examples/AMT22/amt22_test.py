import spidev                                  #import the SPI library for RB Pi 4 B board
import time                                    #import the Timing library for RB Pi 4 B board
import math					#import the Math library for RB Pi 4 B board

AMT22_NOP = 0x00                               #command to read the position of the encoder

spi = spidev.SpiDev()                          #create the spi object
spi.open(0, 0)                                 #SPI port 0, CS 0
speed_hz=500000                                #setting the speed in hz
delay_us=3                                     #setting the delay in microseconds

def checksum_ok(n):		#function for checking the checksum equation
    return (not(bool(n & (1<<(8+5))) ^ bool(n & (1<<(8+3))) ^ bool(n & (1<<(8+1))) ^ bool(n & (1<<7)) ^ bool(n & (1<<5)) ^ bool(n & (1<<3)) ^ bool(n & (1<<1)))==bool(n & (1<<15))) and (not(bool(n & (1<<(8+4))) ^ bool(n & (1<<(8+2))) ^ bool(n & (1<<8)) ^ bool(n & (1<<6)) ^ bool(n & (1<<4)) ^ bool(n & (1<<2)) ^ bool(n & (1<<0)))==bool(n & (1<<14)))

try:
    while True:          			#creating an infinite while loop
        result = spi.xfer2([AMT22_NOP, AMT22_NOP], speed_hz, delay_us)      #transferring the data from the encoder
        pos_data = result[0]<<8 | result[1]    # Concatenate two bytes into 16-bit value
        print(hex(pos_data))
        print(checksum_ok(pos_data))  # Check the checksum
        position = pos_data & 0x3FFF
        print (position)

        angle = position*(360/16384)   		#Using this equation to convert the recieved data to an angle[°]
        print (angle)						#printing the angular position
        
        
except: 								#In case while loop doesn't work
    print("done")
    spi.close()