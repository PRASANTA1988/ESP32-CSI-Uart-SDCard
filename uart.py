import RPi.GPIO as GPIO
import serial
import csv
import time
import math
import os

if __name__ == '__main__':
    standby=2
    movement=3
    #data_sel=23
    standby_led=4
    movement_led=27
    standby_flag=0
    movement_flag=0

    def standby_data(channel):
        global standby_flag,movement_flag
        if not movement_flag:
            #GPIO.output(data_sel,GPIO.HIGH)
            GPIO.output(standby_led,GPIO.HIGH)
            print("standby on\n")
            standby_flag=1

    def movement_data(channel):
        global standby_flag,movement_flag
        if not standby_flag:
            #GPIO.output(data_sel,GPIO.LOW)
            GPIO.output(movement_led,GPIO.HIGH)
            print("movement on\n")
            movement_flag=1

    def timestamp(tok):
        tm=time.localtime()
        tm_str1=str(tm.tm_year)+'-'+str(tm.tm_mon)+'-'+str(tm.tm_mday)+'/'
        tm_str=str(tm.tm_hour)+':'+str(tm.tm_min)+':'+str(tm.tm_sec)
        return(tm_str1,tm_str)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(standby,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    GPIO.setup(movement,GPIO.IN,pull_up_down=GPIO.PUD_UP)
    #GPIO.setup(data_sel,GPIO.OUT)
    #GPIO.output(data_sel,GPIO.LOW)
    GPIO.setup(standby_led,GPIO.OUT)
    GPIO.output(standby_led,GPIO.LOW)
    GPIO.setup(movement_led,GPIO.OUT)
    GPIO.output(movement_led,GPIO.LOW)
    GPIO.add_event_detect(standby,GPIO.FALLING,callback=standby_data,bouncetime=500)
    GPIO.add_event_detect(movement,GPIO.FALLING,callback=movement_data,bouncetime=500)
    
    # if connected via serial Pin(RX, TX)
    ser = serial.Serial('/dev/ttyAMA0', baudrate = 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1) #115200 is baud rate(must be same with that of NodeMCU)
    ser.flush()
    try:
        while True:
            if standby_flag:
                f_name,standby_time=timestamp(1)                              #time.ctime(time.time()+330*60)
                time.sleep(10)
                try:
                    os.mkdir(f_name)
                except:
                    pass
                standby_time=f_name+"Standby_"+standby_time+".csv"
                init_time=time.time()
                print(standby_time+"_"+str(init_time))
                f_standby=open(standby_time,"w+")
                standby_writer=csv.writer(f_standby)
                while (time.time()-init_time)<30:                
                    t=time.time()
                    #ls=[time.ctime(time.time()+330*60)]  #set according to time zone
                    ls=[time.ctime(time.time())]  #set according to time zone
                    #string = input("enter string:") #input from user
                    #string = "1" #"\n" for line seperation
                    #string = string.encode('utf_8')
                    #ser.write(string) #sending over UART
                    line = ser.readline().decode('utf-8').rstrip()
                    print("received: ",line)
                    ls=ls+[line]
                    standby_writer.writerow(ls)
                    time.sleep(math.fabs(.01-(time.time()-t)))
                GPIO.output(standby_led,GPIO.LOW)    
                standby_flag=0
                f_standby.close()
                print("stop")
            elif movement_flag:
                f_name, movement_time=timestamp(1)
                time.sleep(10)
                try:
                    os.mkdir(f_name)
                except:
                    pass
                movement_time=f_name+"Movement_"+movement_time+".csv"
                init_time=time.time()
                f_movement=open(movement_time,"w+")
                movement_writer=csv.writer(f_movement)
                while (time.time()-init_time)<30:               
                    t=time.time()
                    #ls=[time.ctime(time.time()+330*60)]  #set according to time zone
                    ls=[time.ctime(time.time())]  #set according to time zone
                    #string = input("enter string:") #input from user
                    #string = "1" #"\n" for line seperation
                    #string = string.encode('utf_8')
                    #ser.write(string) #sending over UART
                    line = ser.readline().decode('utf-8').rstrip()
                    print("received: ",line)
                    ls=ls+[line]
                    movement_writer.writerow(ls)
                    time.sleep(math.fabs(.01-(time.time()-t)))
                GPIO.output(movement_led,GPIO.LOW)
                movement_flag=0
                f_movement.close()
                print("stop")
            time.sleep(1) #delay of 1 second               
    finally:
        GPIO.cleanup()   
    