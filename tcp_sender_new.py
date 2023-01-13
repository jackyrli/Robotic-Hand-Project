from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
#import esp32
from time import sleep
import time

blue = ADC(Pin(33)) #insead ofA0/26
green = ADC(Pin(32)) #instead of A1/25
brown = ADC(Pin(34)) #A2
maroon = ADC(Pin(39)) #A3
grey = ADC(Pin(36)) #A4

blue.atten(ADC.ATTN_11DB)       #Full range: 3.3v
blue.width(ADC.WIDTH_12BIT)
green.atten(ADC.ATTN_11DB)       #Full range: 3.3v
green.width(ADC.WIDTH_12BIT)
brown.atten(ADC.ATTN_11DB)       #Full range: 3.3v
brown.width(ADC.WIDTH_12BIT)
maroon.atten(ADC.ATTN_11DB)       #Full range: 3.3v
maroon.width(ADC.WIDTH_12BIT)
grey.atten(ADC.ATTN_11DB)       #Full range: 3.3v
grey.width(ADC.WIDTH_12BIT)

grey_min = 3260
blue_min = 3200
maroon_min = 2760
green_min = 3000
brown_min = 2830
all_max = 4095

angle_min = 0
angle_max = 90

def mapRange(value, inMin, inMax, angleMin, angleMax):
    return angleMin + (value - inMin) * ( (angleMax - angleMin)/(inMax - inMin) )

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn) 

def connect_to_internet(): #connect to wifi
    wlan = WLAN(STA_IF)
    wlan.active(True)
    wlan.connect('ME100-2.4G', '122Hesse')
    #'Meowmeow', 'whiskers2020', 5000
    #('2336wifi', 'jiaobaba') #这里放wlan.connect('ME100-2.4G', '122Hesse', 5000)
    while not wlan.isconnected(): # and tries < 10
        print("Waiting for wlan connection")
        time.sleep(1)
    print("WiFi connected at", wlan.ifconfig()[0])
    return wlan

def configure_tcp_sender(): #initialize server connection
    HOST = wlan.ifconfig()[0] #  192.168.4.116 #172.20.10.8
    PORT = 10
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    print('connected at host')
    s.listen(10)
    print('wait for connections at this server')
    (sc,addr) = s.accept()
    return s, sc, addr

if __name__ == "__main__":
    wlan = connect_to_internet()
    s, sc, addr = configure_tcp_sender()
    while True:
        if not wlan.isconnected:
            wlan = connect_to_internet()
            sc.close()
            s, sc, addr = configure_tcp_sender()
        else:
            # blue_val = str(blue.read())
            # green_val = str(green.read())
            # brown_val = str(brown.read())
            # maroon_val = str(maroon.read())
            # grey_val = str(grey.read())
            # print("blue", blue_val)
            # print("green",green_val)
            # print("brown",brown_val)
            # print("maroon",maroon_val)
            # print("grey",grey_val)

            blue_val = clamp(blue.read(), blue_min, all_max)
            green_val = clamp(green.read(), green_min, all_max)
            brown_val = clamp(brown.read(), brown_min, all_max)
            maroon_val = clamp(maroon.read(), maroon_min, all_max)
            grey_val = clamp(grey.read(), grey_min, all_max)

            thumb = str( round( mapRange(blue_val, blue_min, all_max, angle_min, angle_max), 1) )
            index = str( round( mapRange(maroon_val, maroon_min, all_max, angle_min, angle_max), 1) )
            middle = str( round( mapRange(brown_val, brown_min, all_max, angle_min, angle_max),1) )
            ring = str( round(mapRange(green_val, green_min, all_max, angle_min, angle_max),1) )
            pinkie = str( round(mapRange(grey_val, grey_min, all_max, angle_min, angle_max),1) )

            message = thumb + ',' + index + ',' + middle + ',' + ring + ',' + pinkie 
            #message = "new line"
            print("grey " , grey_val, "angle", pinkie )
            print("green " , green_val, "angle", ring )
            print("brown " , brown_val, "angle", middle )
            print("maroon " , maroon_val, "angle", index )
            print("blue " , blue_val, "angle", thumb)
            sleep(0.1)
             #IMPORTANT: Receive message of byte size=?
            #TODO: Need some decoding and encoding
            #TODO: Need actuation
            sc.send(message.encode())
            print('sent: ', message.encode())
    sc.close()
    s.close()
