from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
import esp32
import time

def connect_to_internet(): #connect to wifi
    wlan = WLAN(STA_IF)
    wlan.active(True)
    wlan.connect('2336wifi', 'jiaobaba') #这里放wlan.connect('ME100-2.4G', '122Hesse', 5000)
    while not wlan.isconnected(): # and tries < 10
        print("Waiting for wlan connection")
        time.sleep(1)
    print("WiFi connected at", wlan.ifconfig()[0])
    return wlan

def configure_tcp_sender(): #initialize server connection
    HOST = '192.168.4.62'
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
            message = 'hi'
             #IMPORTANT: Receive message of byte size=?
            #TODO: Need some decoding and encoding
            #TODO: Need actuation
            sc.send(message.encode())
            print('sent: ', message.encode())
    sc.close()
    s.close()