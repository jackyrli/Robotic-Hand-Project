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

#tcp connection
def configure_tcp_receiver(): #initialize server connection
    HOST = '192.168.4.93'
    PORT = 20
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    print('connected at host')
    s.listen(10)
    print('wait for connections at this server')
    (sc,addr) = s.accept()
    return s, sc, addr

def decode_message(message):
    message = message.decode()
    message = message.replace('\r\n','')
    ret = message.split(',')
    ret = [float(x) for x in ret]
    return ret

if __name__ == "__main__":
    wlan = connect_to_internet()
    s, sc, addr = configure_tcp_receiver()
    while True:
        if not wlan.isconnected:
            wlan = connect_to_internet()
            s.close()
            sc.close()
            s, sc, addr = configure_tcp_receiver()
        else:
            message = sc.recv(64) #IMPORTANT: Receive message of byte size=?
            #TODO: Need some decoding and encoding
            try:
                message = decode_message(message)
                print(type(message))
                print(message)
            except:
                s.close()
                sc.close()
                s, sc, addr = configure_tcp_receiver()
            #TODO: Need actuation
            
    sc.close()
    s.close()