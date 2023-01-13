from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule
from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
import esp32
import time

state = 0 # 1:bend; 0: stay, -1: strech
"""Connecting GPIO pins A0 and A1 to the signal-in of the H-Bridge"""
motor_vpin = Pin(25, mode=Pin.OUT) #bend
motor_gnd = Pin(26, mode=Pin.OUT)  #strech

def connect_to_internet(): #connect to wifi
    wlan = WLAN(STA_IF)
    wlan.active(True)
    wlan.connect('ME100-2.4G', '122Hesse') #这里放wlan.connect('ME100-2.4G', '122Hesse', 5000)
    while not wlan.isconnected(): # and tries < 10
        print("Waiting for wlan connection")
        time.sleep(1)
    print("WiFi connected at", wlan.ifconfig()[0])
    return wlan

#tcp connection
def configure_tcp_receiver(): #initialize server connection
    HOST = wlan.ifconfig()[0]
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


#useful functions
def configure_pwm_pins(v_pin, g_pin):
    v_ret = PWM(v_pin, freq=100000, duty=0) #100000
    g_ret = PWM(g_pin, freq=100000, duty=0)
    return v_ret, g_ret

def stay(v_pin, g_pin):
    global state
    if state == 0:
        return
    state = 0
    v_pin.duty(0)
    g_pin.duty(0)

def bend(v_pin, g_pin):
    global state
    global encoder_count
    global target
    state = 1
    #ret = int(600 + 10*(target[0]-encoder_count))
    #ret = min(max(700, ret), 800)
    print('bend',ret)
    v_pin.duty(900)
    g_pin.duty(0)
    
def strech(v_pin, g_pin):
    global state
    global encoder_count
    global target
    state = -1
    #ret = int(600 - 5*(target[0]-encoder_count))
    #ret = min(max(700, ret), 800)
    #print('strech', ret)
    v_pin.duty(0)
    g_pin.duty(700)
    
encoder_count = 0

def counter(pin):
    global encoder_count
    global state
    if state == 0:
        pass
    elif state == 1:
        encoder_count = encoder_count + 1
    else:
        encoder_count = encoder_count - 1

def report(timer):
    global encoder_count
    print('encoder_count', encoder_count)
    
"""See how many times the encoder is getting triggered"""
encoder_0 = Pin(34, mode=Pin.IN)
encoder_0.irq(handler=counter, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

"""Find the average speed"""
t1 = Timer(2)
t1.init(period=500, mode=t1.PERIODIC, callback=report)

# configure
#configure_pwm_pins(motor_vpin, motor_gnd)
L1, L2 = configure_pwm_pins(motor_vpin, motor_gnd)

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
            message = decode_message(message)
            target = message[2]
            target = target * 10
            target = [target-3, target+3]
            print('target: ', target)
            if encoder_count>= target[0] and encoder_count <= target[1]:
                stay(L1,L2)
            elif encoder_count < target[0]:
                bend(L1,L2)
            elif encoder_count > target[1]:
                strech(L1,L2)
#             except:
#                 s.close()
#                 sc.close()
#                 s, sc, addr = configure_tcp_receiver()
            #TODO: Need actuation

    sc.close()
    s.close()

