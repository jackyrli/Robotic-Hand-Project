from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule
from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
import esp32
import time

class finger:
    def __init__(self, v_pin_num, g_pin_num, enc_pin_num1, enc_pin_num2, timer_num):
        self.connected = False
        self.state = 0 # 1:bend; 0: stay, -1: strech
        self.target = [-10,10]
        motor_vpin = Pin(v_pin_num, mode=Pin.OUT) #bend
        motor_gnd = Pin(g_pin_num, mode=Pin.OUT)  #strech
        self.direction = 0
        #define encoders
        self.encoder_0 = Pin(enc_pin_num1, mode=Pin.IN)
        self.encoder_1 = Pin(enc_pin_num2, mode=Pin.IN)
        
        self.encoder_0.irq(handler=self.counter, trigger=Pin.IRQ_RISING)
        
        self.v_pin, self.g_pin = self.configure_pwm_pins(motor_vpin, motor_gnd)
        
        
        #initialize initial position
        self.bend(init=True)
        sleep(1)
        self.stay()
        self.encoder_count = 0
        print(self.encoder_count)
        self.target = [-49, -40]

        #initialize control
        self.timer = Timer(timer_num)
        self.timer.init(period=100, mode=self.timer.PERIODIC, callback=self.control) # control timer
        
    def control(self, timer):
        if self.encoder_count>= self.target[0] and self.encoder_count <= self.target[1]:
            self.stay()
        elif self.encoder_count < self.target[0]:
            self.bend()
        elif self.encoder_count > self.target[1]:
            self.strech()
            
    def configure_pwm_pins(self, v_pin, g_pin):
        v_ret = PWM(v_pin, freq=100000, duty=0) #100000
        g_ret = PWM(g_pin, freq=100000, duty=0)
        return v_ret, g_ret
    
    
    def stay(self):
        if self.state == 0:
            return
        self.state = 0
        self.v_pin.duty(0)
        self.g_pin.duty(0)

    def bend(self, init=False):
        
        if init:
            self.v_pin.duty(950)
            self.g_pin.duty(0)
            return
        #print(self.encoder_count)
        self.state = 1
        ret = int(600 + 10*(self.target[0]-self.encoder_count))
        ret = min(max(700, ret), 800)
        self.v_pin.duty(850)
        self.g_pin.duty(0)
    
    def strech(self,init=False):
        #print(self.encoder_count)
        self.state = -1
        ret = int(600 - 5*(self.target[0]-self.encoder_count))
        ret = min(max(700, ret), 800)
        self.v_pin.duty(0)
        self.g_pin.duty(500)
    
    def counter(self, pin):
        if self.state == 0:
            return
        if self.encoder_0.value() != self.encoder_1.value():
            self.encoder_count += 1
            self.direction = 1
        else:
            self.encoder_count -= 1
            self.direction = -1
            
    def generate_target(self, tar):
        self.connected = True
        self.target = [(tar/2-45)-1, (tar/2-45)+1]
        print('target: ', self.target)
      
    def deinit_every_pwm(self):
        self.timer.deinit()
        self.v_pin.deinit()
        self.g_pin.deinit()
    
    
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



# configure
# index_finger = finger(5,18,19,1)
# middle_finger = finger(25,26,34,2)
# finger4 = finger(16,17,21,3)
finger5 = finger(15,32,27,33,4)

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
            try:
                message = sc.recv(64) #IMPORTANT: Receive message of byte size=?
                #TODO: Need some decoding and encoding
                
                message = decode_message(message)
                print(message)
                #actuation
#                 index_finger.generate_target(message[2])
#                 middle_finger.generate_target(message[3])
#                 finger4.generate_target(message[4])
                finger5.generate_target(message[5])
                print(finger5.encoder_count)
            except:
#                 index_finger.deinit_every_pwm()
#                 middle_finger.deinit_every_pwm()
#                 finger4.deinit_every_pwm()
                finger5.deinit_every_pwm()
                sc.close()
                s.close()

