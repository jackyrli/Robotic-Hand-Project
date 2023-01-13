from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule
from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
import esp32
import time

class thumb:
    def __init__(self, v_pin_num, g_pin_num, timer_num):
        self.connected = False
        self.state = 0 # 1:bend; 0: stay, -1: strech
        self.target = [-10,10]
        motor_vpin = Pin(v_pin_num, mode=Pin.OUT) #bend
        motor_gnd = Pin(g_pin_num, mode=Pin.OUT)  #strech
        
        self.v_pin, self.g_pin = self.configure_pwm_pins(motor_vpin, motor_gnd)
        #Initialize position
        self.initialize_position()
        
        #initialize control
        self.timer = Timer(timer_num)
        self.timer.init(period=70, mode=self.timer.PERIODIC, callback=self.control) # control timer
        self.bend_pwm = 0
        self.stretch_pwm = 0
    
    def initialize_position(self):
        #initialize initial position
        print('Initializing Position')
        self.stretch()
        sleep(2)
        self.stay()
        sleep(1)
        self.bend()
        print('Done Initializing')
        sleep(1)
        
    def control(self, timer):
        if self.target == self.state:
            self.stay()
        elif self.target == 1:
            self.bend()
            self.state = 1
        else:
            self.stretch()
            self.state = -1
            
    def configure_pwm_pins(self, v_pin, g_pin):
        v_ret = PWM(v_pin, freq=100000, duty=0) #100000
        g_ret = PWM(g_pin, freq=100000, duty=0)
        return v_ret, g_ret
    
    def stay(self):
        self.v_pin.duty(0)
        self.g_pin.duty(0)
        self.bend_pwm = 0
        self.stretch_pwm = 0

    def bend(self, pwm=900):
        self.v_pin.duty(pwm)
        self.g_pin.duty(0)
        self.bend_pwm = pwm
    
    def stretch(self,pwm=750):
        self.v_pin.duty(0)
        self.g_pin.duty(pwm)
        self.stretch_pwm = pwm
    
    def counter(self, pin):
        if self.encoder_0.value() != self.encoder_1.value():
            self.encoder_count -= 1
        else:
            self.encoder_count += 1
            
    def generate_target(self, tar):
        if tar > 50:
            self.target = 1
        else:
            self.target = -1
      
    def deinit_every_pwm(self):
        self.timer.deinit()
        self.v_pin.deinit()
        self.g_pin.deinit()
    
    
def connect_to_internet(): #connect to wifi
    wlan = WLAN(STA_IF)
    wlan.active(True)
    wlan.connect('Sarooshki','12345678') #这里放wlan.connect('ME100-2.4G', '122Hesse', 5000)
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
#index_finger = finger(5,18,19,1)
# middle_finger = finger(25,26,34,2)
# finger4 = finger(16,17,21,3)
finger5= thumb(v_pin_num=25, g_pin_num=26, timer_num=4)

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
                #actuation
#                 index_finger.generate_target(message[2])
#                 middle_finger.generate_target(message[3])
#                 finger4.generate_target(message[4])
                finger5.generate_target(message[1])
                if finger5.direction == 1:
                    print('Encoder Count:' ,finger5.encoder_count, 'Bend PWM', finger5.bend_pwm)
                elif finger5.direction == -1:
                    print('Encoder Count:' ,finger5.encoder_count, 'Stretch', finger5.stretch_pwm)
                else:
                    print('Encoder Count:' , finger5.encoder_count)
                sleep(0.7)
            except:
#                 index_finger.deinit_every_pwm()
#                 middle_finger.deinit_every_pwm()
#                 finger4.deinit_every_pwm()

                #finger5.deinit_every_pwm()
                #sc.close()
                #s.close()
                pass


