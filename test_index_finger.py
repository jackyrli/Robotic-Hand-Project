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
        self.error = 0
        self.encoder_0.irq(handler=self.counter, trigger=Pin.IRQ_RISING)
        
        self.v_pin, self.g_pin = self.configure_pwm_pins(motor_vpin, motor_gnd)
        self.encoder_count = 0
        #Initialize position
        self.initialize_position()
        
        #initialize control
        self.target = [-2, 0]
        self.timer = Timer(timer_num)
        self.timer.init(period=70, mode=self.timer.PERIODIC, callback=self.control) # control timer
    
        self.bend_pwm = 0
        self.stretch_pwm = 0
    def initialize_position(self):
        #initialize initial position
        print('Initializing Position')
        self.stretch()
        sleep(2)
        last_count = 100000000
        while(True):
            current_count = self.encoder_count
            if (current_count - last_count) == 0:
                break
            self.bend()
            sleep(0.1)
            last_count = current_count
        self.stay()
        self.encoder_count = 0
        print('Done Initializing')
        sleep(1)
        self.encoder_count = 0 
        
    def control(self, timer):
        self.error += abs(self.target[0] + (self.target[1] - self.target[0])/2 - self.encoder_count)
        if self.encoder_count>= self.target[0] and self.encoder_count <= self.target[1]:
            self.stay()
            self.error = 0
        elif self.encoder_count < self.target[0]:
            ret = 550 + 3.5*(self.target[0]-self.encoder_count)
            ret = min(max(550, int(ret + 1.1 * self.error)), 900)
            #print('Bend PWM', ret)
            self.direction = 1
            self.bend(ret)
        elif self.encoder_count > self.target[1]:
            if self.encoder_count < -20 and self.encoder_count > -40:
                max_stretch_pwm = 600
            else:
                max_stretch_pwm = 600
            ret = int(520 - 0.04*(self.target[0]-self.encoder_count))
            ret = min(max(300, int(ret + 0.6 * self.error)), max_stretch_pwm)
            #print('Stretch PWM', ret)
            self.direction = -1
            self.stretch(ret)
            
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
    
    def stretch(self,pwm=500):
        self.v_pin.duty(0)
        self.g_pin.duty(pwm)
        self.stretch_pwm = pwm
    
    def counter(self, pin):
        if self.encoder_0.value() != self.encoder_1.value():
            self.encoder_count -= 1
        else:
            self.encoder_count += 1
            
    def generate_target(self, tar):
        self.connected = True
        self.target = [(tar-90)/3.6-1, (tar-90)/3.6+1]
        print('target: ', self.target)
      
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
#Enc_pin_num1 - Yellow Wire
#Enc_pin_num2 - Green Wire
finger2= finger(v_pin_num=21, g_pin_num=14, enc_pin_num1=34, enc_pin_num2=39, timer_num=4)
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
                finger2.generate_target(message[2])
                if finger2.direction == 1:
                    print('Encoder Count:' ,finger2.encoder_count, 'Bend PWM', finger2.bend_pwm)
                elif finger2.direction == -1:
                    print('Encoder Count:' ,finger2.encoder_count, 'Stretch', finger2.stretch_pwm)
                else:
                    print('Encoder Count:' , finger2.encoder_count)
                sleep(0.7)
            except:
#                 index_finger.deinit_every_pwm()
#                 middle_finger.deinit_every_pwm()
#                 finger4.deinit_every_pwm()

                #finger5.deinit_every_pwm()
                #sc.close()
                #s.close()
                pass


