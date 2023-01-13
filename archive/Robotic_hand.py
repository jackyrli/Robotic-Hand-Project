from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule
from machine import Pin, ADC
from utime import sleep_ms
from network import WLAN,STA_IF
import socket
import esp32
import machine
import time

class finger: #finger num is 2 to 5
    def __init__(self, v_pin_num, g_pin_num, enc_pin_num1, enc_pin_num2, timer_num, finger_num):
        control_list = [self.control2, self.control3, self.control4, self.control5]
        target_list = [self.generate_target2, self.generate_target3, self.generate_target4, self.generate_target5]
        self.control = control_list[finger_num-2]
        self.generate_target = target_list[finger_num-2]
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
            
    def configure_pwm_pins(self, v_pin, g_pin):
        v_ret = PWM(v_pin, freq=100000, duty=0) #100000
        g_ret = PWM(g_pin, freq=100000, duty=0)
        return v_ret, g_ret
    
    def stay(self):
        self.v_pin.duty(0)
        self.g_pin.duty(0)
        self.bend_pwm = 0
        self.stretch_pwm = 0
        self.direction = 0

    def bend(self, pwm=1000):
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
      
    def deinit_every_pwm(self):
        self.timer.deinit()
        self.v_pin.deinit()
        self.g_pin.deinit()
    
    def control2(self, timer):
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
    
    def generate_target2(self, tar):
        self.connected = True
        self.target = [(tar-90)/3.9-1, (tar-90)/3.9+1]
        
    def control3(self, timer):
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

    def generate_target3(self, tar):
        self.connected = True
        self.target = [(tar-90)/2.9-1, (tar-90)/2+1]
    
    def control4(self, timer):
        self.error += abs(self.target[0] + (self.target[1] - self.target[0])/2 - self.encoder_count)
        if self.encoder_count>= self.target[0] and self.encoder_count <= self.target[1]:
            self.stay()
            self.error = 0
        elif self.encoder_count < self.target[0]:
            ret = 550 + 3*(self.target[0]-self.encoder_count)
            ret = min(max(550, int(ret + 0.9 * self.error)), 800)
            #print('Bend PWM', ret)
            self.direction = 1
            self.bend(ret)
        elif self.encoder_count > self.target[1]:
            if self.encoder_count < -25 and self.encoder_count > -55:
                max_stretch_pwm = 650
            else:
                max_stretch_pwm = 600
            ret = int(500 - 0.05*(self.target[0]-self.encoder_count))
            ret = min(max(300, int(ret + 0.2 * self.error)), max_stretch_pwm)
            #print('Stretch PWM', ret)
            self.direction = -1
            self.stretch(ret)
    
    def generate_target4(self, tar):
        self.connected = True
        self.target = [tar-1, tar+1]

    def control5(self, timer):
        self.error += abs(self.target[0] + (self.target[1] - self.target[0])/2 - self.encoder_count)
        if self.encoder_count>= self.target[0] and self.encoder_count <= self.target[1]:
            self.stay()
            self.error = 0
        elif self.encoder_count < self.target[0]:
            ret = 540 + 3*(self.target[0]-self.encoder_count)
            ret = min(max(550, int(ret + 0.9 * self.error)), 1000)
            #print('Bend PWM', ret)
            self.direction = 1
            self.bend(ret)
        elif self.encoder_count > self.target[1]:
            if self.encoder_count < -25 and self.encoder_count > -55:
                max_stretch_pwm = 650
            else:
                max_stretch_pwm = 600
            ret = int(500 - 0.07*(self.target[0]-self.encoder_count))
            ret = min(max(300, int(ret + 0.3 * self.error)), max_stretch_pwm)
            #print('Stretch PWM', ret)
            self.direction = -1
            self.stretch(ret)
            
    def generate_target5(self, tar):
        self.connected = True
        self.target = [(tar-90)/2-1, (tar-90)/2+1]


class thumb:
    def __init__(self, v_pin_num, g_pin_num, timer_num):
        self.connected = False
        self.state = 1 # 1:bend; 0: stay, -1: strech
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
#index_finger = finger(5,18,19,1)
# middle_finger = finger(25,26,34,2)
# finger4 = finger(16,17,21,3)
#Enc_pin_num1 - Yellow Wire
#Enc_pin_num2 - Green Wire
finger1= thumb(v_pin_num=25, g_pin_num=26, timer_num=0)
finger2= finger(v_pin_num=21, g_pin_num=14, enc_pin_num1=34, enc_pin_num2=39, timer_num=1, finger_num=2)
finger3= finger(v_pin_num=15, g_pin_num=32, enc_pin_num1=23, enc_pin_num2= 22, timer_num=2, finger_num=3)
finger4= finger(v_pin_num=27, g_pin_num=33, enc_pin_num1=36, enc_pin_num2=5, timer_num=3, finger_num=4)
finger5= finger(v_pin_num=13, g_pin_num=12, enc_pin_num1=18, enc_pin_num2=19, timer_num=4, finger_num=5)
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
                finger1.generate_target(message[1])
                finger2.generate_target(message[2])
                finger3.generate_target(message[3])
                finger4.generate_target(message[4])
                finger5.generate_target(message[5])
                sleep(0.7)
                print('Finger 1 State: ', finger1.state)
                print('Finger 1 Target: ', finger1.target)
                #if finger3.direction==1:
                #    print('Bend PWM: ',finger3.bend_pwm)
                #else:
                #    print('Stretch PWM: ', finger3.stretch_pwm)
                #print('Finger 3 Target', finger3.target[0], finger3.target[1])
                
            except:
                pass
#                 index_finger.deinit_every_pwm()
#                 middle_finger.deinit_every_pwm()
#                 finger4.deinit_every_pwm()

                #finger5.deinit_every_pwm()
                #sc.close()
                #s.close()






