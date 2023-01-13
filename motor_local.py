from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule
state = 0 # 1:bend; 0: stay, -1: strech
"""Connecting GPIO pins A0 and A1 to the signal-in of the H-Bridge"""
motor_vpin = Pin(25, mode=Pin.OUT) #bend
motor_gnd = Pin(26, mode=Pin.OUT)  #strech

#useful functions
def configure_pwm_pins(v_pin, g_pin):
    v_ret = PWM(v_pin, freq=100000, duty=0)
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
    if state == 1:
        return
    state = 1
    v_pin.duty(900)
    g_pin.duty(0)
    print('changed')
    
def strech(v_pin, g_pin):
    global state
    if state == -1:
        return
    state = -1
    v_pin.duty(0)
    g_pin.duty(800)

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
t1.init(period=300, mode=t1.PERIODIC, callback=report)

# configure
#configure_pwm_pins(motor_vpin, motor_gnd)
L1, L2 = configure_pwm_pins(motor_vpin, motor_gnd)

while True:
    sleep(0.01)
    target=[200,230] #40 240
    if encoder_count <= target[0]:
        bend(L1, L2)
        
    elif encoder_count >= target[1]:
        strech(L1,L2)
        
#     elif encoder_count > target:
#         strech(L1, L2)


# motor_vpin.value(0)
# motor_gnd.value(0)
