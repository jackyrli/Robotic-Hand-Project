from machine import Pin, PWM, Timer
from time import sleep
from micropython import schedule

encoder_0 = Pin(22, mode=Pin.IN)
encoder_1 = Pin(17, mode=Pin.IN)

counter = 0
def encoder0_isr(pin):
    global counter
    if encoder_0.value() != encoder_1.value():
        counter -= 1
    else:
        counter += 1
    
encoder_0.irq(handler=encoder0_isr, trigger=Pin.IRQ_RISING)

while True:
    sleep(0.1)
    print(counter)