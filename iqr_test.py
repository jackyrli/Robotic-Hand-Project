from machine import Pin
from time import sleep
i=0
def isr(pin):
    global i
    i+=1
    
pin_to_test = Pin(23, mode = Pin.IN)
pin_to_test.irq(handler=isr, trigger = Pin.IRQ_RISING)

while(True):
    print(i)
    sleep(0.1)
    
    