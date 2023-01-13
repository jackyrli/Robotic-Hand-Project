import time
import machine
from machine import Pin,Timer

LED_BLINK_TIMER_CLOCK_ms = 1000
timer_period = 1000
led_blink_active_flag = 1
ledstate = 1
timer_interrupt_flag = 1

def toggle_led(timer):
    global ledstate
    ledstate = ledstate^1
    if led_blink_active_flag == 1:
        led(ledstate)

led = Pin(21, mode=Pin.OUT)
led(ledstate)
t1 = Timer(1)
t1.init(period=LED_BLINK_TIMER_CLOCK_ms, mode=t1.PERIODIC, callback=toggle_led)

print('\r\nESP32 Ready to accept Commands\r\n')

try:
    while(1):
        Command=input('')
        if Command == 'T':
            if led_blink_active_flag == 1:
                print("TLED blinking paused \r\n")
            else:
                print("TLED blinking resumed \r\n")
            led_blink_active_flag ^= 1
        elif Command == 'A':
            print('AReceived the A command\r\n')
            t1.period(LED_BLINK_TIMER_CLOCK_ms)
            timer_period=LED_BLINK_TIMER_CLOCK_ms
        elif Command == 'B':
            print('BReceived the B command\r\n')
            timer_period=timer_period-100
            t1.period(timer_period)
            if timer_period <= 200:
                timer_period = 200
        elif Command == 'I':
            print('IESP32\r\n')
except:
    t1.deinit()
    pass


