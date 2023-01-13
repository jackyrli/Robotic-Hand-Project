from machine import Pin, ADC
from time import sleep


blue = ADC(Pin(33)) #insead ofA0/26
green = ADC(Pin(32)) #instead of A1/25
brown = ADC(Pin(34)) #A2
maroon = ADC(Pin(39)) #A3
grey = ADC(Pin(36)) #A4

blue.atten(ADC.ATTN_11DB)       #Full range: 3.3v
blue.width(ADC.WIDTH_12BIT)
green.atten(ADC.ATTN_11DB)       #Full range: 3.3v
green.width(ADC.WIDTH_12BIT)
brown.atten(ADC.ATTN_11DB)       #Full range: 3.3v
brown.width(ADC.WIDTH_12BIT)
maroon.atten(ADC.ATTN_11DB)       #Full range: 3.3v
maroon.width(ADC.WIDTH_12BIT)
grey.atten(ADC.ATTN_11DB)       #Full range: 3.3v
grey.width(ADC.WIDTH_12BIT)

while True:
    blue_val = blue.read()
    green_val = green.read()
    brown_val = brown.read()
    maroon_val = maroon.read()
    grey_val = grey.read()
    print("blue", blue_val)
    print("green",green_val)
    print("brown",brown_val)
    print("maroon",maroon_val)
    print("grey",grey_val)
    sleep(1)
