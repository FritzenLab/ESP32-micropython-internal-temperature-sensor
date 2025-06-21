# reference 1: https://domoticx.net/docs/ntc-sensor-rpi-pico/
# reference 2: https://bhave.sh/micropython-measure-temperature/
# reference 3: https://randomnerdtutorials.com/esp32-esp8266-analog-readings-micropython/

from machine import Pin, Timer, ADC
import time
import esp32
import math

led = Pin(8, Pin.OUT)
tempsensor = ADC(4)
ledDelay= time.ticks_ms()
tempDelay= time.ticks_ms()
adcntc = ADC(Pin(3))
adcntc.atten(ADC.ATTN_11DB) # 0 - 3.3V input range

BETA = 4000           # Beta parameter (adjust to your thermistor)
T0 = 298.15           # Reference temperature (Kelvin) = 25°C (273.15 + 25)
R0 = 10000            # Thermistor resistance at 25°C (Ohms)
R_FIXED = 10000       # Fixed series resistor (Ohms)
VREF = 3.3            # ADC reference voltage

def read_temperature():
    adc_val = adcntc.read_u16()
    voltage = adc_val * VREF / 65535
    #print("V ntc= ", voltage) #analog voltage on pin A3 (ntc reading)
    
    # Calculate thermistor resistance using voltage divider formula
    r_ntc = R_FIXED * (VREF / voltage - 1)

    # Apply Beta equation to calculate temperature (Kelvin)
    temperature_kelvin = 1 / (1/T0 + (1/BETA) * math.log(r_ntc / R0))

    # Convert to Celsius
    temperature_celsius = temperature_kelvin - 273.15
    return temperature_celsius

while True:
    if time.ticks_ms() - tempDelay > 1000:
        tempDelay= time.ticks_ms()
        temp = esp32.mcu_temperature()
        print("CPU Temperature: {:.0f}°C".format(temp))
        tempntc = read_temperature()
        print("NTC Temperature: {:.2f}°C".format(tempntc))
        print("__________________________")
                
    if time.ticks_ms() - ledDelay > 200: # blinks an LED (onboard ESP32-C3 Super mini, pin 8)
        ledDelay= time.ticks_ms()
        led.value(not led.value())