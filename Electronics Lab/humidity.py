import time
import adafruit_dht
import board
from time import sleep
from  RPLCD.i2c import CharLCD
from digitalio import DigitalInOut, Direction

dht_device = adafruit_dht.DHT22(board.D4)
mosfet_pin = DigitalInOut(board.D11)  
mosfet_pin.direction = Direction.OUTPUT
display = CharLCD('PCF8574', 0x27)

def lcd_display_string(text, line):
    """
    Write a string to a specific line of the LCD.
    
    :param text: The text to display
    :param line: Line number (1-indexed)
    """
    if line not in [1, 2]:  # adjust if you have 4-line LCD
        raise ValueError("Line number must be 1 or 2")
    
    display.cursor_pos = (line - 1, 0)  # set cursor to start of the line
    display.write_string(text.ljust(16))  # pad/truncate to 16 characters



try:
    while True:
        # Your DHT22 + LCD logic here
        temperature_c = dht_device.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = dht_device.humidity
        str1 = "Temp:{:.1f} C / {:.1f} F Humidity: {}%".format(temperature_c, temperature_f, humidity)
        print(str1)

        if temperature_c is not None and temperature_c > 20:
            mosfet_pin.value = True # ON 
            print("MOSFET ON (Load Active)") 
        else: 
            mosfet_pin.value = False # OFF 
            print("MOSFET OFF (Load Inactive)")
        
        print("Writing to display")

        lcd_display_string(f"Temp: {temperature_c:.1f}C", 1)
        lcd_display_string(f"Hum : {humidity:.1f}%", 2)

        time.sleep(2)

except RuntimeError as err:
    # DHT22 often throws random RuntimeErrors, just retry
    print(f"DHT22 Error: {err.args[0]}")

except KeyboardInterrupt:
    # Handle Ctrl+C cleanly
    print("Cleaning up...")
    display.clear()
    lcd_display_string("Ara Ara Sayonara...", 1)

    for i in range(10, -1, -1):  # start at 10, stop at 0
        minutes = 0
        seconds = i
        timer_str = f"{minutes} {minutes} : {seconds//10} {seconds%10}"
        lcd_display_string(timer_str, 2) 
        time.sleep(1)

    display.clear()
    lcd_display_string("ALLAHU AKBAAR", 1)
    lcd_display_string("!!!!!!!!!!!!!!!!", 2)
    time.sleep(2)
    display.backlight_enabled = False
