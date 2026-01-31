from time import sleep
from  RPLCD.i2c import CharLCD
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
        # Remember that your sentences can only be 16 characters long!
        print("Writing to display")
        lcd_display_string("Fuckibazz Ratul!", 1)  # Write line of text to first line of display
        lcd_display_string("Campus a ashben?", 2)  # Write line of text to second line of display
        sleep(2)                                           # Give time for the message to be read
        display.clear()                                # Clear the display of any data
        lcd_display_string("meow!", 1)   # Refresh the first line of display with a different message
        lcd_display_string("sync hoyna aaa!", 2)   # Refresh the first line of display with a different message
        sleep(2)                                           # Give time for the message to be read
        display.clear()                                # Clear the display of any data
        sleep(2)                                           # Give time for the message to be read
except KeyboardInterrupt:
    # If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
    print("Cleaning up!")
    display.clear()