# Victoria Scavetta
# 05/04/2022
# Advanced Robotics
#
# This program allows a robot to follow a light and translate morse code. The robot's speed is controlled by two PID controllers.
# The robot is only capable of translating one word from morse code.

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

# Initialize hub, left color sensor, right color sensor, left motor, right motor, and the timer
hub = PrimeHub()
left_sensor = ColorSensor('A')
right_sensor = ColorSensor('F')
left_motor = Motor('C')
right_motor = Motor('D')
timer = Timer()

# Morse code translation for each letter in the English alphabet
# A dot is represented by a 1
# A dash is represented by a 3
# 0 represents the intra-character space
morse_code = {
    "A": [1, 0, 3],
    "B": [3, 0, 1, 0, 1, 0, 1],
    "C": [3, 0, 1, 0, 3, 0, 1],
    "D": [3, 0, 1, 0, 1],
    "E": [1],
    "F": [1, 0, 1, 0, 3, 0, 1],
    "G": [3, 0, 3, 0, 1],
    "H": [1, 0, 1, 0, 1, 0, 1],
    "I": [1, 0, 1],
    "J": [1, 0, 3, 0, 3, 0, 3],
    "K": [3, 0, 1, 0, 3],
    "L": [1, 0, 3, 0, 1, 0, 1],
    "M": [3, 0, 3],
    "N": [3, 0, 1],
    "O": [3, 0, 3, 0, 3],
    "P": [1, 0, 3, 0, 3, 0, 1],
    "Q": [3, 0, 3, 0, 1, 0, 3],
    "R": [1, 0, 3, 0, 1],
    "S": [1, 0, 1, 0, 1],
    "T": [3],
    "U": [1, 0, 1, 0, 3],
    "V": [1, 0, 1, 0, 1, 0, 3],
    "W": [1, 0, 3, 0, 3],
    "X": [3, 0, 1, 0, 1, 0, 3],
    "Y": [3, 0, 1, 0, 3, 0, 3],
    "Z": [3, 0, 3, 0, 1, 0, 1]
}


def calibrate():
    """
    Calibrate the robot by first shining a flashlight directly at the robot's left sensor and pressing the right button after the robot beeps.
    When it beeps again, turn the flashlight off and press the right button again.
    """
    hub.speaker.beep()  # Ready to take reading

    # Take reading of max light when right button is pressed
    hub.right_button.wait_until_pressed()
    max_light = left_sensor.get_ambient_light()

    wait_for_seconds(1)
    hub.speaker.beep()  # Ready to take second reading

    # Take reading of min light when right button is pressed
    hub.right_button.wait_until_pressed()
    min_light = left_sensor.get_ambient_light()

    return (max_light, min_light)


def follow_light(duration=20, max_light=90):
    """
    The robot will follow a light source for duration seconds. If the light turns to the left, the robot will turn to the left. 
    If the light turns to the right, the robot will turn to the right. If either sensor detects an ambient light value >= max_light, 
    the robot will stop. The robot will start moving again if the light starts moving away from the robot.
    """
    # PID variables
    left_error_sum = 0
    right_error_sum = 0
    left_prev_error = 0
    right_prev_error = 0
    gain_p = 0.3
    gain_i = 0.0001
    gain_d = 0.4
    
    timer.reset()   # Make sure the timer is set to 0

    # Follow a light source for duration seconds
    while timer.now() <= duration:
        left_light = left_sensor.get_ambient_light()    # Amount of ambient light on the left side of the front of the robot
        right_light = right_sensor.get_ambient_light()  # Amount of ambient light on the right side of the front of the robot

        left_error = max_light - left_light     # How far off the left sensor reading is from the threshold
        right_error = max_light - right_light   # How far off the right sensor reading is from the threshold

        # If one of the sensors has an error of 0, the robot is close enough to the light to stop moving
        if left_error == 0 or right_error == 0:
            left_error = 0
            right_error = 0
            
            # Reset the integral terms to 0 if the error is 0 to make sure the integral term doesn't cause the robot not to stop
            left_error_sum = 0
            right_error_sum = 0
        else:
            left_error_sum = left_error_sum + left_error        # The sum of all of the errors from the left sensor readings
            right_error_sum = right_error_sum + right_error     # The sum of all of the errors from the right sensor readings

        left_error_diff = left_error - left_prev_error      # The differential term of the left sensor readings
        right_error_diff = right_error - right_prev_error   # The differential term of the right sensor readings

        # If one of the differential terms is 0, set the other to 0 so that both wheels will stop moving
        if left_error_diff == 0 or right_error_diff == 0:
            left_error_diff = 0
            right_error_diff = 0

        left_prev_error = left_error    # Set the previous left sensor error to be the current one
        right_prev_error = right_error  # Set the previous right sensor error to be the current one

        # The speed of the left wheel is controlled by the left sensor error, error sum, and error differential
        # The speed of the right wheel is controlled by the right sensor error, error sum, and error differential
        left_power = int(gain_p * left_error + gain_i * left_error_sum + gain_d * left_error_diff)
        right_power = int(gain_p * right_error + gain_i * right_error_sum + gain_d * right_error_diff)
            
        left_motor.start(speed=-left_power)
        right_motor.start(speed=right_power)

    left_motor.stop()
    right_motor.stop()


def read_morse_with_pauses(num_readings, min_light, max_light):
    """
    Read in ambient light values from the left sensor num_readings times. Before each reading, the robot will beep once. When the robot beeps,
    press the right button to take a reading. After num_readings readings have been taken, figure out how long the light was on/off and store
    the lengths in a list. For every reading the light is off, there will be a 0 in the list. If the light is on for three readings in a row, 
    then off for three, then on for another, the list will look like [3, 0, 0, 0, 1].
    """
    readings = []   # A list of readings the left sensor will take
    morse_lengths = []  # A list of how long the light was on in a row to help distinguish between dots and dashes
    counter = 0     # Counter to help keep track of how many readings it has taken so far

    # Take num_readings readings
    while counter < num_readings:
        wait_for_seconds(0.2)
        
        # Let the user know the robot is ready for a reading
        # User will press right button when they are ready to take a reading
        hub.speaker.beep()
        hub.right_button.wait_until_pressed()

        # Take a reading from the left sensor and add the reading to the list of readings
        reading = left_sensor.get_ambient_light()
        readings.append(reading)

        counter += 1    # Increment the counter by 1 to keep track of how many readings it has taken so far

    morse_length = 0    # How many readings in a row the light was detected to be on

    # For each reading, find out if the light was on. If it was, increment morse_length. If it wasn't, add the current value of morse_length
    # to the morse_lengths list. If morse_length is > 0 and the light is off, also append a 0 to morse_lengths and reset morse_length to 0.
    for reading in readings:
        # If the reading value is closer to max_light than min_light, the light can be considered to be on
        if abs(reading - max_light) < abs(reading - min_light):
            morse_length += 1
        else:
            morse_lengths.append(morse_length)
            if morse_length > 0:
                morse_lengths.append(0)
                morse_length = 0

    # Make sure all readings were accounted for by checking if morse_length is still > 0 after the loop finishes. If it is, add this value to the list.
    if morse_length > 0:
        morse_lengths.append(morse_length)

    morse_lengths = trim_morse_list(morse_lengths)  # Get rid of any trailing 0's
    
    return morse_lengths


def translate_morse_code(morse_lengths):
    """
    Given a list of morse code lengths (ex. [3, 0, 0, 0, 1]), separate each letter. Spaces between letters are three units. Three 0's in a row in the list
    represents a space between letters. One 0 represents a space between dots/dashes of the same letter. Using the example [3, 0, 0, 0, 1], a 2D list
    [[3], [1]] will be formed. This 2D list will be translated into text using the get_word_string function.
    """
    letters = []    # List where morse code of each letter will be stored, each in their own list

    break_time = 0  # Length of the space between dots and dashes
    curr_letter = []    # List to store the current letter's morse code
    last_index = len(morse_lengths) - 1     # Last index of the morse_lengths list

    # Separate letters in the morse code lengths into their own lists and put each letter's list into the letters list
    for i in range(len(morse_lengths)):
        curr_length = morse_lengths[i]
        # If i is the last index of the list, just set next_length to 0 to not get an out of bounds error
        if i < last_index:
            next_length = morse_lengths[i+1]
        else:
            next_length = 0

        # If the current length is not 0, but the next length is, append both lengths to the current letter's list and reset break time to 0
        # Otherwise, if the current length is 0, increment the break time
        if curr_length != 0 and next_length == 0:
            curr_letter.append(curr_length)
            curr_letter.append(next_length)
            break_time = 0
        elif curr_length == 0:
            break_time += 1
        
        # If the break time is 3 or i is the last index, we have reached the end of a letter
        if break_time == 3 or i == last_index:
            # If the current letter list is not empty, append the current list to the letters list and reset the current list
            if curr_letter:
                curr_letter.pop()   # Remove last 0 from list as it is not necessary for translation
                letters.append(curr_letter)
                curr_letter = []

    return get_word_string(letters)


def get_word_string(letters):
    """
    Turns a morse code word into text given a list containing lists of morse code for each letter in the word. If the list is [[1, 0, 3], [1, 0, 1]] 
    the translated word will be "AI".
    """
    word = ""

    # Match the morse code list for each letter to the dictionary of translations to find the letter it corresponds to and add it to the word string
    for i in range(len(letters)):
        for key, value in morse_code.items():
            if letters[i] == value:
                word += key
    return word


def trim_morse_list(morse_list):
    """
    Gets rid of trailing 0's in the morse code list
    """
    found = False
    i = len(morse_list) - 1     # The first index that will be checked will be the last index in the list

    # While a value > 0 has not been found and the list is not empty, check if the last value in the list is a 0.
    # If it is a 0, remove it from the list
    while not found and morse_list:
        if morse_list[i] > 0:
            found = True
        else:
            morse_list.pop()
            i -= 1

    return morse_list

        
def demo():
    """
    First, calibrate the robot. After calibration, the robot will beep 3 times indicating that it is ready to start following a light source.
    Press the right button to have the robot follow the light source for 20 seconds. After 20 seconds, the robot will beep 3 times again. Press 
    the right button to start sending the robot a word in morse code that takes 50 readings. If the robot is able to translate at least one letter 
    from the word, it will display a happy face for 5 seconds, repeat the morse code message in beeps, and display the translation on the light 
    matrix. If it cannot translate the word at all, it will display a sad face.
    """
    wait_for_seconds(1)
    left_sensor.get_ambient_light()
    right_sensor.get_ambient_light()
    max_light, min_light = calibrate()
    wait_for_seconds(1)

    hub.speaker.beep()
    hub.speaker.beep()
    hub.speaker.beep()
    hub.right_button.wait_until_pressed()
    follow_light(max_light=max_light)

    hub.speaker.beep()
    hub.speaker.beep()
    hub.speaker.beep()
    hub.right_button.wait_until_pressed()
    morse = read_morse_with_pauses(50, min_light, max_light)
    message = translate_morse_code(morse)
    print(message)

    if len(message) > 0:
        hub.light_matrix.show_image('HAPPY')
        wait_for_seconds(5)

        for length in morse:
            if length > 0:
                hub.speaker.beep(seconds=0.25*length)
            else:
                wait_for_seconds(0.25)

        hub.light_matrix.write(str(message))
    else:
        hub.light_matrix.show_image('SAD')




demo()
