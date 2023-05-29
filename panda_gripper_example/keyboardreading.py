from pynput import keyboard
import redis

r = redis.Redis()

def check_key_pressed(key):
    if key == 'a': # move left
        r.set("MOVE_LEFT", "1")
        print("\nThe 'a' key has been pressed!")
    elif key == 'd': # move right
        r.set("MOVE_RIGHT", "1")
        print("\nThe 'd' key has been pressed!")
    elif key == 's':
        r.set("NEXT_STATE", "1")
        print("\nThe 's' key has been pressed!")
    elif key == 't':
        r.set("START_TRACKING", "1")
        print("\nThe 't' key has been pressed!")    
    else:
        print("\nInput not accepted.")

def on_press(key):
    try:
        check_key_pressed(key.char)
    except AttributeError:
        print('\nSpecial key {} pressed.'.format(key))

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

# Create a listener for keyboard events
listener = keyboard.Listener(
    on_press = on_press,
    on_release = on_release)
listener.start() # start listening

# Keep the program running
while listener.running:
    pass