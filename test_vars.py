import time

relayState = False

def toggle_relay():
    if relayState == False:
        print("relay ON")
        time.sleep(0.8)
    print("relay OFF")
    time.sleep(0.2)
    toggle_relay()  # Recursively call the function

# Call the function to start the toggling
toggle_relay()
