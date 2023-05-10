import time
import test_globals
import threading

def increment_A():
    while True:
        test_globals.A += 1
        time.sleep(1)

t = threading.Thread(target=increment_A)
t.daemon = True
t.start()
