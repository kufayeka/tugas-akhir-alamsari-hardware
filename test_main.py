import multiprocessing
import test_vars as gv
import time

def process1(kp):
    # Modify process1 logic using the updated gv variables
    while True:
        print("PID_kp:", kp)
        time.sleep(1)

def process2(ki):
    # Modify process2 logic using the updated gv variables
    while True:
        print("PID_ki:", ki)
        time.sleep(1)

def process3():
    # Modify process3 logic using the updated gv variables
    while True:
        time.sleep(1)

if __name__ == '__main__':
    kp = gv.kp
    ki = gv.ki

    p1 = multiprocessing.Process(target=process1, args=(kp,))
    p2 = multiprocessing.Process(target=process2, args=(ki,))
    p3 = multiprocessing.Process(target=process3, args=())

    try:
        print("STARTING...")
        p1.start()
        p2.start()
        p3.start()

        while True:
            # Update the shared values with the new values from gv
            kp = gv.kp
            ki = gv.ki
            time.sleep(1)
            
    except KeyboardInterrupt:
        p1.terminate()
        p2.terminate()
        p3.terminate()

        # Save the Excel workbook & Close the workbook when done
        # wb.save(excel_name)

        print("TERMINATED...")
        time.sleep(0.1)
    
    except ValueError:  
        print("PROGRAM ERROR")
        time.sleep(0.5)

    finally:  
        time.sleep(0.5)