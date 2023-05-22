import schedule
import time
import multiprocessing

def process1():
    # Define the task you want to schedule
    def scheduled_task():
        print("btch")

    # Schedule the task to run every 1 second
    schedule.every(1).second.do(scheduled_task)

    # Run the schedule loop
    while True:
        schedule.run_pending()
        time.sleep(1)

if __name__ == '__main__':
    # Create a multiprocessing process for process1
    p1 = multiprocessing.Process(target=process1, args=())

    try:
        print("STARTING...")
        p1.start()
        # Add the rest of your multiprocessing code here

        p1.join()
        # Add the rest of your multiprocessing code here

    except KeyboardInterrupt:
        p1.terminate()
        # Add the rest of your multiprocessing code here

    finally:
        print("TERMINATED...")
        time.sleep(0.1)
