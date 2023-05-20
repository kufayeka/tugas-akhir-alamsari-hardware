# Import necessary libraries
import sys
import global_variables as gv
import multiprocessing
import test_vars as tv

# Function to update the value
def update_value(option, new_value):
    if option == "kp":
        tv.kp = new_value
    elif option == "ki":
        gv.PID_ki.set(new_value)
    elif option == "kd":
        gv.PID_kd.set(new_value)
    elif option == "set_point":
        gv.PID_set_point.set(new_value)
    elif option == "max_interval":
        gv.PWM_max_interval.set(new_value)
    elif option == "min_interval":
        gv.PWM_min_interval.set(new_value)
    print(f"{option} updated to {new_value}")

# Main loop
while True:
    # Display the options with respective values
    kp = tv.kp
    ki = gv.PID_ki.get()
    kd = gv.PID_kd.get()
    set_point = gv.PID_set_point.get()
    max_interval = gv.PWM_max_interval.get()
    min_interval = gv.PWM_min_interval.get()

    print("\nOptions:")
    print("- kp (Current value: {})".format(kp))
    print("- ki (Current value: {})".format(ki))
    print("- kd (Current value: {})".format(kd))
    print("- set_point (Current value: {})".format(set_point))
    print("- max_interval (Current value: {})".format(max_interval))
    print("- min_interval (Current value: {})".format(min_interval))
    print("- exit")

    # Prompt for the option
    option = input("\nEnter an option: ")

    # Check if the option is valid
    if option == "exit":
        sys.exit(0)
    elif option not in ["kp", "ki", "kd", "set_point", "max_interval", "min_interval"]:
        print("Invalid option. Please try again.")
        continue

    # Prompt for the new value
    new_value = input("Change value to: ")

    # Update the value
    try:
        new_value = float(new_value)
        update_value(option, new_value)
    except ValueError:
        print("Invalid value. Please try again.")
