import datetime
import minimalmodbus
import openpyxl
from time import sleep

# Set up sensor 1: address=0x001
sensor1 = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
sensor1.serial.baudrate = 9600
sensor1.serial.timeout = 0.2
sensor1.mode = minimalmodbus.MODE_RTU

# Set up sensor 2: address=0x002
sensor2 = minimalmodbus.Instrument('/dev/ttyUSB0', 2)
sensor2.serial.baudrate = 9600
sensor2.serial.timeout = 0.2
sensor2.mode = minimalmodbus.MODE_RTU

def read_climate_sensors():
    # Read values from sensor 1
    temperature1 = sensor1.read_register(1, 1)
    humidity1 = sensor1.read_register(0, 1)

    # Read values from sensor 2
    temperature2 = sensor2.read_register(1, 1)
    humidity2 = sensor2.read_register(0, 1)

    return temperature1, temperature2, humidity1, humidity2

def record_data():
    excel_name = f"pengujian_kinerja_sensor_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
    wb = openpyxl.Workbook()
    ws = wb.active
    ws.title = 'Data'
    ws.append(['Timestamp', 'Temperature 1', 'Temperature 2', 'Humidity 1', 'Humidity 2'])

    while True:
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        temperature1, temperature2, humidity1, humidity2 = read_climate_sensors()
        ws.append([timestamp, temperature1, temperature2, humidity1, humidity2])
        wb.save(excel_name)

        # Display information in CMD
        print(f"Timestamp: {timestamp}")
        print(f"Temperature 1: {temperature1} °C")
        print(f"Temperature 2: {temperature2} °C")
        print(f"Humidity 1: {humidity1} %")
        print(f"Humidity 2: {humidity2} %")
        print("-----------------------------")

        sleep(5)

# Start recording data
record_data()
