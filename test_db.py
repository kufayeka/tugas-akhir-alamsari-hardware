import sqlite3
import time
import datetime
import random

# Define the database file name based on the current timestamp
db_name = f"data_klimat_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.db"

# Create a connection to the database
conn = sqlite3.connect(db_name)

# Create a table to store the records
conn.execute('''CREATE TABLE IF NOT EXISTS data_klimat
                 (value REAL, timestamp TEXT)''')

# Insert a new record every second
while True:
    value = random.uniform(0, 1) # Generate a random value between 0 and 1
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S') # Get the current timestamp
    conn.execute("INSERT INTO data_klimat (value, timestamp) VALUES (?, ?)", (value, timestamp))
    conn.commit()
    time.sleep(1)

# Close the database connection when done
conn.close()
