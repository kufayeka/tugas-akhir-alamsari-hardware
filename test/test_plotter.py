import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the Excel file
data = pd.read_excel('data_klimat_20230510_213713.xlsx')

# Extract the values and timestamps from the data
values = data['Value']
setpoint = data['Set Point']
timestamps = pd.to_datetime(data['Timestamp'])

# Create a line plot of the data
plt.plot(timestamps, values)
plt.plot(timestamps, setpoint, color='green', linestyle='dotted')


# Customize the plot with labels and a title
plt.xlabel('Timestamp')
plt.ylabel('Value')
plt.title('Grafik PID data klimat')

# Set the y-axis limits to 0 and 50
plt.ylim(0, 50)

# Set the y-axis tick locations and labels
yticks = range(0, 51, 10)
plt.yticks(yticks)

# Add horizontal grid lines
plt.grid(axis='y')

# Show the plot
plt.show()
