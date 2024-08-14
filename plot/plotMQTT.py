import pandas as pd
import matplotlib.pyplot as plt

# Read CSV files
timestamps_df = pd.read_csv('MQTTreceivemessages.csv', parse_dates=['timereceive'])
robot_data_df = pd.read_csv('MQTTpublisher.csv', parse_dates=['timestamp'])

# Ensure timestamp columns are in datetime format
timestamps_df['timestamp'] = pd.to_datetime(timestamps_df['timereceive'])
robot_data_df['timestamp'] = pd.to_datetime(robot_data_df['timestamp'])

# Check if the lengths of both dataframes are the same
if len(timestamps_df) == len(robot_data_df):
    # Calculate the time difference
    timestamps_df['robot_timestamp'] = robot_data_df['timestamp'].values
    timestamps_df['delay'] = -(timestamps_df['robot_timestamp'] - timestamps_df['timestamp']).dt.total_seconds() * 1000  # Convert to milliseconds

    # Convert data to 1D arrays
    x_data =x_data = timestamps_df.index
    y_data = timestamps_df['delay'].values
  # Find the maximum delay and its index
    max_delay = y_data.max()
    max_index = y_data.argmax()
    average_delay = y_data.mean()
    # Plotting
    plt.figure(figsize=(12, 8))
    plt.plot( y_data, marker='o', linestyle='-', color='b')
    plt.axhline(y=average_delay, color='green', linestyle='--', label=f'Average Delay: {average_delay:.2f} ms')
    plt.scatter(x_data[max_index], max_delay, color='red', zorder=5)
    plt.annotate(f'Max Delay: {max_delay:.2f} ms', 
                 (x_data[max_index], max_delay), 
                 textcoords="offset points", 
                 xytext=(10,10), 
                 ha='center', 
                 fontsize=12, 
                 color='red',
                 arrowprops=dict(facecolor='red', shrink=0.05))
    plt.text(x_data[-1], average_delay- 0.15 * y_data.max(), f'Average Delay: {average_delay:.2f} ms', 
             horizontalalignment='right', 
             verticalalignment='bottom', 
             fontsize=12, 
             color='green', 
             bbox=dict(facecolor='white', alpha=0.5))
    plt.xlabel('Number of sent messages over TCP')
    plt.ylabel('Delay (milliseconds)')
    plt.title('Delay Between Time Delays for Receiving and Sending Robot State Data Message Over TCP')
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.legend()

    # Show the plot
    plt.show()
else:
    print("Timestamp counts do not match. Please check the data in both files.")
