import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.ticker import FuncFormatter

# 
def read_timestamps(file_path):
    df = pd.read_csv(file_path)
    df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y-%m-%d %H:%M:%S.%f')
    return df['timestamp']

# 
def align_timestamps(timestamps1, timestamps2):
    # 
    df1 = pd.DataFrame({'timestamp_file1': timestamps1})
    df2 = pd.DataFrame({'timestamp_file2': timestamps2})

    # 
    aligned_df = pd.merge_asof(df1.sort_values('timestamp_file1'),
                               df2.sort_values('timestamp_file2'),
                               left_on='timestamp_file1',
                               right_on='timestamp_file2',
                               direction='nearest')
    
    return aligned_df['timestamp_file1'], aligned_df['timestamp_file2']

# 计算时间戳之间的延迟
def calculate_delays(timestamps1, timestamps2):
    delays = timestamps2 - timestamps1
    delays_in_ms = delays.dt.total_seconds() * 1000000 # us
    return delays_in_ms

# 
def save_delays_to_csv(delays, output_file_path):
    # 
    directory = os.path.dirname(output_file_path)
    if not os.path.exists(directory) and directory != '':
        os.makedirs(directory)
    
    delays_df = pd.DataFrame(delays, columns=["delay"])
    delays_df.to_csv(output_file_path, index=False)

def format_y_axis(value, tick_number):
    if value >= 1_000_000:
        return f'{value / 1_000_000:.1f} ms'
    elif value >= 1_000:
        return f'{value / 1_000:.0f} μs'
    return f'{value:.0f} μs'

# 
def plot_delays(delays):
    plt.figure(figsize=(12, 6))
    
    plt.plot(delays, label='Delay (μs)', linewidth=2, color='blue', marker='o', markersize=5)
    plt.xlabel('Sample')
    plt.ylabel('Delay (μs)')
    plt.title('Delay between Timestamps')

    
    plt.gca().yaxis.set_major_formatter(FuncFormatter(format_y_axis))

    min_delay = delays.min()
    max_delay = delays.max()
    buffer = (max_delay - min_delay) * 0.1  # 
    plt.ylim(min_delay - buffer, max_delay + buffer)

    plt.legend()
    plt.grid(True)
    plt.show()

# 
def main():
    file1 = 'timestamps.csv'  # 
    file2 = 'robot_data.csv'  # 
    output_file_path = 'data_export/delays.csv'  # 

    # 
    timestamps1 = read_timestamps(file1)
    timestamps2 = read_timestamps(file2)

    # 
    timestamps1_aligned, timestamps2_aligned = align_timestamps(timestamps1, timestamps2)

    # 
    delays = calculate_delays(timestamps1_aligned, timestamps2_aligned)

    # 
    save_delays_to_csv(delays, output_file_path)

    # 
    plot_delays(delays)

if __name__ == "__main__":
    main()
