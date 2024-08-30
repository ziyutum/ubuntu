import matplotlib.pyplot as plt

# 
with open('callback_times.txt', 'r') as file:
    lines = file.readlines()

# 
callback_times = [float(line.strip()) for line in lines[1:]]
average_time = sum(callback_times) / len(callback_times)
# 
x = list(range(len(callback_times)))

# 
plt.figure(figsize=(10, 6))
plt.plot(x, callback_times, label='Callback Execution Time (μs)')
plt.axhline(y=average_time, color='red', linestyle='--', label=f'Average Time: {average_time:.2f} μs')
plt.text(len(x) - 1, average_time, f'average value: {average_time:.2f} μs', color='red', ha='right', va='bottom')

plt.xlabel('Callback Index')
plt.ylabel('Execution Time (μs)')
plt.title('Callback Execution Time Over Iterations for Cartesian Motion Control')
plt.legend()
plt.grid(True)
plt.show()
