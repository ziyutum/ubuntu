import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 文件
df = pd.read_csv('csc.csv')

# 确保时间列被正确解析为 datetime 类型
df['ts_in'] = pd.to_datetime(df['ts_in'])
df['ts_out'] = pd.to_datetime(df['ts_out'])

# 计算时间差 (延迟) 并将其转换为毫秒
df['delay'] = (df['ts_out'] - df['ts_in']).dt.total_seconds() * 1000

# 计算平均值和最大值
mean_delay = df['delay'].mean()
max_delay = df['delay'].max()
max_index = df['delay'].idxmax()

# 绘制延迟图表
plt.figure(figsize=(12, 6))
plt.plot(df.index, df['delay'], marker='o', linestyle='-', color='b', label='Delay')
plt.axhline(y=mean_delay, color='r', linestyle='--', label=f'Average Delay ({mean_delay:.2f} ms)')
plt.axhline(y=max_delay, color='g', linestyle='--', label=f'Max Delay ({max_delay:.2f} ms)')

# 标记最大值
plt.scatter(max_index, max_delay, color='orange', zorder=5)
plt.annotate(f'Max Delay\n({max_index}, {max_delay:.2f} ms)', 
             xy=(max_index, max_delay), 
             xytext=(max_index + 10, max_delay + 20),
             arrowprops=dict(facecolor='orange', shrink=0.05),
             fontsize=10,
             color='black')

plt.xlabel('Index')
plt.ylabel('Delay (ms)')
plt.title('Delay between ts_in and ts_out')
plt.legend()
plt.grid(True)
plt.show()
