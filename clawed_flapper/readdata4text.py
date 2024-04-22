import scipy.io
import matplotlib.pyplot as plt
import numpy as np

mat = scipy.io.loadmat('#2024_04_06_07_07_16#Data/Ackermann_ep_230.mat')

gamma_des_data = mat['Total_desire_gamma_list']

gamma_data = mat['Total_current_gamma_list']

# print(gamma_data[0][0])
data_1 = []
data_2 = []
data_3 = []

data_des_1 = []
data_des_2 = []
data_des_3 = []

for data in gamma_data:
    data_1.append(data[0][0])
    data_2.append(data[1][0])
    data_3.append(data[2][0])
    
for des_data in gamma_des_data:
    data_des_1.append(des_data[0][0])
    data_des_2.append(des_data[1][0])
    data_des_3.append(des_data[2][0])

x = np.arange(len(gamma_data))

print(data_1)
plt.plot(x, data_1, label='gamma x')
plt.plot(x, data_2, label='gamma y')
plt.plot(x, data_3, label='gamma z')

plt.plot(x, data_des_1, label='gamma des x')
plt.plot(x, data_des_2, label='gamma des y')
plt.plot(x, data_des_3, label='gamma des z')

# Add labels and legend
plt.xlabel('X Label')
plt.ylabel('Y Label')
plt.title('Three Curves Plot')
plt.legend()

# Show plot
plt.show()