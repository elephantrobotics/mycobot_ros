
import time

from pymycobot.mercury import Mercury

# cx = Mercury('/dev/ttyAMA1', 115200)

data_list = [90.78, -34.22, -138.46, -76.63, 65.83, 112.31, 59.23, 
             -128.83, -49.48, 138.13, -103.05, -79.33, 120.49, -120.74,
             7.71, -78.49, 13.33]


data_left = data_list[:7]
print('data_left:', data_left)

data_right = data_list[7: -3]
print('data_right:', data_right)

data_mid = data_list[-3:]
print('data_mid:', data_mid)

all_data = data_left + data_right + data_mid
print(all_data)