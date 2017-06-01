# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

x = [0,1,2,3,4,5,6,7,8,9]
y = [0.1,0.2,0.9,0.2,0.6,0.6,0.1,0.2,0.3,0.5]
plt.scatter(x, y) # 如果没有第一个参数 x，图形的 x 坐标默认为数组的索引
plt.show() # 显示图形