import numpy as np
import matplotlib.pyplot as plt
import csv

before_vkf = []
after_vkf = []
rows = []

with open("result/calibration_result.csv", 'r') as file:
    file = csv.reader(file)
    for row in file:
        rows.append(row)
before_vkf = [float(row[0]) for row in rows]
after_vkf = [float(row[1]) for row in rows]

plt.plot(before_vkf,'r*')
plt.plot(after_vkf,'bo')
plt.title("Pose refinement error")
plt.legend(['Before refinement', 'After refinement'])
plt.xlabel("Pose number")
plt.ylabel("Position error")
plt.show()
