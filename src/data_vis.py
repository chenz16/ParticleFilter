import numpy as np

'''
with open("../data/map_data.txt", "r") as ins:
    array = []
    for line in ins:
        array.append(line)

print(array)
'''

import pandas as pd

data = pd.read_csv("../data/map_data.txt", sep=" ", header = None)
print(data)
