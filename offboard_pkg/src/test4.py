#!/usr/bin/env python
import numpy as np 
import pandas as pd


#a = np.random.random((3,4))
a = np.array([[ 0.73019381,0.94153124,0.23286164, 0.79864706],
              [ 0.70969246,  0.38610202,  0.5731873,   0.0724901 ],
              [ 0.38111594,  0.49668992,  0.75657957,  0.35285703]])

df = pd.DataFrame(a, columns=['a','b','c','d'])


print(df['c'].argmin())