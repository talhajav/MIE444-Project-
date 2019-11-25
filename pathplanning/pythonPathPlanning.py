#Origin at top left corner

import matplotlib.pyplot as plt
import math
import numpy as np

len=8;
width=4;
mapArray = (width,len)
map1 = (np.zeros(mapArray))

#adding blocks where locations are where we cannot drive
map1[1,2]=map1[2,1]=map1[2,3]=map1[2,4]=map1[2,3]=map1[2,6]=map1[3,6]=map1[0,6]=map1[0,4]=1

#locations where LZ is marked by 2
map1[0,0]=map1[0,1]=map1[1,1]=map1[1,0]=2

#location of delivery points marked by 3
map1[2,2]=map1[0,5]=map1[0,7]=map1[3,7]=3

print(map1)
