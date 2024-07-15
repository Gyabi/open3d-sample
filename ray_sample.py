# xyzの成分を合成する形で表現できないか。
# Y成分は焦点距離をピクセル換算で出せばok？

# ★明日この理論でrayを作って平面に打ち込んで曲線が描かないかチェックしてみる
# ->XYのfが一致しないのはなぜ？

import math

fov = 60.
fov = 18.
w = 640.
h = 480.

cy = 320


# 垂直方向のfをだしてみる
fov_v = fov/w*h
fx = 240/math.tan(math.radians(fov_v/2.))

xx = []
for d in range(320):
    cy = d
    fy = 320/math.tan(math.radians(fov/2.))
    a = math.atan2(cy , fy)

    xx.append(math.degrees(a))
    
anss = []
for i in range(len(xx)):
    if i == 0:
        continue
    
    anss.append(xx[i] - xx[i-1]) 
print(anss)

import matplotlib.pyplot as plt

# ヒストグラムを表示
plt.figure()
plt.hist(anss, bins=20, edgecolor='black')
plt.xlabel('Angle Difference (degrees)')
plt.ylabel('Frequency')
plt.title('Histogram of Angle Differences')
plt.grid(True)

# 折れ線グラフを表示
plt.figure()
plt.plot(anss, marker='o')
plt.xlabel('Index')
plt.ylabel('Angle Difference (degrees)')
plt.title('Line Graph of Angle Differences')
plt.grid(True)


plt.show()
    