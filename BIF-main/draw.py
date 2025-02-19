import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import argparse
import math
from matplotlib.font_manager import FontProperties
font = FontProperties(fname='/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf', size=20)
from functools import reduce
def str2int(s):
    def fn(x,y):
        return x*10+y
    def char2num(s):
        return {'0':0,'1':1,'2':2,'3':3,'4':4,'5':5,'6':6,'7':7,'8':8,'9':9}[s]
    return reduce(fn,map(char2num,s))

sys.path.append("..")

seq = "00"
N = 0
num_candidates=300

#read kitti pose
traj = np.loadtxt("./poses/"+seq +".txt")
loop_results = np.loadtxt("./results/BIF_loop_"+seq+".txt")
gt = {}
for line in open("./gt_loop/gt_loop_"+seq+".txt", "r"):
    if line.strip():
        frames_id = line.split()
        if len(frames_id) >= 2:
            if str2int(frames_id[0]) - str2int(frames_id[1]) > num_candidates:
                gt[frames_id[0]] = 1
                N = N+1
            else:
                gt[frames_id[0]] = 0
print('the number of revisit events:',N)

x_cord = traj[:,3]
z_cord = traj[:,11]

#dist 0
th0 = []
PRE = []
REC= []
F1_score=[]
min_similarity = loop_results[:, 2].min()
max_similarity = loop_results[:, 2].max()

#print(min_similarity, max_similarity)
for i in np.arange(min_similarity, max_similarity + (max_similarity-min_similarity) * 1.0 /100, (max_similarity-min_similarity) * 1.0 /100):
    TP = 0 
    P = 0
    for j in range(0, loop_results.shape[0]):
        if loop_results[j][2] <= i:
            P = P+1
            if loop_results[j][3] == 1.0:
                TP = TP+1
    
    Recall = TP * 1.0 / N
    Precision = TP * 1.0 / P
    th0.append(i)
    REC.append(Recall)
    PRE.append(Precision)
    F1_score.append(2*Recall*Precision/(Recall+Precision))
print('F1 max score:',max(F1_score))  

R_P100 = 0
thres=0;
for i in range(len(th0)-1):
    if PRE[i]==1.0 and PRE[i+1]!=1.0:
        thres = th0[i]
        R_P100=REC[i]
        break 
print('EP metric:',0.5*(R_P100+1)) 
fig1 = plt.figure(1) 

plt.xlabel('Recall', color='b', fontproperties=font, fontstyle='normal')
plt.ylabel('Precision',color='b',fontproperties=font, fontstyle='normal')
plt.tick_params(labelsize=24)
plt.plot(REC, PRE,  "r-o", label = "Image fingerprint", linewidth=3.0)
plt.legend(loc="lower left", prop=font)
new_ticks=np.linspace(0,1,11)
plt.xticks(new_ticks,fontproperties=font)
plt.yticks(new_ticks,fontproperties=font)
plt.title('Precision-Recall Curve', fontproperties=font)

ax=plt.gca()
ax.yaxis.grid(color='black', linestyle='-', linewidth=1,alpha=0.1)
ax.xaxis.grid(color='black', linestyle='-', linewidth=1,alpha=0.1)


fig2 = plt.figure(2)
plt.title('Trajectory truth and results',fontproperties=font)
plt.xlabel('x', fontproperties=font, fontstyle='normal')
plt.ylabel('z',fontproperties=font, fontstyle='normal')
plt.tick_params(labelsize=18)
plt.xticks(fontproperties=font)
plt.yticks(fontproperties=font)
plt.plot(x_cord, z_cord,  "k", linewidth=1.5)

for i in range(len(loop_results[:,0])):
    if gt[str(int(loop_results[i][0]))]:
        index = int(int(loop_results[i][0])-1)
        plt.scatter(x_cord[index], z_cord[index], c="g",alpha=0.2)
    if loop_results[i][2] <= thres and loop_results[i][3] == 1:
        index = int(loop_results[i][0]-1)
        plt.scatter(x_cord[index], z_cord[index], c="r")
    if loop_results[i][2] <= thres and loop_results[i][3] == 0:
        index = int(loop_results[i][0]-1)
        plt.scatter(x_cord[index], z_cord[index], c="b")

plt.show()
