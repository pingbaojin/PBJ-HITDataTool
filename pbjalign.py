# 双矢量初始对准算法
# pbj20270403
# 坐标系定义：东北天坐标系
# 输入：ws(rad,增量式),fs(m/s2,增量式),pos0(rad)
# 输出姿态角：北偏西为正

import numpy as np
from math import pi
import math

PI=pi
class GeoModelWGS84:
    Ra = 6378137.0000000000
    Rb = 6356752.3142451793
    e = (1.0 / 298.257223563)
    wie = 7.292115147e-5
    GM = 3.986004418e14
    g0 = 9.7803267714
    e2 = 0.0066943799013
class earth:
    def get_grav(blh:np.ndarray)->float:
        sin2=math.sin(blh[0])**2
        return  GeoModelWGS84.g0 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2**2) + \
               blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + \
               0.0000000000007211 * blh[2]**2
def Align(ws:np.ndarray,fs:np.ndarray,blh:np.ndarray)->np.ndarray:
    g=earth.get_grav(blh)
    gn=np.array([0,0,-g])
    lat=blh[0]
    W_ie=GeoModelWGS84.wie
    sin_lat=math.sin(lat)
    cos_lat=math.cos(lat)
    tan_lat=math.tan(lat)
    W_ien=np.zeros(3)
    W_ien[0]=0
    W_ien[1]=W_ie*cos_lat
    W_ien[2]=W_ie*sin_lat
    T12=np.cross(-fs,ws)
    T13=np.cross(T12,-fs)
    fs=fs/np.linalg.norm(fs)
    T12=T12/np.linalg.norm(T12)
    T13=T13/np.linalg.norm(T13)
    T1=np.vstack((fs,T12,T13)).T
    # print(T1)


    T22=np.cross(gn,W_ien)
    T23=np.cross(T22,gn)
    gn=gn/np.linalg.norm(gn)
    T22=T22/np.linalg.norm(T22)
    T23=T23/np.linalg.norm(T23)
    T2=np.vstack((gn,T22,T23)).T
    T1_inv=np.linalg.inv(T1)
    T=np.dot(T2,T1_inv)
    att=np.zeros(3)
    pitchmain = math.asin(T[2, 1])
    rollmain = math.atan(-T[2, 0] / T[2, 2])
    yawmain = math.atan(-T[0, 1] / T[1, 1])
    print(-T[0, 1] )
    print(T[1, 1])
    # rollmain = math.atan2(-T[2, 0], T[2, 2])  # 使用 atan2 替代 atan
    # yawmain = math.atan2(-T[0, 1], T[1, 1])  # 使用 atan2 替代 atan
	
    att[0] = pitchmain
    att[1]=rollmain
    att[2]=yawmain
    # if T[2, 2] > 0:
    #     att[1] = rollmain
    # elif rollmain<0:
    #     att[1] = rollmain+PI
    # else: 
    #     att[1] = rollmain-PI
    # print(att[2]/pi*180)

    if (T[1, 1] < 0):
         att[2] = yawmain + PI


    # if (-T[0, 1]>0) and (T[1, 1] < 0):
    #     att[2] = yawmain + PI
    # elif (-T[0, 1]<=0) and (T[1, 1] < 0):
    #     att[2] = yawmain + PI
    # elif yawmain > 0:
    #     att[2] = yawmain
    # else:
    #     att[2] = yawmain + 2*PI
    
    print(att[2]/pi*180)
    if att[2]>PI:
        att[2]-=2*PI
    elif att[2]<-PI:
        att[2]+=2*PI
    else:
        att[2]=att[2]
    return att


# aligndata = [0,0,0,0,0,0]
# len = 12000
# with open("G:\\1.data_collect\\2025-03-27-car1\\2025-03-27-14-42-13\\IMU2025-03-27-14-42-13.txt", "r", encoding="utf-8") as file:
#     # 读取文件的每一行，返回一个列表
#     for i in range(len):
#         line = file.readline()
#         words = line.split()
#         aligndata[0] = aligndata[0]+float(words[1])
#         aligndata[1] = aligndata[1]+float(words[2])
#         aligndata[2] = aligndata[2]+float(words[3])
#         aligndata[3] = aligndata[3]+float(words[4])
#         aligndata[4] = aligndata[4]+float(words[5])
#         aligndata[5] = aligndata[5]+float(words[6])
#     ws_tmp=np.array(aligndata[0:3])/len/0.01
#     fs_tmp=np.array(aligndata[3:6])/len/0.01

# ws= np.array([ws_tmp[1],ws_tmp[0],-ws_tmp[2]])
# fs= np.array([fs_tmp[1],fs_tmp[0],-fs_tmp[2]])

# print(ws)
# print(fs)
# with open("G:\\1.data_collect\\2025-03-27-car1\\2025-03-27-14-42-13\\GNSS-RTK2025-03-27-14-42-13.txt", "r", encoding="utf-8") as file:
#     line = file.readline()
#     words = line.split()
#     pos0=np.array([float(words[1])/180*pi,float(words[2])/180*pi,float(words[3])])

# print(pos0)
# att0=Align(ws,fs,pos0)
# print(att0/pi*180)


# 测试数据
# -2.70647144365785e-07	4.38459901955650e-07	-5.22366400332709e-07	0.000442416706836017	0.000327001034026252	-0.0980669239226977
# 0.798102836329702	2.21004235393546	450

# # 0deg
# ws=np.array([0,	6.04299453844936e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([1.30694564356065e-21,	3.01976591637292e-15,	0.0979513850504081])/0.01

# 45deg
# ws=np.array([4.27304241681069e-07,	4.27304241681069e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([2.13529788686528e-15,	2.13529602736327e-15,	0.0979513850504081])/0.01

# # 90deg
# ws=np.array([6.04299453844936e-07,	3.70026724960713e-23,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([3.01976591637292e-15,	-1.30363692111044e-21,	0.0979513850504081])/0.01

# # 130deg
# ws=np.array([4.62920238597739e-07,	-3.88436201471881e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([2.31327405753603e-15,	-1.94106911600942e-15,	0.0979513850504081])/0.01	

# 180deg
# ws=np.array([7.40053386817813e-23,	-6.04299453844936e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([-1.30694564356065e-21,	-3.01976591637292e-15	,0.0979513850504081])/0.01	
	
# # 220deg
ws=np.array([-3.88436201471881e-07,	-4.62920238597739e-07,	4.08131845376033e-07])/0.01/180*pi
fs=np.array([-1.94106911600942e-15,	-2.31327406084475e-15,	0.0979513850504081])/0.01	

# 270deg
# ws=np.array([-6.04299453844936e-07,	-1.11008011177853e-22,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([-3.01976591637292e-15,	1.30363692111044e-21,	0.097951385050408])/0.01		

# 330deg
# ws=np.array([-3.02149726922468e-07,	5.23338678522779e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([-1.50988182598488e-15,	2.61519465024564e-15,	0.0979513850504081])/0.01	


# ws=np.array([4.27304241681069e-07,	4.27304241681069e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([2.13529788686528e-15,	2.13529602736327e-15,	0.0979513850504081])/0.01
# ws=np.array([4.27304241681069e-07,	-4.27304241681069e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([2.13529602736327e-15,	-2.13529788355684e-15,	0.0979513850504081])/0.01
# ws=np.array([-4.27304241681069e-07,	-4.27304241681069e-07,	4.08131845376033e-07])/0.01/180*pi
# fs=np.array([-2.13529788686556e-15,	-2.13529602736327e-15,	0.0979513850504081])/0.01

# # -45deg
# ws=np.array([-4.27304241681069e-07,4.27304241681069e-07	,4.08131845376033e-07])/0.01/180*pi
# fs=np.array([-2.13529602736327e-15,	2.13529788686528e-15,	0.0979513850504081])/0.01


pos0 = np.array([0.594010768144431	,1.89848934641274	,450])
att0=Align(ws,fs,pos0)
print(att0/pi*180)