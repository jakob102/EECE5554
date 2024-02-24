#!/usr/bin/env python3

import allantools
import numpy as np
import rosbag
import math
import pylab as plt

yaw = []
pitch = []
roll = []

gyro_x = []
gyro_y = []
gyro_z = []

accel_x = []
accel_y = []
accel_z = []

mag_x = []
mag_y = []
mag_z = []

q_w = []
q_x = []
q_y = []
q_z = []

vnymr = []
time_s = []
time_n = []


bag = rosbag.Bag('/home/jakob102/gnss/src/vn_driver/python/LocationC.bag')
for topic, msg, t in bag.read_messages(topics=['/vectornav']):
   d = msg.data
   if '\x00' not in d and '00.00.' not in d:
      vnymr.append(d)
      time_s.append(msg.header.stamp.secs)
      time_n.append(msg.header.stamp.nsecs)
bag.close()

vnymr = np.array(vnymr)
vnymr1 = np.array([x.split(',') for x in vnymr], dtype=str)
time_s = np.array(time_s)
time_n = np.array(time_n)
time_n = np.divide(time_n,1e9)
time = np.add(time_s,time_n)
time = time-time[0]

print(vnymr.size)
print(time.size)

for line in vnymr:
   
   vnymr_read = str.strip(line)
   items = vnymr_read.split(",")
   label=items[0]
   
   if label=="$VNYMR":
      yaw.append(float(items[1])*(math.pi/180))           #rad (z) (psi)
      pitch.append(float(items[2])*(math.pi/180))         #rad (y) (phi)
      roll.append(float(items[3])*(math.pi/180))          #rad (x) (theta)
      mag_x.append(float(items[4]))         #unitless (magnitude)
      mag_y.append(float(items[5]))         #unitless (magnitude)
      mag_z.append(float(items[6]))         #unitless (magnitude)
      accel_x.append(float(items[7]))       #m/s^2
      accel_y.append(float(items[8]))       #m/s^2
      accel_z.append(float(items[9]))      #m/s^2
      gyro_x.append(float(items[10])*(180/math.pi))       #rad/s #after conversion now in degrees
      gyro_y.append(float(items[11])*(180/math.pi))      #rad/s
      gyro_z.append(float((items[12]).split('*')[0])*(180/math.pi)) #splitting ####*6F and splitting into two, then takes first value

yaw = np.array(yaw)
pitch = np.array(pitch)
roll = np.array(roll)
mag_x = np.array(mag_x)
mag_y = np.array(mag_y)
mag_z = np.array(mag_z)
accel_x = np.array(accel_x)
accel_y = np.array(accel_y)
accel_z = np.array(accel_z)
gyro_x = np.array(gyro_x) #np.array(gyro_x[:5000])
gyro_y = np.array(gyro_y)
gyro_z = np.array(gyro_z)

gx_oadev = allantools.oadev(gyro_x, rate=40, data_type='freq', taus='all')
gy_oadev = allantools.oadev(gyro_y, rate=40, data_type='freq', taus='all')
gz_oadev = allantools.oadev(gyro_z, rate=40, data_type='freq', taus='all')


######## computing B. the bias instability (value at the local minimum allen variance)
computed_oadevx = gx_oadev[1]
minx = np.argmin(computed_oadevx) #finds the array number with the smallest value
bx = computed_oadevx[minx]        #gives you the smallest value using the above array number
print('B, the bias instability of gyro_x is:'+ bx)                         #prints out this minimum oadev THIS is the bias instability

figx = plt.loglog(gx_oadev[0],gx_oadev[1])
plt.title("Allan Deviation")
plt.xlabel("tao (seconds)")
plt.ylabel("oadev")
plt.show()

computed_oadevy = gy_oadev[1]
miny = np.argmin(computed_oadevy)
by = computed_oadevy[miny]
print('B, the bias instability of gyro_y is:'+ by)

figy = plt.loglog(gy_oadev[0],gy_oadev[1])
plt.title("Allan Deviation")
plt.xlabel("tao (seconds)")
plt.ylabel("oadev")

plt.show()

computed_oadevz = gz_oadev[1]
minz = np.argmin(computed_oadevz)
bz = computed_oadevz[minz]
print('B, the bias instability of gyro_z is:'+ bz)

figz = plt.loglog(gz_oadev[0],gz_oadev[1])
plt.title("Allan Deviation")
plt.xlabel("tao (seconds)")
plt.ylabel("oadev")
plt.show()

######## computing N, angle random walk (at tao = 1 s)
taus2x = gx_oadev[0]                            #takes all the time values
difference_to_onex = np.absolute(taus2x-1)      #finds the absolute difference from every value to 1
difference_to_zerox = np.absolute(taus2x)        #finds the absolute difference from every value to 0
N1x_zero = difference_to_zerox.argmin()          # finds the array number of the value closest to (0), which is the diff to 0 so closest to 0.
N1x = difference_to_onex.argmin()               #finds the array number of the value closest to (0), which is the diff to 1 so closest to 1. 
Nx_zero = computed_oadevx[N1x_zero]             #finds the oadev value at t = 0
Nx = computed_oadevx[N1x]                       #finds the oadev value at t = 1

slopex = Nx-Nx_zero                             #finds the slope of the line change in y over 1 second.
print('N, the angle random walk value of gyro_x is:' +slopex)                                   #prints the value of the slope of the line

taus2y = gy_oadev[0]
difference_to_oney = np.absolute(taus2y-1)
difference_to_zeroy = np.absolute(taus2y)
N1y_zero = difference_to_zeroy.argmin()
N1y = difference_to_oney.argmin()
Ny_zero = computed_oadevy[N1y_zero]
Ny = computed_oadevy[N1y]

slopey = Ny-Ny_zero
print('N, the angle random walk value of gyro_y is:' + slopey)

taus2z = gz_oadev[0]
difference_to_onez = np.absolute(taus2z-1)
difference_to_zeroz = np.absolute(taus2z)
N1z_zero = difference_to_zeroz.argmin()
N1z = difference_to_onez.argmin()
Nz_zero = computed_oadevz[N1z_zero]
Nz = computed_oadevz[N1z]

slopez = Nz-Nz_zero
print('N, the angle random walk value of gyro_z is:'+ slopez)

#K = #rate random walk (slope after local minimum)


tau_at_minx = taus2x[minx]          #inputs the array number with the smallest oadev value and outputs the tau at that min

next_taux = taus2x[minx+40]          #finds the tau value of a point one sec after the min
next_bx = computed_oadevx[minx+40]   #finds the oadev value of a point one sec after the min

slopex1 = (tau_at_minx-next_taux) / (bx-next_bx) #finds the slope after the minimum, this is the K value

print('K, the rate random walk value of gyro_x is:' + slopex1)

tau_at_miny = taus2y[miny]

next_tauy = taus2y[miny+40]
next_by = computed_oadevy[miny+40]

slopey1 = (tau_at_miny-next_tauy) / (by-next_by)

print('K, the rate random walk value of gyro_y is:'+ slopey1)

tau_at_minz = taus2z[minz]

next_tauz = taus2z[minz+40]
next_bz = computed_oadevz[minz+40]

slopez1 = (tau_at_minz-next_tauz) / (bz-next_bz)

print('K, the rate random walk value of gyro_z is:'+ slopez1)