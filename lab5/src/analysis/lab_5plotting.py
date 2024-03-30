import allantools
import numpy as np
import rosbag
import math
import pylab as plt
from scipy.optimize import least_squares
import scipy.integrate as integrate
from scipy.signal import butter, sosfilt, freqz
import matplotlib.pyplot as plt

roll_q_circle = []
pitch_q_circle = []
yaw_q_circle = []

roll_q_driving = []
pitch_q_driving = []
yaw_q_driving = []

time_s_circle_imu = []
time_n_circle_imu = []

alt_circle        = []
utm_east_circle   = []
utm_north_circle  = []
time_s_circle_gps = []
time_n_circle_gps = []

qx_circle = []
qy_circle = []
qz_circle = []
qw_circle = []

gyro_x_circle = []
gyro_y_circle = []
gyro_z_circle = []

accel_x_circle = []
accel_y_circle = []
accel_z_circle = []

mag_x_circle = []
mag_y_circle = []
mag_z_circle = []

velocity1 = []

bag_circle = rosbag.Bag('data_going_in_circles.bag')
bag_driving = rosbag.Bag('data_driving_brookline.bag')

for topic, msg, t in bag_circle.read_messages(topics=['/imu']):

    qx_circle.append(msg.imu.orientation.x)
    qy_circle.append(msg.imu.orientation.y)
    qz_circle.append(msg.imu.orientation.z)
    qw_circle.append(msg.imu.orientation.w)

    gyro_x_circle.append(msg.imu.angular_velocity.x)
    gyro_y_circle.append(msg.imu.angular_velocity.y)
    gyro_z_circle.append(msg.imu.angular_velocity.z)

    accel_x_circle.append(msg.imu.linear_acceleration.x)
    accel_y_circle.append(msg.imu.linear_acceleration.y)
    accel_z_circle.append(msg.imu.linear_acceleration.z)

    mag_x_circle.append(msg.mag_field.magnetic_field.x)
    mag_y_circle.append(msg.mag_field.magnetic_field.y)
    mag_z_circle.append(msg.mag_field.magnetic_field.z)

    time_s_circle_imu.append(msg.header.stamp.secs)
    time_n_circle_imu.append(msg.header.stamp.nsecs)
bag_circle.close()

qx_circke=np.array(qx_circle)
qy_circle=np.array(qy_circle)
qz_circle=np.array(qz_circle)
qw_circle=np.array(qw_circle)

time_s_circle_imu=np.array(time_s_circle_imu)
time_n_circle_imu=np.array(time_n_circle_imu)

bag_circle = rosbag.Bag('data_going_in_circles.bag')
bag_driving = rosbag.Bag('data_driving_brookline.bag')

for topic, msg, t in bag_circle.read_messages(topics=['/gps']):
    alt_circle.append(msg.altitude)      
    utm_east_circle.append(msg.utm_easting) 
    utm_north_circle.append(msg.utm_northing) 
    time_s_circle_gps.append(msg.header.stamp.secs)
    time_n_circle_gps.append(msg.header.stamp.nsecs)
    
bag_circle.close()

time_s_circle_gps=np.array(time_s_circle_gps)
time_n_circle_gps=np.array(time_n_circle_gps)

time_s_driving_imu = []
time_n_driving_imu = []

alt_driving        = []
utm_east_driving   = []
utm_north_driving  = []
time_s_driving_gps = []
time_n_driving_gps = []

qx_driving = []
qy_driving = []
qz_driving = []
qw_driving = []

gyro_x_driving = []
gyro_y_driving = []
gyro_z_driving = []

accel_x_driving = []
accel_y_driving = []
accel_z_driving = []

mag_x_driving = []
mag_y_driving = []
mag_z_driving = []

bag_circle = rosbag.Bag('data_going_in_circles.bag')
bag_driving = rosbag.Bag('data_driving_brookline.bag')

for topic, msg, t in bag_driving.read_messages(topics=['/imu']):

    qx_driving.append(msg.imu.orientation.x)
    qy_driving.append(msg.imu.orientation.y)
    qz_driving.append(msg.imu.orientation.z)
    qw_driving.append(msg.imu.orientation.w)

    gyro_x_driving.append(msg.imu.angular_velocity.x)
    gyro_y_driving.append(msg.imu.angular_velocity.y)
    gyro_z_driving.append(msg.imu.angular_velocity.z)

    accel_x_driving.append(msg.imu.linear_acceleration.x)
    accel_y_driving.append(msg.imu.linear_acceleration.y)
    accel_z_driving.append(msg.imu.linear_acceleration.z)

    mag_x_driving.append(msg.mag_field.magnetic_field.x)
    mag_y_driving.append(msg.mag_field.magnetic_field.y)
    mag_z_driving.append(msg.mag_field.magnetic_field.z)

    np.array(time_s_driving_imu.append(msg.header.stamp.secs))
    np.array(time_n_driving_imu.append(msg.header.stamp.nsecs))
bag_driving.close()

gyro_z_driving=np.array(gyro_z_driving)
qx_driving=np.array(qx_driving)
qy_driving=np.array(qy_driving)
qz_driving=np.array(qz_driving)
qw_driving=np.array(qw_driving)

time_s_driving_imu=np.array(time_s_driving_imu)
time_n_driving_imu=np.array(time_n_driving_imu)

bag_circle = rosbag.Bag('data_going_in_circles.bag')
bag_driving = rosbag.Bag('data_driving_brookline.bag')

for topic, msg, t in bag_driving.read_messages(topics=['/gps']):
    alt_driving.append(msg.altitude)      
    np.array(utm_east_driving.append(msg.utm_easting)) 
    np.array(utm_north_driving.append(msg.utm_northing)) 
    np.array(time_s_driving_gps.append(msg.header.stamp.secs))
    np.array(time_n_driving_gps.append(msg.header.stamp.nsecs))
bag_driving.close()

utm_east_driving=np.array(utm_east_driving)
utm_north_driving=np.array(utm_north_driving)
time_s_driving_gps=np.array(time_s_driving_gps)
time_n_driving_gps=np.array(time_n_driving_gps)

time_circle_gps = time_s_circle_gps+(time_n_circle_gps/1e9)
time_circle_gps = time_circle_gps-time_circle_gps[0]

time_circle_imu = time_s_circle_imu+(time_n_circle_imu/1e9)
time_circle_imu = time_circle_imu-time_circle_imu[0]

time_driving_gps= time_s_driving_gps+(time_n_driving_gps/1e9)
time_driving_gps = time_driving_gps-time_driving_gps[0]

time_driving_imu = time_s_driving_imu+(time_n_driving_imu/1e9)
time_driving_imu = time_driving_imu-time_driving_imu[0]

#####################PLOT 1
#This function describes my mapping from measured data back to a circle
def distortion_model(X_meas, dist_params):
    x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
    y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
    X = np.array([x,y])
    return X
    
#This function finds the difference between a circle and my transformed measurement
def residual(p, X_mag, X_meas):
    return (X_mag - distortion_model(X_meas, p)).flatten()

#Completely made up data set for "perfect" magnetometer readings
field_strength = 20509e-9 #The horizontal magnetic field strength in Boston is approx 20,500 nT 
angle = np.linspace(-4*np.pi, 4*np.pi, 4213)
x_mag = field_strength * np.sin(angle) 
y_mag = field_strength * np.cos(angle) 
X_mag = np.array([x_mag, y_mag])

#More made up data describing the distortion of the magnetic field.
x_meas = mag_x_circle
y_meas = mag_y_circle
X_meas = np.array([x_meas, y_meas])

#Least squares optimization to find model coefficients
p0 = [0,0,0,0,0,0]
lsq_min = least_squares(residual, p0, args=(X_mag, X_meas))

print("least square fitting values are: ")
print(lsq_min.x)

X_model = distortion_model(X_meas, lsq_min.x) 

#Plotting ellipse and lsq
plt.style.use("seaborn-v0_8-dark")

fig, ax = plt.subplots()
ax.grid()
ax.plot(X_mag[0],X_mag[1])
ax.scatter(X_model[0],X_model[1], label="calibrated data")
ax.scatter(X_meas[0],X_meas[1], label="measured data")
ax.axis('equal')
ax.legend()
plt.xlabel('X component of magnetic field (T)')
plt.ylabel('Y component of magnetic field (T)')
plt.title('Calibration of Magnetometer before and after')
#plt.show()

####################PLOT 2 CIRCLE

#fig, (aa1, aa2) = plt.subplots(1, 2)

yaw = np.arctan2(mag_x_circle, mag_y_circle)
#aa1.scatter(time_circle_imu, yaw)

yaw_corrected = np.arctan2(X_mag[0],X_mag[1])
#aa2.scatter(time_circle_imu,yaw_corrected)

#aa1.set_xlabel("Time [sec]")
#aa2.set_ylabel("")
#aa1.set_title("Magnetometer yaw estimation (CIRCLE)before c")
#plt.show()

###################PLOT 2 DRIVING
fig, (ab1) = plt.subplots(1,1)

yaw = np.arctan2(mag_x_driving, mag_y_driving)
ab1.plot(time_driving_imu, yaw, label='Before calibration')

x_meas = mag_x_driving
y_meas = mag_y_driving
X_meas = np.array([x_meas, y_meas])

dist_params = [-8.34822679e-01, -1.67506046e-02, -1.47106076e-01, 7.61381676e-01 , 5.09501278e-05 , 7.80980239e-06]
x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
X = np.array([x,y])

yaw_corrected = np.arctan2(X[0],X[1])
yaw_corrected = np.mod(yaw_corrected + np.pi, 2 * np.pi) - np.pi
ab1.plot(time_driving_imu,yaw_corrected, label='After calibration')

ab1.set_xlabel("Time [sec]")
ab1.set_ylabel("Heading [radians]")
ab1.set_title("Magnetometer Yaw Estimation Before and After Calibration")
ab1.grid()
ab1.legend(loc=1)

#plt.show()

#####################PLOT 3

fig, (ac1) = plt.subplots(1)

rotation_z = -integrate.cumulative_trapezoid(((gyro_z_driving*(180/np.pi))), time_driving_imu, initial = 0)-np.pi 
ac1.plot(time_driving_imu,rotation_z)

ac1.grid()
ac1.set_xlabel("Time [sec]")
ac1.set_ylabel("Heading [radians]")
ac1.set_title("Gyroscope Yaw Estimation using Integrated Rotation Rate around Z")

#plt.show()

#####################PLOT 4

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

# Setting filter requirements
order = 1 #you can increase this to make the filter "sharper"
sampl_freq = 40 #change to sampling frequency of your data collection
cutoff_freq = 0.15 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

# Creating some made up, noisy data tp show filter properties
data = yaw_corrected
t = time_driving_imu

# Filtering and plotting
sos, y = butter_filter(data, cutoff_freq, sampl_freq, "lowpass", order)

# Setting filter requirements
order = 1 #you can increase this to make the filter "sharper"
sampl_freq = 40 #change to sampling frequency of your data collection
cutoff_freq = 1 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

# Creating some made up, noisy data tp show filter properties
data1 = rotation_z
t = time_driving_imu

# Filtering and plotting
sos, y1 = butter_filter(data1, cutoff_freq, sampl_freq, "highpass", order)

mag_yaw = y
gyro_yaw = y1
a = 0.4

fused = mag_yaw*a+(1-a)*gyro_yaw

fig, (ad1, ad2, ad3) = plt.subplots(3, 1)

ad1.plot(t, data, "b-", label="data")
ad1.plot(t, y, "g-", linewidth=2, label="filtered data")
filtered_magnetometer = y 

ad2.plot(t, data1,"b-", label="data")
ad2.plot(t, y1, "g-", linewidth=2, label="filtered data")
#ad2.set_ylim(-0.2,0.2)

ad3.plot(t,fused,"b-")

ad1.set_xlabel("Time [sec]")
ad1.set_ylabel("Heading [radians]")
ad1.set_title("Low-Pass Filtered Magnetometer Yaw")

ad2.set_xlabel("Time [sec]")
ad2.set_ylabel("Heading [radians]")
ad2.set_title("High-Pass Filtered Gyroscope Yaw")

ad3.set_xlabel("Time [sec]")
ad3.set_ylabel("Heading [radians]")
ad3.set_title("Complementary Filter Sensor Fusion of Magnetomer and Gyrometer")

ad2.grid()
ad3.grid()
ad1.legend(loc=1)
ad2.legend(loc=1)

plt.subplots_adjust(hspace=0.35)

########################### PLOT 5

fig, (ae1, ae2) = plt.subplots(2, 1)


order = 3 #you can increase this to make the filter "sharper"
sampl_freq = 40 #change to sampling frequency of your data collection
cutoff_freq = 0.1 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

# Creating some made up, noisy data tp show filter properties
data = accel_x_driving
data = data - np.mean(data)
t = time_driving_imu

# Filtering and plotting
sos, y = butter_filter(data, cutoff_freq, sampl_freq, "lowpass", order)

velocityB = integrate.cumulative_trapezoid(accel_x_driving, time_driving_imu, initial = 0) 
velocityA = integrate.cumulative_trapezoid(y, time_driving_imu, initial = 0) 

ae1.plot(time_driving_imu,velocityB,label = 'Before Adjustments')
ae1.plot(time_driving_imu,velocityA, label = 'After Adjustments (Moved Mean & Low-Pass filter)')

ae1.set_xlabel("Time [sec]")
ae1.set_ylabel("Velocity [m/s]")
ae1.set_title("Forward Velocity from Accelerometer Before and After Adjustments")
ae1.grid()

ae1.legend(loc=1)

ae2.plot(time_driving_imu,velocityA)

ae2.set_xlabel("Time [sec]")
ae2.set_ylabel("Velocity [m/s]")
ae2.set_title("After Adjustments (Moved Mean & Low-Pass filter")
ae2.grid()

#plt.show()

################## PLOT 6

fig, (af) = plt.subplots(1)

utm_north_driving = utm_north_driving-utm_north_driving[0]
utm_east_driving = utm_east_driving-utm_east_driving[0]

position = np.sqrt(np.square(utm_east_driving) + np.square(utm_north_driving))


# Calculate velocities using a for loop
for i in range(0, len(position)):
    north = utm_north_driving[i]-utm_north_driving[i-1]
    east = utm_east_driving[i]-utm_east_driving[i-1]
    position = math.sqrt(east**2+north**2)
    time_gap = time_driving_gps[i] - time_driving_gps[i-1]  # Calculate time gap between consecutive samples

    velocity1.append(position / time_gap)

zero_times = [] 

for i in range(len(velocity1)):
    if velocity1[i] <= 0.1:
        zero_times.append(time_driving_gps[i])

print(zero_times)

for i in range(len(zero_times)):
    for j in range(len(time_driving_imu)):
        if int(time_driving_imu[j]) == zero_times[i]:
            accel_x_driving[j] = 0

sos, y4 = butter_filter(accel_x_driving, cutoff_freq, sampl_freq, "lowpass", order)

velocitynew = integrate.cumulative_trapezoid(accel_x_driving, time_driving_imu, initial = 0) 

order = 3 #you can increase this to make the filter "sharper"
sampl_freq = 1 #change to sampling frequency of your data collection
cutoff_freq = 0.05 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

sos, y = butter_filter(velocity1, cutoff_freq, sampl_freq, "lowpass", order)

af.set_xlabel("Time [sec]")
af.set_ylabel("Velocity [m/s]")
af.set_title("Forward Velocity from GPS")
af.grid()
af.plot(time_driving_gps,velocity1)

##################### PLOT 7

#fig, (ag1, ag2, ag3) = plt.subplots(1,3)
fig, (ag1, ag2) = plt.subplots(1,2)

#step 4 part 1
Position_imu = integrate.cumulative_trapezoid(velocityA, time_driving_imu, initial = 0) 
Position_gps = integrate.cumulative_trapezoid(velocity1, time_driving_gps, initial = 0)

ag1.scatter(time_driving_gps,Position_gps,label='GPS')
ag1.scatter(time_driving_imu,Position_imu,label='imu')
ag1.set_xlabel("Time [sec]")
ag1.set_ylabel("Distance [m]")
ag1.set_title("Distance as recorded by IMU and GPS")
ag1.legend(loc=1)
ag1.grid()

#step 4 part 2
x_prime = velocityB
x_prime_w = x_prime*gyro_z_driving
    ########## The two datas are surprisingly similar, however the accel_y_observed "y double prime" is more noisy thus I will pass it through a lowpass filter and plot again, I will also divide it by -1

fig, (aj1) = plt.subplots(1)
aj1.plot(time_driving_imu,x_prime_w, label='wx_prime')
#aj1.scatter(time_driving_imu,accel_y_driving,s=1, label='y_doubleprime_obs')

order = 5 #you can increase this to make the filter "sharper"
sampl_freq = 40 #change to sampling frequency of your data collection
cutoff_freq = 0.75 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

sos, y = butter_filter(accel_y_driving, cutoff_freq, sampl_freq, "lowpass", order)
aj1.plot(time_driving_imu,-y,label='filtered y_observed')

aj1.legend(loc=1)
aj1.grid()
##################################### NOW the two data sets look reasonably similar

ag2.scatter(utm_east_driving,utm_north_driving,s=2, label='gps')

ag2.scatter(utm_east_driving[0],utm_north_driving[0], color = 'red',s=100)
ag2.set_xlabel("Offset UTM Easting [m]")
ag2.set_ylabel("Offset UTM Northing [m]")
ag2.set_title("Estimated trajectory from GPS")
ag2.grid()
ag2.legend(loc=1)

########### Now use the heading from the magnetometer to rotate the velocity(velocityA), denote this vector by v_east,v_north, then integrate using the trapezoid function to get position trajectory

v_e = velocityA*np.cos(filtered_magnetometer)
v_n = velocityA*np.sin(filtered_magnetometer)

p_e = integrate.cumulative_trapezoid(v_e, time_driving_imu, initial = 0)
p_n = integrate.cumulative_trapezoid(v_n, time_driving_imu, initial = 0)

############ show before and after the below editing, I flipped it and only used data points 0:11500 and scaled the imu data down by 0.85
ag2.scatter(-0.85*p_e[0:11500],-0.85*p_n[0:11500],s=2, label='imu')
ag2.scatter(p_e[0],p_n[0], color = 'red',s=100)
ag2.legend(loc=1)
#ag3.set_xlabel("Offset UTM Easting [m]")
#ag3.set_ylabel("Offset UTM Northing [m]")
#ag3.set_title("Estimated trajectory from IMU")
#ag3.grid()

############################ Quaternion to euler angle
fig, (ah) = plt.subplots(1, 1)

for i in range(0, len(qx_circle)):

    roll_qc   = np.arctan2(2*(qw_circle[i]*qx_circle[i]+qy_circle[i]*qz_circle[i]),1-2*(qx_circle[i]**2+qy_circle[i]**2))
    pitch_qc = -np.pi/2 +2*np.arctan2(np.sqrt(1+2*(qw_circle[i]*qy_circle[i]-qx_circle[i]*qz_circle[i])),np.sqrt(1-2*(qw_circle[i]*qy_circle[i]-qx_circle[i]*qz_circle[i])))
    yaw_qc  = np.arctan2(2*(qw_circle[i]*qz_circle[i]+qx_circle[i]*qy_circle[i]),1-2*(qy_circle[i]**2+qz_circle[i]**2))

    roll_q_circle.append(roll_qc)
    pitch_q_circle.append(pitch_qc)
    yaw_q_circle.append(yaw_qc)
    

for i in range(0, len(qx_driving)):
    roll_qd   = np.arctan2(2*(qw_driving[i]*qx_driving[i]+qy_driving[i]*qz_driving[i]),1-2*(qx_driving[i]**2+qy_driving[i]**2))
    pitch_qd = -np.pi/2 +2*np.arctan2(np.sqrt(1+2*(qw_driving[i]*qy_driving[i]-qx_driving[i]*qz_driving[i])),np.sqrt(1-2*(qw_driving[i]*qy_driving[i]-qx_driving[i]*qz_driving[i])))
    yaw_qd = np.arctan2(2*(qw_driving[i]*qz_driving[i]+qx_driving[i]*qy_driving[i]),1-2*(qy_driving[i]**2+qz_driving[i]**2))

    roll_q_driving.append(roll_qd)
    pitch_q_driving.append(pitch_qd)
    yaw_q_driving.append(yaw_qd)

yaw_q_driving = np.array(yaw_q_driving)
yaw_q_driving = np.mod(yaw_q_driving + np.pi, 2 * np.pi) - np.pi
ah.plot(time_driving_imu,yaw_q_driving)

ah.set_xlabel("Time [sec]")
ah.set_ylabel("Heading [Radians]")
ah.set_title("RAW IMU Yaw-Angle")
ah.grid()

plt.show()