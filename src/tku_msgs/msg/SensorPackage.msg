std_msgs/MultiArrayLayout layout
float64[] imudata
int32[] forcesensordata
float64 yaw
float64 pitch
float64 roll

# 四元數 (相對於歸零姿態)
float64 quat_w
float64 quat_x
float64 quat_y
float64 quat_z

# 加速度 (g,感測器座標)
float64 acc_x
float64 acc_y
float64 acc_z

# 角速度 (deg/s,感測器座標)
float64 gyr_x
float64 gyr_y
float64 gyr_z

# 磁力 (uT,感測器座標)
float64 mag_x
float64 mag_y
float64 mag_z

# 晶片溫度 (度C)
float64 temperature
