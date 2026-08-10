std_msgs/Header header

# 目标人在里程计坐标系中的位置 (m)
float32[3] pos

# 目标人在里程计坐标系中的速度 (m/s)
float32[3] vel

# 位置不确定度 (std)，单位 m
float32[3] pos_noise

# 速度不确定度 (std)，单位 m/s
float32[3] vel_noise

# 目标人到机器人的直线距离 (m)
float32 dis

# 水平方位角 (rad)，0=正前方，左正右负
float32 azimuth

# 仰角 (rad)，水平=0，上正下负
float32 elevation
