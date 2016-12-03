# This message shows about dobot arm position.
# Not only control dobot to move, but can publish this current pose from dobot
# 

# control = 0        : ( mode, x, y, z, r ) in the below is the current pose of dobot
# control = 1        : move to zero position command, below values don't used
int8 control

# mode = 0          : ( x, y, z, r ) is the relative coordinate, which is based on current position
# mode = 1          : ( x, y, z, r ) is the dobot_link Cartesian coordinate, which is based on dobot base point
# mode = 2          : ( radius, theta, z, r ) is the dobot_link Polar coordinate(2-D), is based on dobot base point
int8 mode

# coordinate
float32 x
float32 y
float32 z
float32 r
float32 radius
float32 theta
