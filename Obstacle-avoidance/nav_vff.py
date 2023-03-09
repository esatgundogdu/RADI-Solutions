from GUI import GUI
from HAL import HAL
import math

def parse_laser_data (laser_data):
    laser = []
    i = 0
    while (i < 180):
        dist = laser_data.values[i]
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
        i+=1
    return laser


def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return [x_rel, y_rel]
    
def target_reached(currentTarget):
    x = HAL.getPose3d().x
    y = HAL.getPose3d().y
    target = currentTarget.getPose()
    
    dist = math.sqrt((x-target.x)**2 + (y-target.y)**2)
    if dist < 3:
        return True
    
    return False

def get_target_vector(currentTarget):
    robot_pose = HAL.getPose3d()
    x = HAL.getPose3d().x
    y = HAL.getPose3d().y
    yaw = HAL.getPose3d().yaw
    
    targetPose = currentTarget.getPose()
    
    return absolute2relative(targetPose.x, targetPose.y, x, y, yaw)

def get_obstacle_vector(data):
    result = [0, 0]
    for i in range(180):
        dist = data[i][0]
        angle = data[i][1]
        
        # since this is repulsive vector, multiply with -1
        y = -(math.sin(angle) / (dist))
        result[1] += y
    return result

LIMIT = 2
def get_control(directionVector):
    x = directionVector[0]
    y = directionVector[1]
    
    # its already relative to robot
    yaw = math.atan2(y, x)
    
    w = (yaw**2)
    
    print(yaw)
    
    if yaw < 0:
        w = -w
    
    if w > LIMIT:
        w = LIMIT
    elif w < -LIMIT:
        w = -LIMIT
    
    return w

def main():
    # get current target
    currentTarget = GUI.map.getNextTarget()
    
    # check if we have reached currentTarget
    if target_reached(currentTarget):
        currentTarget.setReached(True)
        currentTarget = GUI.map.getNextTarget()
    
    # get target vector
    targetVector = get_target_vector(currentTarget)
    
    GUI.showLocalTarget([currentTarget.getPose().x, currentTarget.getPose().y])
    
    # get laser data
    laser_data = parse_laser_data(HAL.getLaserData())
    
    # get and set obstacle vector
    obstacleVector = get_obstacle_vector(laser_data)
    
    # optimize target and obstacle vector first
    targetVector = [x / 4 for x in targetVector]
    
    # get and set direction vector
    directionVector = [targetVector[0], targetVector[1] + obstacleVector[1]]
    GUI.map.avgx = directionVector[0]
    GUI.map.avgy = directionVector[1]

    # Car direction
    GUI.map.carx = targetVector[0]
    GUI.map.cary = targetVector[1]
    
    GUI.map.obsx = obstacleVector[0]
    GUI.map.obsy = obstacleVector[1]
    
    w = get_control(directionVector)
    
    HAL.setV(2)
    HAL.setW(w)

while True:
    main()
