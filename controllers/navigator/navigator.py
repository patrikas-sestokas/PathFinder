from controller import Supervisor, MouseState, Pen
import numpy


'''
Class for converting arbitrary coordinates to specific tile
'''
class Tile:
    x: int
    y: int
    tile_size = 0.25

    '''
    Calculates which tile the robot's position refers to
    @param robot_pos: Robot's current position in x, y, z
    '''

    def __init__(self, robot_pos):
        self.x = robot_pos[0] // self.tile_size
        self.y = robot_pos[2] // self.tile_size

    '''
    Compares tile objects and checks if they are equal
    @param other: The other tile
    @return: True or False
    '''

    def __eq__(self, other):
        if not isinstance(other, Tile):
            return False
        return other.x == self.x and other.y == self.y

    '''
    Display tile coordinates
    @return: Coordinates in text
    '''

    def __repr__(self):
        return f'{{x: {self.x}, y: {self.y}}}'

    '''
    Converts tile coordinates to the original coordinates
    @return: Converted x and y coordinates
    '''

    def as_coordinates(self):
        return self.x * self.tile_size + self.tile_size / 2, self.y * self.tile_size + self.tile_size / 2


'''
Setups robot's sensors: sets initial motor position and velocity, enables gps,
enables robot's inertial unit sensor and setups pen for drawing on mode 2
@robot: Current robot the controller is operates
@time_step: Increment executed at each iteration of the control loop
@return: Setuped sensors
'''


def setup_sensors(robot, time_step):
    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)

    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    gps = robot.getDevice('gps')
    gps.enable(1)

    mouse = robot.getMouse()
    mouse.enable(250)
    mouse.enable3dPosition()

    inertial_unit = robot.getDevice('inertial unit')
    inertial_unit.enable(1)

    pen = robot.getDevice('pen')
    pen.write(True)
    pen.setInkColor(0xFF0000, 1)

    proximity_sensors = [robot.getDevice(f'ps{idx}') for idx in range(8)]
    for sensor in proximity_sensors:
        sensor.enable(time_step)
    return [left_motor, right_motor, gps, mouse, inertial_unit, proximity_sensors, pen]


'''
Method checks if the robot reached the finish line
@robot_vec: Current robot position in x, y, z
@finish_vec: Finish line position in x, y, z
@return: Boolean value which determines whether the robot is on the finish line
'''
def is_finish_line(robot_vec, finish_vec):
    return finish_vec[0] + 0.1 > robot_vec[0] > finish_vec[0] - 0.1 and finish_vec[2] + 0.1 > robot_vec[2] > \
           finish_vec[2] - 0.1


'''
Method operates the robot through the maze using wall follower algorithm:
gets the front, left and left corner proximity sensor distances and checks
whether they are greater than 80. If the robot's front sensor detects an object
then the robot turns right. If there's no object in front of the robot then it
drives near the left wall keeping its distance and turning right if the robot gets
too close to the wall
@proximity_sensors: Robot's proximity sensor data
@max_speed: Robot's max speed
@return: New motor speed values
'''
def mode_1(proximity_sensors, max_speed):
    left_wall = proximity_sensors[5].getValue() > 80
    left_corner = proximity_sensors[6].getValue() > 80
    front_wall = proximity_sensors[7].getValue() > 80

    left_speed = max_speed
    right_speed = max_speed

    if front_wall:
        # print("Turn right")
        left_speed = max_speed
        right_speed = -max_speed
    else:
        if left_wall:
            # print("Drive forward")
            left_speed = max_speed
            right_speed = max_speed
        else:
            # print("Turn left")
            left_speed = max_speed / 8
            right_speed = max_speed
        if left_corner:
            # print("Too close, right")
            left_speed = max_speed
            right_speed = max_speed / 8

    return left_speed, right_speed


'''
Calculates the angle vector needed to reach tile's destination
@angle: Relative turn angle to the tile from robot's current facing point
@max_speed: Robot's motor max_speed
@return: Angle vector for the next tile
'''
def mode_2(angle, max_speed):
    # necessary offset so that "pi" and "-pi" angles would result in (0.75pi, 0.75pi) wheel rotation ratios
    offset = -0.75 * numpy.pi
    angle_vector = numpy.array([numpy.sin(angle + offset), numpy.cos(angle + offset)])
    # rescaling unit direction vector to actual vehicle speed
    angle_vector *= max_speed
    return angle_vector


'''
Gets the distance between the robot and the tile
@robot_pos: Current robot position in x, y, z
@tile_pos: Tile position in x, y, z
'''
def get_distance(robot_pos, tile_pos):
    A_B = [tile_pos[0] - robot_pos[0], tile_pos[1] - robot_pos[2]]
    return numpy.linalg.norm(A_B)


'''
Gets the relative angle which is needed to turn the robot to the tile's direction 
@robot_pos: Current robot position in x, y, z
@yaw: Angle between a robot's direction of motion and the relative tile vector.
@tile_pos: Tile position in x, y, z
@distance: Distance between the robot and the tile
@return: Turn angle from robot's direction to the tile
'''
def get_angle(robot_pos, yaw, tile_pos, distance):
    A_B = [tile_pos[0] - robot_pos[0], tile_pos[1] - robot_pos[2]]
    A_F = [numpy.sin(yaw), numpy.cos(yaw)]
    A_B = A_B / distance
    dot = numpy.dot(A_B, A_F)
    det = numpy.linalg.det([A_B, A_F])
    return numpy.arctan2(det, dot)


'''
Method setups robot's sensors and runs it in a loop getting it's current position
and managing robot's modes depending on the scenario. Initially robot operates
on the first mode using the wall following algorithm and constructing the shortest path. 
On reaching the maze exit it switches to the second mode driving back to it's initial
position from the constructed path list. Upon reaching the initial position on the second
mode, the robot stops and prints out a message
@robot: Instance of a robot object
'''
def run_robot(robot):
    timeStep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    left_motor, right_motor, gps, mouse, inertial_unit, proximity_sensors, pen = setup_sensors(robot, timeStep)

    finish_line_node = robot.getFromDef("finish_line")
    finish_line_translation = finish_line_node.getField("translation")
    finish_line_vector = finish_line_translation.getSFVec3f()

    path = []
    mode = 1
    distance = float('inf')

    # Main loop:
    while robot.step(timeStep) != -1:
        robot_pos = gps.getValues()
        if mode == 1 and is_finish_line(robot_pos, finish_line_vector):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            path = path[:-1]
            print(f'First phase time: {robot.getTime()}s')
            firstModeTime = robot.getTime()
            mode = 2
            pen.setInkColor(0x00FF00, 1)
            
        if mode == 1:
            tile = Tile(robot_pos)
            if len(path) == 0 or path[-1] != tile:
                if tile in path:
                    path = path[:path.index(tile) + 1]
                else:
                    path.append(tile)
            left_speed, right_speed = mode_1(proximity_sensors, max_speed)

        elif mode == 2:
            pen.write(True)
            yaw = inertial_unit.getRollPitchYaw()[2]
            new_distance = get_distance(robot_pos, path[-1].as_coordinates())
            # 0.125 path smoothness modifier, should not exceed tile_size / 2, lets the robot cut corners
            if new_distance < 0.125:
                path = path[:-1]
                if len(path) == 0:
                    print(f'Second phase time: {robot.getTime() - firstModeTime}s')
                    print("Exploration finished!")
                    left_motor.setVelocity(0)
                    right_motor.setVelocity(0)
                    break
                distance = get_distance(robot_pos, path[-1].as_coordinates())
            else:
                distance = new_distance
            angle = get_angle(robot_pos, yaw, path[-1].as_coordinates(), distance)
            left_speed, right_speed = mode_2(angle, max_speed)
        else:
            raise Exception(f'Unknown mode {mode}!')

        # point = (mouse.getState().x, mouse.getState().z)
        # if not numpy.isnan(point[0]):
        #     yaw = inertial_unit.getRollPitchYaw()[2]
        #     print(get_angle(robot_pos, yaw, point))

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    my_robot = Supervisor()
    run_robot(my_robot)
