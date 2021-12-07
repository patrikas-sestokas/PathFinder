"""navigator controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import math
from controller import MouseState


class Tile:
    x: int
    y: int
    tile_size = 0.25

    def __init__(self, robot_pos):
        self.x = robot_pos[0] // self.tile_size
        self.y = robot_pos[2] // self.tile_size

    def __eq__(self, other):
        if not isinstance(other, Tile):
            return False
        return other.x == self.x and other.y == self.y

    def __repr__(self):
        return f'{{x: {self.x}, y: {self.y}}}'

    def as_coordinates(self):
        return self.x * self.tile_size + self.tile_size / 2, self.y * self.tile_size + self.tile_size / 2


def setup_sensors(robot, time_step):
    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)

    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    gps = robot.getDevice('gps')
    gps.enable(1)

    pen = robot.getDevice('pen')
    pen.setInkColor(0x00FF00, 1)
    pen.write(True)

    mouse = robot.getMouse()
    mouse.enable(100)

    proximity_sensors = [robot.getDevice(f'ps{idx}') for idx in range(8)]
    for sensor in proximity_sensors:
        sensor.enable(time_step)
    return [left_motor, right_motor, gps, pen, mouse, proximity_sensors]


def is_finish_line(robot_vec, finish_vec):
    return finish_vec[0] + 0.1 > robot_vec[0] > finish_vec[0] - 0.1 and finish_vec[2] + 0.1 > robot_vec[2] > \
           finish_vec[2] - 0.1


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


def run_robot(robot):
    timeStep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    left_motor, right_motor, gps, pen, mouse, proximity_sensors = setup_sensors(robot, timeStep)

    finish_line_node = robot.getFromDef("finish_line")
    finish_line_translation = finish_line_node.getField("translation")
    finish_line_vector = finish_line_translation.getSFVec3f()

    path = []

    mode = 1

    # Main loop:
    while robot.step(timeStep) != -1:
        robot_pos = gps.getValues()
        if is_finish_line(robot_pos, finish_line_vector):
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            mode = 2

        tile = Tile(robot_pos)

        if mode == 1:
            if len(path) == 0 or path[-1] != tile:
                if tile in path:
                    path = path[:path.index(tile) + 1]
                else:
                    path.append(tile)
                #print(path)
            left_speed, right_speed = mode_1(proximity_sensors, max_speed)

        elif mode == 2:
            print("LABYRINTH SOLVED!")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break

        else:
            raise Exception(f'Unknown mode {mode}')

        point = (mouse.getState().x, mouse.getState().z)
        if not math.isnan(point[0]):
            dx = point[0] - path[0].as_coordinates()[0]
            dz = point[1] - path[0].as_coordinates()[1]
            print(math.degrees(math.atan2(dz, dx)))

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)


if __name__ == "__main__":
    my_robot = Supervisor()
    run_robot(my_robot)
