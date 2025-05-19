from XRPLib.board import Board
from XRPLib.differential_drive import DifferentialDrive
from XRPLib.rangefinder import Rangefinder
from Husky.huskylensPythonLibrary import HuskyLensLibrary
import time
import random

# Initialize systems
board = Board.get_default_board()
husky = HuskyLensLibrary("I2C")
differentialDrive = DifferentialDrive.get_default_differential_drive()
rangefinder = Rangefinder.get_default_rangefinder()

# Configuration constants
base_velocity = 20
delta_velocity = 20
min_velocity = 3
max_velocity = 10
upper_thresholdR = 0.85

class BehaviorBasedRobot:
    def __init__(self):
        self.robot_state = "walk"
        
        # Ensure HuskyLens is in line tracking mode
        while not husky.line_tracking_mode():
            husky.line_tracking_mode()

    def get_sensor_input(self):
        # Get line tracking state from HuskyLens
        state = husky.command_request_arrows()
        wall_distance = rangefinder.distance()

        # State transition logic with improved wall detection
        if self.robot_state == "walk":
            if len(state) > 0:
                self.robot_state = "line"
            elif wall_distance < 30:
                self.robot_state = "wall"
            else:
                self.robot_state = "walk"

        elif self.robot_state == "line":
            if wall_distance < 30:
                self.robot_state = "wall"
            elif len(state) == 0:
                self.robot_state = "walk"
            else:
                self.robot_state = "line"

        elif self.robot_state == "wall":
            # Only transition from wall following if no wall is detected AND no line is detected
            if len(state) > 0:
                self.robot_state = "line"
            elif wall_distance >= 30:
                self.robot_state = "walk"
            else:
                # Keep wall following if wall is still detected
                self.robot_state = "wall"

        return self.robot_state

    def random_walk(self):
        directions = ["forward", "left", "right"]
        chosen_direction = random.choice(directions)
        
        if chosen_direction == "forward":
            differentialDrive.set_speed(max_velocity, max_velocity)
            time.sleep(2)    
        elif chosen_direction == "left": 
            differentialDrive.set_speed(min_velocity, max_velocity) 
            time.sleep(1)  
        elif chosen_direction == "right": 
            differentialDrive.set_speed(max_velocity, min_velocity) 
            time.sleep(1)  
            
        # Stop after movement
        differentialDrive.set_speed(0, 0)
        print(f"Robot is wandering {chosen_direction}.")

    def line_following(self):
        print("Robot is line following.")
        state = husky.command_request_arrows()

        if (len(state) > 0): 
            state_vector = state[0]

            # x1 and x2 are the left and right points of the arrow
            state_x1 = state_vector[0]  # x1
            state_x2 = state_vector[2]  # x2

            x = (state_x1 + state_x2) / 2  # center position of the line
            error = 160 - x
            kp1=0.1
            delta_velocity = abs(kp1*error)

            if delta_velocity > 15 : 
                delta_velocity=15

            if x < 140:
                # Turn left
                differentialDrive.set_speed(base_velocity-delta_velocity, base_velocity+delta_velocity)  # left motor slower

            elif x > 180:
                # Turn right
                differentialDrive.set_speed(base_velocity+delta_velocity, base_velocity-delta_velocity)  # right motor slower
            else:
                differentialDrive.set_speed(base_velocity, base_velocity)  # both motors at 60% speed


        time.sleep(0.1)

    def wall_following(self):
        kP = 0.02
        targetDist = 20

        # Compute error and adjust speeds
        current_distance = rangefinder.distance()
        speed_adjustment = kP * (current_distance - targetDist)
        
        # Apply wall following speeds
        left_speed = 0.4 + speed_adjustment
        right_speed = 0.4 - speed_adjustment
        
        differentialDrive.set_effort(left_speed, right_speed)
        print("Robot is wall following.")
        

    def run(self):
        try:
            while True:
                # Update sensor inputs and get current state
                self.get_sensor_input()
                
                # Execute behavior based on current state
                if self.robot_state == "line":
                    self.line_following()
                elif self.robot_state == "wall":
                    self.wall_following()
                else:
                    self.random_walk()
        
        except KeyboardInterrupt:
            print("Robot operation stopped.")
            differentialDrive.set_speed(0, 0)

# Initialize and run the robot
if __name__ == "__main__":
    robot = BehaviorBasedRobot()
    robot.run()
