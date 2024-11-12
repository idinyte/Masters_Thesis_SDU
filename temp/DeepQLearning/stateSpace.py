from enum import Enum

class Action(Enum):
    Pick_Ball_1 = 0
    Pick_Ball_2 = 1
    Squeeze = 2
    Unsqueeze = 3
    Release_ball = 4
    Hower_above_box_1 = 5
    Hower_above_box_2 = 6

CAN_PERFORM = 1
CANT_PERFORM = -1

class StateSpace():
    
    def reset_action_table(self):
        self.action_table = [CANT_PERFORM for _ in range(7)]

    def get_available_actions(self, is_robot_holding_ball, is_squeezing, reset_action_table = True):
        self.reset_action_table()
            
        self.action_table[int(Action.Pick_Ball_1)] = CANT_PERFORM if is_robot_holding_ball else CAN_PERFORM
        self.action_table[int(Action.Pick_Ball_2)] = CANT_PERFORM if is_robot_holding_ball else CAN_PERFORM
        self.action_table[int(Action.Squeeze)] = CANT_PERFORM if is_squeezing or not is_robot_holding_ball else CAN_PERFORM
        self.action_table[int(Action.Unsqueeze)] = CAN_PERFORM if is_squeezing and is_robot_holding_ball else CANT_PERFORM
        self.action_table[int(Action.Release_ball)] = CAN_PERFORM if is_robot_holding_ball else CANT_PERFORM
        self.action_table[int(Action.Hower_above_box_1)] = CAN_PERFORM
        self.action_table[int(Action.Hower_above_box_2)] = CAN_PERFORM
        
        return self.action_table