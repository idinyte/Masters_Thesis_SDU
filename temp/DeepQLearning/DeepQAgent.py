import numpy as np
import random
from DeepQlearning.rewards import Rewards
from DeepQlearning.stateSpace import Action, StateSpace
from DeepQlearning.DeepQNetwork import DQN
import torch
import torch.optim as optim
import sys

class DeepQlearningAgent(StateSpace):
    state, action = -1, -1
    epsilon, gamma, learning_rate = 0, 0, 0
    replay_memory, max_memory = [], 0
    dqn = None
    device = None
    optimizer = None
    debug = False
    total_reward = 0

    def __init__(self, player_i, gamma, learning_rate, epsilon, max_memory = 50000):
        super().__init__()
        self.epsilon = epsilon
        self.gamma = gamma
        self.rewards = Rewards(len(Action))
        self.player_i = player_i
        self.max_memory = max_memory
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.dqn = DQN(21, 4).to(self.device)
        self.optimizer = optim.Adam(self.dqn.parameters(), lr=1e-4)

    def update(self, obs, players):
        dice, move_pieces, player_pieces, enemy_pieces = obs
        super().update_state_space(players, self.player_i, move_pieces, dice)
        self.state = self.get_state(player_pieces, enemy_pieces, dice)
        self.action = self.select_action(move_pieces)

        return self.action
    
    def get_state(self, player_pieces, enemy_pieces, dice):
        return [*player_pieces, *[piece for pieces in enemy_pieces for piece in pieces.tolist()], dice, *self.action_table_obj.action_table]

    def select_action(self, move_pieces):
        if random.random() < self.epsilon:
            return move_pieces[random.randint(0, len(move_pieces) - 1)]
        else:
            with torch.no_grad():
                state_tensor = torch.tensor([self.state], dtype=torch.float32).to(self.device)
                available_actions_mask_tensor = torch.tensor(self.available_actions_mask(move_pieces), dtype=torch.float32).to(self.device)
                q_values = self.dqn(state_tensor)
                q_values *= available_actions_mask_tensor
                q_values[torch.isnan(q_values)] = -float('inf')
                move = torch.argmax(q_values).item()
                if self.debug:
                    print(f"q values {q_values}")
                    print(f"move {move}")
                
                return move
            
    def save_weights(self, file_path):
        torch.save(self.dqn.state_dict(), file_path)
        
    def available_actions_mask(self, move_pieces):
        available_actions_mask = np.full(4, np.nan)
        available_actions_mask[move_pieces] = 1
        return available_actions_mask
        
    def update_epsilon(self, epsilon, decay_rate, episode):
        self.epsilon = epsilon * np.exp(-decay_rate*episode)

    def get_reward(self):
        action = self.action_table_obj.action_table[self.action] # convert move piece to action that occured
        reward = self.rewards.rewards_table[int(action)]
        if self.debug:
            print(self.action)
            print(f"reward  {reward}")
            self.print_state_action()
        return reward
    
    def train_dqn(self, batch_size, debug):
        if len(self.replay_memory) < batch_size:
            return 500

        self.optimizer.zero_grad()
        self.debug = debug
        transitions = np.random.choice(len(self.replay_memory), batch_size, replace=False)
        batch = [self.replay_memory[idx] for idx in transitions]

        states = torch.tensor([transition['state'] for transition in batch], dtype=torch.float32)
        next_states = torch.tensor([transition['next_state'] for transition in batch], dtype=torch.float32)
        rewards = torch.tensor([transition['reward'] for transition in batch], dtype=torch.float32)
        actions = torch.tensor([transition['action'] for transition in batch], dtype=torch.long)
        dones = torch.tensor([transition['done'] for transition in batch], dtype=torch.float32)

        q_values = self.dqn.forward(states)
        next_q_values = self.dqn.forward(next_states)

        target_q_values = rewards + self.gamma * torch.max(next_q_values, dim=1).values * (1 - dones)

        mask = torch.eye(4)[actions]
        masked_q_values = torch.sum(q_values * mask, dim=1)
        
        loss = torch.nn.functional.smooth_l1_loss(masked_q_values, target_q_values.detach())
        torch.nn.utils.clip_grad_value_(self.dqn.parameters(), 100)
        loss.backward()
        self.optimizer.step()
        return loss.item()
        
    def updateReplayMemory(self, reward, next_state, move_pieces, done):
        replay = {
            'state' : self.state,
            'next_state': next_state,
            'action' : self.action,
            'reward': reward,
            'done': done
        }
        if len(self.replay_memory) < self.max_memory:
            self.replay_memory.append(replay)
        else:
            self.replay_memory[np.random.randint(self.max_memory)] = replay

    def print_state_action(self):
        action_str = ""
        match self.action_table_obj.action_table[self.action]:
            case Action.SAFE_MoveOut.value:
                action_str = "SAFE_MoveOut"
            case Action.SAFE_KillMoveOut.value:
                action_str = "SAFE_KillMoveOut"
            case Action.SAFE_Dice.value:
                action_str = "SAFE_Dice"
            case Action.SAFE_Star.value:
                action_str = "SAFE_Star"
            case Action.SAFE_Globe.value:
                action_str = "SAFE_Globe"
            case Action.SAFE_DoubleUp.value:
                action_str = "SAFE_DoubleUp"    
            case Action.SAFE_Kill.value:
                action_str = "SAFE_Kill"    
            case Action.SAFE_KillStar.value:
                action_str = "SAFE_KillStar"    
            case Action.SAFE_Die.value:
                action_str = "SAFE_Die"    
            case Action.SAFE_GoalZone.value:
                action_str = "SAFE_GoalZone"    
            case Action.SAFE_Finish.value:
                action_str = "SAFE_Finish"    
            case Action.UNSAFE_Dice.value:
                action_str = "UNSAFE_Dice"    
            case Action.UNSAFE_Star.value:
                action_str = "UNSAFE_Star"    
            case Action.UNSAFE_Globe.value:
                action_str = "UNSAFE_Globe"
            case Action.UNSAFE_DoubleUp.value:
                action_str = "UNSAFE_DoubleUp"   
            case Action.UNSAFE_Kill.value:
                action_str = "UNSAFE_Kill"   
            case Action.UNSAFE_KillStar.value:
                action_str = "UNSAFE_KillStar"   
            case Action.UNSAFE_Die.value:
                action_str = "UNSAFE_Die"   
            case Action.UNSAFE_GoalZone.value:
                action_str = "UNSAFE_GoalZone"   
            case Action.UNSAFE_Finish.value:
                action_str = "UNSAFE_Finish"
        print(f"Action: {action_str}")
        return action_str