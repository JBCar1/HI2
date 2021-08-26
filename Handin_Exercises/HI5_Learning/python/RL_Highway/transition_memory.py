import torch
from collections import namedtuple
import random

Transition = namedtuple('Transition', ('state', 'action', 'reward', 'next_state', 'terminal'))


class TransitionMemory:
    """Memory for state transitions (state, action, reward, next_state, terminal)"""
    def __init__(self, capacity=10000, seed=None, device="cpu"):
        self.capacity = capacity
        self.position = 0
        self.memory = []
        self.device = device
        random.seed(seed)

    def push(self, transition):
        """Push transition onto memory

        arguments:
        transition -- Tuple (state, action, reward, next_state, terminal)
        """
        if len(self.memory) == self.capacity:
            self.memory = self.memory[-(self.capacity - 1):]
        self.memory.append(Transition(*transition))

    def sample(self, batch_size):
        """Sample a random batch from memory

        Returns a sample batch of batch_size from memory. Returns None if not enough data is available in memory.
        """
        if len(self.memory) < batch_size:
            return None
        else:
            # return [self.memory[idx] for idx in self.rg.choice(len(self.memory), batch_size)]
            batch = Transition(*zip(*random.sample(self.memory, batch_size)))
            state = torch.cat(tuple(torch.tensor([batch.state], dtype=torch.float))).to(self.device)
            action = torch.tensor(batch.action, dtype=torch.long).to(self.device)
            reward = torch.tensor(batch.reward, dtype=torch.float).to(self.device)
            next_state = torch.cat(tuple(torch.tensor([batch.next_state], dtype=torch.float))).to(self.device)
            terminal = torch.tensor(batch.terminal, dtype=torch.bool).to(self.device)
            batch_tensor = Transition(state, action, reward, next_state, terminal)

            return batch_tensor

    def __len__(self):
        "Size of memory"
        return len(self.memory)
