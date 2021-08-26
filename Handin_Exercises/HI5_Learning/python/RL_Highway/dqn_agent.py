import torch
import random
import datetime
import os
import json
from transition_memory import TransitionMemory
from torchinfo import summary


# %%
class DQN_Agent:
    """A simplistic Deep Q Learning agent"""
    def __init__(self, env, Q, opts):
        """Initialize Deep Q Learning agent

        arguments:
            env -- Open AI Gym environment
            Q -- Torch class (not object) for functional approximation neural network
            opts -- Options dictionary with keys
                explore_exploit -- Function explore_exploit(k) returning probability for random action during episode k
                gamma -- discount factor
                batch_size -- size of batches used in training [default: 32]
                lr -- learning rate for Adam optimizer [default: 5e-4]
                target_update -- Update rate for target network (number of batches, not episodes) [default: 40]
                double_DQN -- DQN or Double DQN [default: False]
                memory_size -- Size of (s,a,r,s)-memory buffer [default: 100000]
                
                
        The implementation is based on https://github.com/eleurent/rl-agents but simplified for educational purposes.
        """
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.env = env

        self._Q = Q().to(self.device)
        self._Q_target = Q().to(self.device)
        self._Q_target.load_state_dict(self._Q.state_dict())
        self._Q_target.eval()

        self.config = self.default_config()
        for k in opts:
            self.config[k] = opts[k]

        self.train_mode = True
        self.model_name_str = None

        self.loss_fun = torch.nn.MSELoss()
        self.optimizer = torch.optim.Adam(params=self._Q.parameters(), lr=self.config["lr"], weight_decay=0)

        self.memory = TransitionMemory(capacity=self.config["memory_size"], device=self.device)
        self._batch_iteration = 0

        self.explore_exploit = self.config["explore_exploit"]

        self.stats = {'episode_rewards': [],
                      'episode_rewards_': [],
                      'eps': []}

    def default_config(self):
        return {
            "batch_size": 32,  # batch size
            "lr": 5e-4,  # Learning rate
            "target_update": 40,  # Update rate for target network (number of batches, not episodes)
            "double_DQN": False,  # DQN or Double DQN
            "memory_size": 100000  # Size of (s,a,r,s)-memory buffer
        }

    def act(self, s):
        """Return action for state s

        Returns arg max_a Q(s, a)
        """
        _, actions = self._Q(torch.tensor([s], dtype=torch.float).to(self.device)).max(1)
        return actions.data.cpu().numpy()[0]

    def Q(self, s):
        """Get state value function for state s

        Returns Q(s,a) for all a
        """
        return self._Q(torch.tensor([s], dtype=torch.float).to(self.device)).data.cpu().numpy()

    def epsilon_greedy(self, s, k):
        """Return epsilon-greedy exploration/explotation action

        arguments:
            s -- state
            k -- episode number

        returns random or greedy action based on property explore_exploit
        """
        if random.uniform(0, 1) < self.explore_exploit(k):
            return self.env.action_space.sample()
        else:
            return self.act(s)

    def update(self, s, a, r, s_next, done):
        """Update model based on (s, a, r, s)-tuple

        1. store training statistics
        2. push transition to memory
        3. random sample a batch
        4. train neural net on batch

        argument:
            s -- current state
            a -- current action
            r -- reward
            s_next -- next state
            done -- terminal
        """
        self.stats['episode_rewards_'][-1].append(r)

        self.memory.push((s, a, r, s_next, done))  # Push (s_t, a_t, r_t+1, s_t+1)-tuple to memory
        batch = self.memory.sample(self.config["batch_size"])  # Get a random batch of tuples for training

        if batch:
            # Compute current Q(s_t, a_t) for entire batch
            Q_sa = self._Q(batch.state)
            # Efficient way to extract value for the specific actions in batch
            Q_sa = Q_sa.gather(1, batch.action.unsqueeze(1)).squeeze(1)

            with torch.no_grad():  # No need for gradients for target Q
                # Compute Q_sa_prim = max_a Q(s_t+1, a) for entire batch (set to 0 for terminal states)
                Q_sa_prim = torch.zeros(batch.reward.shape).to(self.device)  # Set all to 0
                if self.config["double_DQN"]:  # Double DQN
                    # Pick greedy action from policy network
                    _, best_actions = self._Q(batch.next_state).max(1)
                    # Estimate Q from target network
                    max_values = self._Q_target(batch.next_state).gather(1, best_actions.unsqueeze(1)).squeeze(1)
                else:
                    # DQN: Estimate Q and action from target network
                    max_values, _ = self._Q_target(batch.next_state).max(1)

                Q_sa_prim[~batch.terminal] = max_values[~batch.terminal]  # Only update non-terminal states

                # Compute the new predicted Q values Q_pred(s_t, a_t)
                Q_sa_pred = batch.reward + self.config["gamma"] * Q_sa_prim

            loss = self.loss_fun(Q_sa, Q_sa_pred)  # Loss: MSE(Q(s_t, a_t) - Q_pred(s_t, a_t))

            # Update the model weights
            self.optimizer.zero_grad()
            loss.backward()
            for param in self._Q.parameters():
                param.grad.data.clamp_(-1, 1)
            self.optimizer.step()
            self._batch_iteration += 1

            if self._batch_iteration % self.config["target_update"] == 0:
                self._Q_target.load_state_dict(self._Q.state_dict())

    def train(self, num_episodes, verbose=True, display=False):
        """Train DQN model

        argument:
            num_episodes -- number of episodes to train
            verbose -- print out training information
            display -- show graphical environment during training
        """
        for k in range(num_episodes):
            if verbose:
                print(f"Episode {k + 1} (p={self.explore_exploit(k):.3f}) ... ", end="")
            self.stats['episode_rewards_'].append([])  # Initialize empty list for episode statistics

            s = self.env.reset()
            done = False
            tot_reward = 0
            while not done:
                a = self.epsilon_greedy(s, k)  # Choose training action

                s_next, r, done, _ = self.env.step(a)  # Act on the environment and record next state and reward
                tot_reward += r
                if display:
                    self.env.render()  # Update environment graphics

                self.update(s, a, r, s_next, done)  # Add (s_t, a_t, r_t+1, s_t+1) to training set and update model

                s = s_next

            self.stats['episode_rewards'].append(tot_reward)
            self.stats['eps'].append(self.explore_exploit(k))
            if verbose:
                print(f"total reward {tot_reward:.1f}")

    def summary(self):
        return summary(self._Q)

    def eval(self, mode=True):
        """Put DQN agent in evaluation mode

        Put DQN agent in evaluation mode. This turns off all exploration/exploitation
        or random parts of the model. This should be done when evaluating the model.
        """
        self._Q.train(not mode)
        self.train_mode = not mode

    def _model_dir(self):
        t = datetime.datetime.now()
        self.model_name_str = f"{t.year}_{t.month:02d}_{t.day:02d}-{t.hour:02d}_{t.minute:02d}_{t.second:02d}"
        model_dir = os.path.join("models", self.model_name_str)
        os.makedirs(model_dir)
        return model_dir

    def save(self):
        """Save model and statistics in directory models/[run_identifier]"""
        model_dir = self._model_dir()

        model_filename = os.path.join(model_dir, "last_model.tar")
        state = {'state_dict': self._Q.state_dict(),
                 'optimizer': self.optimizer.state_dict()}
        torch.save(state, model_filename)

        if len(self.stats["episode_rewards"]) > 0:
            stats_filename = os.path.join(model_dir, "stats.json")
            with open(stats_filename, "wt") as f:
                json.dump(self.stats, f, indent=4)
        return model_dir

    def load(self, filename):
        """Load saved model

        argument:
            filename -- path to .tar file with model weights
        """
        checkpoint = torch.load(filename, map_location=self.device)
        self._Q.load_state_dict(checkpoint['state_dict'])
        self._Q_target.load_state_dict(self._Q.state_dict())
        self._Q_target.eval()

        self.optimizer.load_state_dict(checkpoint['optimizer'])
        print(f"Loaded model weights from file {filename}")
        stats_filename = os.path.join(os.path.dirname(filename), "stats.json")
        if os.path.exists(stats_filename):
            print(f"Loaded stats from file {stats_filename}")
            with open(stats_filename, "rt") as f:
                self.stats = json.load(f)

        return filename
