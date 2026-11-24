"""Read-only checkpoint follower and isolated evaluation state for the desktop viewer."""
from dataclasses import dataclass
import json
from pathlib import Path
import zipfile
import numpy as np
from .env import RoboEnv

@dataclass(frozen=True)
class Checkpoint:
    path: Path
    steps: int
    modified: int
    size: int


def latest_checkpoint(run):
    """Only follow the atomically published manifest, never pending ZIPs or directory scans."""
    run = Path(run).resolve()
    manifest = json.loads((run/'latest.json').read_text())
    path = Path(manifest['model'])
    if not path.is_absolute(): path = run/path
    path = path.resolve()
    if not path.is_relative_to(run/'checkpoints') or path.suffix != '.zip':
        raise ValueError('Checkpoint fora da pasta checkpoints deste treino')
    stat = path.stat()
    steps = int(manifest['steps'])
    if steps < 0: raise ValueError('Contagem negativa de passos')
    return Checkpoint(path, steps, stat.st_mtime_ns, stat.st_size)


def load_policy(checkpoint):
    """Validate the complete ZIP, load on CPU and freeze all parameters. Never save or learn."""
    import torch
    from stable_baselines3 import PPO
    with zipfile.ZipFile(checkpoint.path) as archive:
        if archive.testzip() is not None: raise ValueError('CRC inválido no checkpoint')
    model = PPO.load(checkpoint.path, device='cpu')
    if int(model.num_timesteps) != checkpoint.steps:
        raise ValueError('Checkpoint não corresponde aos passos do manifest')
    if model.action_space.shape != (3,) or model.observation_space.shape != (152,):
        raise ValueError('Espaços incompatíveis com o simulador V2')
    # Atomic replacement during loading is safe; retry on the next poll rather than mislabel weights.
    stat = checkpoint.path.stat()
    if (stat.st_mtime_ns, stat.st_size) != (checkpoint.modified, checkpoint.size):
        raise ValueError('Checkpoint mudou durante a leitura; tentando novamente depois')
    model.policy.set_training_mode(False)
    model.policy.requires_grad_(False)
    return model


class EvaluationSession:
    """One frozen model per evaluation; metrics never written to the training directory."""
    def __init__(self, max_steps=1500, seed=2_100_000_000, scenarios=8):
        self.env = RoboEnv(level=4, split='viewer_eval', max_steps=max_steps)
        self.seed_base, self.scenarios = seed, scenarios
        self.model = self.checkpoint = None
        self.episode_index = 0
        self.results = {}
        self.history = []
        self.reward = self.total_reward = 0.
        self.action = np.zeros(3)
        self.info = {}
        self.done = False
        self.observation = None

    def summary(self):
        values = list(self.results.values())
        return dict(episodes=len(values), collisions=sum(int(v['collision']) for v in values),
                    success_rate=sum(int(v['success']) for v in values)/len(values) if values else None)

    def activate(self, checkpoint, model):
        if self.checkpoint is not None:
            self.history.append(dict(steps=self.checkpoint.steps, **self.summary()))
            self.history = self.history[-5:]
        self.checkpoint, self.model = checkpoint, model
        self.results = {}
        self.episode_index = 0
        self.restart()

    def restart(self):
        if self.model is None: return
        self.episode_seed = self.seed_base + self.episode_index % self.scenarios
        self.observation, _ = self.env.reset(seed=self.episode_seed)
        self.reward = self.total_reward = 0.
        self.action = np.zeros(3)
        self.info = {}
        self.done = False

    def next_episode(self):
        self.episode_index += 1
        self.restart()

    def step(self):
        if self.model is None or self.done: return
        import torch
        with torch.inference_mode():
            action, _ = self.model.predict(self.observation, deterministic=True)
        self.action = np.clip(np.asarray(action), -1, 1)
        self.observation, self.reward, terminated, truncated, self.info = self.env.step(self.action)
        self.total_reward += self.reward
        self.done = terminated or truncated
        if self.done:
            # Replay of a fixed seed replaces its result; it does not inflate the denominator.
            self.results[self.episode_seed] = dict(self.info)

    def decision_velocity(self):
        target = self.action*self.env.limits
        if target[0] < 0: target[0] = self.action[0]*self.env.reverse_limit
        norm = np.linalg.norm(target[:2])
        if norm > self.env.max_xy: target[:2] *= self.env.max_xy/norm
        return target
