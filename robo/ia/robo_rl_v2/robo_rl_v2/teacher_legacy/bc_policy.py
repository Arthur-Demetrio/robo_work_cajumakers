"""Inference wrapper for the isolated phase-conditioned BC candidate."""
from pathlib import Path
import sys
import numpy as np
import torch

_TRAINER = Path(__file__).resolve().parents[2] / "experimentos_v2_20261110" / "imitation"
if str(_TRAINER) not in sys.path:
    sys.path.insert(0, str(_TRAINER))
from train_bc import Actor, PHASES, OBS


class BehaviorCloningPolicy:
    """Load a candidate only when explicitly called; returns normalized action labels."""
    def __init__(self, checkpoint):
        self.path = Path(checkpoint).resolve()
        payload = torch.load(self.path, map_location="cpu", weights_only=True)
        if tuple(payload.get("phases", ())) != PHASES or payload.get("observation_size") != OBS:
            raise ValueError("Checkpoint BC incompatível com fases/observação esperadas")
        self.actor = Actor().cpu().eval()
        self.actor.load_state_dict(payload["state_dict"], strict=True)

    @torch.no_grad()
    def predict(self, observation, phase, deterministic=True):
        if phase not in PHASES:
            raise ValueError(f"Fase desconhecida: {phase}")
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape != (OBS,) or not np.isfinite(obs).all():
            raise ValueError(f"Observação deve conter {OBS} valores finitos")
        onehot = np.zeros((1, len(PHASES)), dtype=np.float32)
        onehot[0, PHASES.index(phase)] = 1.0
        action = self.actor(torch.from_numpy(obs[None]), torch.from_numpy(onehot))[0].numpy()
        return action.astype(np.float32), None
