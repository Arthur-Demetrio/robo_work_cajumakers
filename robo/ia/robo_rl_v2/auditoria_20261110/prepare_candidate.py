from pathlib import Path
import shutil
out=Path(__file__).parent;root=out.parent;c=out/'candidata'
c.mkdir(exist_ok=True)
shutil.copytree(root/'robo_rl_v2',c/'robo_rl_v2',ignore=shutil.ignore_patterns('__pycache__'),dirs_exist_ok=True)
shutil.copytree(root/'assets',c/'assets',dirs_exist_ok=True)
p=c/'robo_rl_v2/maps.py';s=p.read_text();needle='        self.cells[max(0,y0):min(self.cells.shape[0],y1), max(0,x0):min(self.cells.shape[1],x1)] = 1'
s=s.replace(needle,'''        # Reject the entire rectangle if clipped or intersecting occupied/unknown cells.
        if x0 < 0 or y0 < 0 or x1 > self.cells.shape[1] or y1 > self.cells.shape[0]:
            return False
        region = self.cells[y0:y1, x0:x1]
        if region.size == 0 or np.any(region != 0):
            return False
        region[:] = 1
        return True''');p.write_text(s)
p=c/'robo_rl_v2/env.py';s=p.read_text();s=s.replace('''            limits=np.where(np.abs(delayed)>np.abs(self.velocity),self.accel,self.decel)
            self.velocity += np.clip(delayed-self.velocity,-limits*h,limits*h)''','''            # Brake to zero before accelerating in the opposite direction.
            for axis in range(3):
                v, target_v = self.velocity[axis], delayed[axis]
                remaining = h
                if v * target_v < 0:
                    braking_time = abs(v) / self.decel[axis]
                    used = min(remaining, braking_time)
                    v -= np.sign(v) * self.decel[axis] * used
                    remaining -= used
                    if remaining > 0: v = 0.
                rate = self.accel[axis] if abs(target_v) > abs(v) else self.decel[axis]
                self.velocity[axis] = v + np.clip(target_v-v, -rate*remaining, rate*remaining)''');p.write_text(s)
(c/'README.md').write_text('''# Candidata de correção — não treinada e não ativada

Cópia isolada. Nenhum checkpoint foi modificado. Não continuar o treino principal com esta versão sem nova validação.

Correções restritas a problemas demonstráveis:
- `Grid.add_box` rejeita caixas cortadas pelos limites ou sobrepostas a células ocupadas/desconhecidas. O retorno booleano permite auditar colocações rejeitadas. Ainda pode gerar menos caixas que o nível pretendia: adicionar reamostragem com contagem explícita antes de novo treino.
- A inversão de velocidade freia até zero e só então acelera no sentido oposto, respeitando separadamente os limites configurados.

Não altera recompensas nem presume localização perfeita. Revisar localização, protocolo de currículo e domínio físico antes de investir em novo treinamento. Esta candidata não demonstrou ganho de sucesso; corrige contratos locais, não o aprendizado completo.
''')
