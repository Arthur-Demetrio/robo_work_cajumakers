#!/usr/bin/env python3
"""Preparacao REVERSIVEL do deadband no YAML. Nao reinicia o Nav2.

Modo padrao: mostra preview e valida o arquivo.
--apply: faz backup com timestamp, altera apenas linha inline do deadband.
"""
import argparse
import datetime
import re
import shutil
from pathlib import Path
import yaml

p = argparse.ArgumentParser()
p.add_argument("--apply", action="store_true")
p.add_argument(
    "--config", default=str(Path.home() /
    "robo_jazzy_ws/src/robo_navigation/config/nav2_params.yaml")
)
args = p.parse_args()
file = Path(args.config)
text = file.read_text()
config = yaml.safe_load(text)
params = config.get("velocity_smoother", {}).get("ros__parameters", {})
old = params.get("deadband_velocity")
if not isinstance(old, list) or len(old) != 3:
    raise SystemExit("Arquivo sem deadband_velocity esperado: alteracao cancelada")
if not all(isinstance(x, (int, float)) for x in old):
    raise SystemExit("Deadband invalido: alteracao cancelada")
pattern = re.compile(r"(?m)^(\s*deadband_velocity:\s*)\[([^\]\n]*)\](\s*)$")
matches = list(pattern.finditer(text))
if len(matches) != 1:
    raise SystemExit("Formato inline ambiguo, nenhuma alteracao realizada")
target = [0.005, 0.005, 0.015]
print("Arquivo:", file)
print("Atual:", old)
print("Proposto:", target)
print("Angular preservada?", abs(old[2] - target[2]) < 1e-9)
if abs(old[2] - target[2]) > 1e-9:
    raise SystemExit("Deadband angular diferente de 0.015: revisar manualmente")
if all(abs(a-b)<1e-9 for a,b in zip(old,target)):
    print("Ja esta configurado. Nenhuma alteracao necessaria.")
    raise SystemExit(0)
if not args.apply:
    print("PREVIA APENAS. Use --apply com Nav2 parado, apos revisar.")
    raise SystemExit(0)
backup = file.with_name(
    file.name + ".bak_deadband_" +
    datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
)
shutil.copy2(file, backup)
new_text = pattern.sub(
    lambda m: m.group(1) + "[0.005, 0.005, 0.015]" + m.group(3),
    text, count=1
)
new_params = yaml.safe_load(new_text)
new_vals = new_params["velocity_smoother"]["ros__parameters"]["deadband_velocity"]
if new_vals != target:
    raise SystemExit("Validacao da alteracao falhou; original preservado")
file.write_text(new_text)
print("BACKUP:", backup)
print("Alteracao concluida no YAML; reinicie o Nav2 para aplicar.")
