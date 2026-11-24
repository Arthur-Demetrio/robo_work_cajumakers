#!/usr/bin/env bash
# TESTE_GIRO_PWM v1 (05/11/2026): mede o giro parado com minimum_rotation_pwm 80, 65, 50 (ou os dados).
#
# Por que: o giro parado mais lento hoje e ~15 graus/s (PWM de rotacao 80). Correcoes pequenas
# (2-7 graus) passam do ponto e voltam: no cruzamento, com 12 cm de folga, a quina encosta na
# parede (saida lateral para a esquerda falhou nas missoes 13-17).
#
# Para cada valor: troca no hardware.yaml, compila, pede para voce REINICIAR o robo
# (Ctrl+C no subir_robo.sh e rodar de novo), confere o valor na ponte e roda o teste de giro.
# No fim volta para 80 e pede para reiniciar.
# Resultados: ~/teste_velocidade_giro_pwmrot<N>.txt (me mande).
#
# Uso:  bash teste_giro_pwm.sh            (ou: bash teste_giro_pwm.sh 70 60 50)
set +u
CFG=~/robo_jazzy_ws/src/robo_hardware/config/hardware.yaml
V=$(grep -l "VELOCIDADE_V2" ~/Downloads/teste_velocidade*.py 2>/dev/null | head -1)
VALORES=("$@"); [ ${#VALORES[@]} -eq 0 ] && VALORES=(65 50)
[ -n "$V" ] || { echo "Nao achei o teste_velocidade_v2.py em ~/Downloads."; exit 1; }
grep -q "^\s*minimum_rotation_pwm:" "$CFG" || { echo "Nao achei minimum_rotation_pwm em $CFG."; exit 1; }
[ -f "$CFG.bak_teste_giro" ] || cp "$CFG" "$CFG.bak_teste_giro"

trocar() {
  sed -i "s/^\(\s*minimum_rotation_pwm:\) [0-9]*/\1 $1/" "$CFG"
  grep -n "minimum_rotation_pwm" "$CFG"
  (cd ~/robo_jazzy_ws && colcon build --packages-select robo_hardware > /tmp/teste_giro_build.log 2>&1) \
    || { echo "colcon build falhou (/tmp/teste_giro_build.log)"; exit 1; }
  echo
  echo ">>> REINICIE o robo: Ctrl+C no terminal do subir_robo.sh e rode ~/subir_robo.sh de novo."
  echo "    Espere o ROBO PRONTO e aperte Enter aqui."
  read -r _
  source ~/robo_jazzy_ws/install/setup.bash
  atual=$(ros2 param get /robo_hardware minimum_rotation_pwm 2>/dev/null | grep -o "[0-9]*$")
  [ "$atual" = "$1" ] || { echo "   A ponte responde minimum_rotation_pwm=${atual:-?} (esperado $1). Reinicie e aperte Enter."; read -r _; }
}

for P in "${VALORES[@]}"; do
  echo; echo "################ minimum_rotation_pwm = $P ################"
  trocar "$P"
  echo ">>> GIRO: 0,6 m livres em volta do robo. Enter."
  read -r _
  python3 "$V" --giro --pwm "rot$P" 0.02 0.03 0.05 0.10 0.20
  echo "(anote se em algum comando o robo NAO girou, girou aos trancos ou arrastou roda)"
done
echo; echo "################ voltando para minimum_rotation_pwm = 80 ################"
sed -i "s/^\(\s*minimum_rotation_pwm:\) [0-9]*/\1 80/" "$CFG"; grep -n "minimum_rotation_pwm" "$CFG"
(cd ~/robo_jazzy_ws && colcon build --packages-select robo_hardware > /tmp/teste_giro_build.log 2>&1)
echo ">>> Reinicie o robo uma ultima vez (subir_robo.sh). Arquivos:"; ls -1 ~/teste_velocidade_giro_pwmrot*.txt 2>/dev/null
