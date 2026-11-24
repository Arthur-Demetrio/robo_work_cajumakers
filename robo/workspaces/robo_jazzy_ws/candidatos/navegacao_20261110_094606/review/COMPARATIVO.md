# Comparação offline

Plugins Navfn e DWB reais; integração ideal dos comandos, sem física, BT, Collision Monitor ou recuperação neste laço. Tempos não são medições no robô.

| Perfil | Cenário | Replaneja | Chegou | Duração s | Sem translação s | Yaw intermediário rad | Primeiro lateral x m | Interseções |
|---|---|---|---|---:|---:|---:|---:|---:|
| shim | open | False | True | 6.3 | 0 | 0 | -1 | 0 |
| shim | lateral | False | True | 12.5 | 11.9 | 1.48 | 0 | 0 |
| shim | curve | False | True | 9.7 | 2.3 | 0.509333 | 0 | 0 |
| shim | near | False | False | 1.4 | 0.1 | 0.109333 | 0 | 1 |
| shim | corner | False | False | 5.3 | 2.3 | 0.773333 | 0 | 1 |
| shim | final_yaw | False | True | 11.9 | 8.2 | 0 | -1 | 0 |
| shim | obstacle | False | False | 4.9 | 1.2 | 0.170667 | 0.431333 | 0 |
| shim | obstacle | True | False | 30 | 7.5 | 2.50133 | 0 | 0 |
| shim | dynamic | True | False | 30 | 9.3 | 3.312 | 0.184667 | 0 |
| dwb | open | False | True | 6.3 | 0 | 0 | -1 | 0 |
| dwb | lateral | False | True | 0.7 | 0 | 0 | 0 | 0 |
| dwb | curve | False | True | 7.1 | 0 | 0.141926 | 0 | 0 |
| dwb | near | False | False | 1.4 | 0.1 | 0.109333 | 0 | 1 |
| dwb | corner | False | False | 8.4 | 2.1 | 0.592 | 0 | 0 |
| dwb | final_yaw | False | True | 11.9 | 8.2 | 0 | -1 | 0 |
| dwb | obstacle | False | False | 4.9 | 1.2 | 0.170667 | 0.431333 | 0 |
| dwb | obstacle | True | False | 30 | 6.9 | 1.54133 | 0.0253333 | 0 |
| dwb | dynamic | True | False | 14.4 | 3.3 | 0.744 | 0.424667 | 0 |
| early | open | False | True | 6.8 | 0 | 0 | -1 | 0 |
| early | lateral | False | True | 0.7 | 0 | 0 | 0 | 0 |
| early | curve | False | True | 7.9 | 0 | 0.0906667 | 0 | 0 |
| early | near | False | False | 30 | 0.6 | 0.757778 | 0 | 0 |
| early | corner | False | False | 6.6 | 1.5 | 0.267111 | 0 | 0 |
| early | final_yaw | False | True | 12.6 | 8.8 | 0 | -1 | 0 |
| early | obstacle | False | False | 5 | 1.3 | 0.181333 | 0.422 | 0 |
| early | obstacle | True | True | 28 | 0 | 0.165333 | 0 | 0 |
| early | dynamic | True | False | 30 | 4.6 | 1.35467 | 0.184667 | 0 |
| predictive | open | False | True | 7.8 | 0 | 0 | -1 | 0 |
| predictive | lateral | False | True | 0.8 | 0 | 0 | 0 | 0 |
| predictive | curve | False | True | 8 | 0.3 | 0.304 | 0 | 0 |
| predictive | near | False | False | 30 | 0 | 0.656 | 0 | 0 |
| predictive | corner | False | False | 30 | 12.8 | 3.92533 | 0 | 0 |
| predictive | final_yaw | False | True | 13.4 | 8.7 | 0 | -1 | 0 |
| predictive | obstacle | False | False | 30 | 0.5 | 1.17333 | 0.383332 | 0 |
| predictive | obstacle | True | True | 23.1 | 1.5 | 0.501333 | 0 | 0 |
| predictive | dynamic | True | True | 27.2 | 1.4 | 0.826667 | 0.2 | 0 |

Caixa inicia em x=0,85 m; frente do footprint em x=0,365 m. x=-1 significa nenhum comando lateral. Primeiro lateral é indicador, não prova isolada de desvio útil. Yaw intermediário não classifica automaticamente todo giro como desnecessário.

Recuperações não medidas neste laço. Os contornos concluídos não executaram recuperação. A árvore integrada ainda precisa de ensaios fechados com todos os nós e sensores. As interseções da referência são geométricas neste modelo, não colisões físicas comprovadas.
