# Comparativo offline NAV_INTEGRADA

Gerado por `ferramentas/testes/comparar_nav_integrada.py`. Nada disto e medicao no robo.

## Arvores (servidores simulados, mesmos cenarios)

| Caso | Arvore | Resultado | Planejamentos | Atraso replan s | destravar | spin | limpezas |
|---|---|---|---:|---:|---:|---:|---:|
| caminho livre | integrada | SUCCESS | 1 | - | 0 | 0 | 0 |
| caminho livre | anterior | SUCCESS | 1 | - | 0 | 0 | 0 |
| caminho livre | reduzida | SUCCESS | 1 | - | 0 | 0 | 0 |
| obstaculo novo no restante (t=2,3 s) | integrada | SUCCESS | 2 | 0.2 | 0 | 0 | 0 |
| obstaculo novo no restante (t=2,3 s) | anterior | SUCCESS | 2 | 0.7 | 0 | 0 | 0 |
| obstaculo novo no restante (t=2,3 s) | reduzida | SUCCESS | 2 | 0.2 | 0 | 0 | 0 |
| obstaculo so no trecho percorrido | integrada | SUCCESS | 1 | - | 0 | 0 | 0 |
| obstaculo so no trecho percorrido | anterior | SUCCESS | 6 | - | 0 | 0 | 0 |
| obstaculo so no trecho percorrido | reduzida | SUCCESS | 1 | - | 0 | 0 | 0 |
| suavizacao colide, original valido | integrada | SUCCESS | 1 | - | 0 | 0 | 0 |
| suavizacao colide, original valido | anterior | SUCCESS | 1 | - | 0 | 0 | 0 |
| suavizacao colide, original valido | reduzida | FAILURE | 1 | - | 0 | 0 | 0 |
| DWB sem trajetoria 1x | integrada | SUCCESS | 2 | - | 1 | 0 | 0 |
| DWB sem trajetoria 1x | anterior | SUCCESS | 1 | - | 1 | 0 | 0 |
| DWB sem trajetoria 1x | reduzida | FAILURE | 1 | - | 0 | 0 | 0 |
| DWB sem trajetoria, recuperacao impossivel | integrada | FAILURE | 1 | - | 1 | 0 | 0 |
| DWB sem trajetoria, recuperacao impossivel | anterior | FAILURE | 1 | - | 4 | 2 | 4 |
| DWB sem trajetoria, recuperacao impossivel | reduzida | FAILURE | 1 | - | 0 | 0 | 0 |

## Controle (DWB/Navfn/costmaps reais, laco ideal sem CM nem fisica)

| Perfil | Cenario | Chegou | Duracao s | Sem translacao s | Giro intermediario rad | Intersecoes | max vy |
|---|---|---|---:|---:|---:|---:|---:|
| anterior (Shim) | open | True | 6.3 | 0 | 0 | 0 | 0 |
| anterior (Shim) | lateral | True | 12.5 | 11.9 | 1.48 | 0 | 0.0266667 |
| anterior (Shim) | curve | True | 9.7 | 2.3 | 0.509 | 0 | 0.0533333 |
| anterior (Shim) | final_yaw | True | 11.9 | 8.2 | 0 | 0 | 0 |
| anterior (Shim) | obstacle | False | 30 | 7.5 | 2.501 | 0 | 0.08 |
| anterior (Shim) | dynamic | False | 30 | 9.3 | 3.312 | 0 | 0.08 |
| anterior (Shim) | corner | False | 30 | 15.7 | 4.263 | 0 | 0.08 |
| anterior (Shim) | near | False | 0.2 | 0.2 | 0.08 | 1 | 0 |
| integrada (DWB direto) | open | True | 6.3 | 0 | 0 | 0 | 0 |
| integrada (DWB direto) | lateral | True | 0.7 | 0 | 0 | 0 | 0.08 |
| integrada (DWB direto) | curve | True | 7.1 | 0 | 0.142 | 0 | 0.08 |
| integrada (DWB direto) | final_yaw | True | 11.9 | 8.2 | 0 | 0 | 0 |
| integrada (DWB direto) | obstacle | False | 30 | 6.9 | 1.541 | 0 | 0.08 |
| integrada (DWB direto) | dynamic | False | 14.4 | 3.3 | 0.744 | 0 | 0.0533333 |
| integrada (DWB direto) | corner | False | 30 | 4.9 | 1.024 | 0 | 0.08 |
| integrada (DWB direto) | near | False | 30 | 5.3 | 1.064 | 0 | 0.0533333 |
