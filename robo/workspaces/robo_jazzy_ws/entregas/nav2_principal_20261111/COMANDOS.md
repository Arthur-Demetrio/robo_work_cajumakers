# Ativacao futura (NAO executada)

Usar somente numa janela de teste, sem uma segunda instancia de Nav2 rodando. Este comando sobe navegacao; nao envia goal. Localizacao/sensores seguem procedimento do projeto.

```bash
cd ~/robo_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install_nav2_principal/setup.bash
ros2 launch robo_navigation navigation.launch.py nav2_principal:=true
```

O perfil novo escolhe automaticamente nav2_params_principal.yaml e navigate_to_pose_principal.xml, sem os tres patches pendentes M34.

A/B legado com EXATAMENTE os mesmos parametros/BT (em outra janela de teste, nunca simultaneo):

```bash
ros2 launch robo_navigation navigation.launch.py nav2_principal:=false params_file:=$HOME/robo_jazzy_ws/install_nav2_principal/robo_navigation/share/robo_navigation/config/nav2_params_principal.yaml bt_xml:=$HOME/robo_jazzy_ws/install_nav2_principal/robo_navigation/share/robo_navigation/behavior_trees/navigate_to_pose_principal.xml
```

A flag e somente de inicializacao. Nao alternar em goal ativo. `nav2_principal:=false` sozinho escolhe os arquivos padrao do workspace (que ja continham patches pendentes); para comparar sem promove-los use o comando A/B acima.

Rollback de fontes, com conferencia contra alteracoes posteriores:

```bash
python3 ~/robo_jazzy_ws/entregas/nav2_principal_20261111/rollback.py
```

Rollback nao reinicia processos nem remove backups. O overlay isolado fica preservado, mas nao deve ser utilizado apos rollback de fontes sem recompilar. Para usar instalacao operacional novamente, abrir shell novo e carregar install/setup.bash conforme procedimento anterior.
