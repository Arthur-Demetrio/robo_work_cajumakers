# Robo-WORK

**Robô móvel omnidirecional com rodas mecanum, desenvolvido para navegação autônoma em ROS 2 Jazzy.**

O Robo-WORK reúne firmware, percepção por LiDAR, orientação por IMU, mapeamento, navegação e estratégias de recuperação em espaços estreitos. O objetivo é operar em uma bancada desconhecida, mapear o ambiente e desenvolver a identificação de bancadas/estações sem depender de uma pose inicial específica.

## Tecnologias

ROS 2 Jazzy · Nav2 · SLAM Toolbox · LiDAR 2D · IMU BNO055 · Arduino Mega 2560 · rodas mecanum

## Onde está cada parte

| Pasta | Conteúdo |
| --- | --- |
| [src/](src/) | Pacotes ROS 2: hardware, LiDAR, localização, navegação, recuperação e identificação de estações |
| [firmware/workotvio/](firmware/workotvio/) | Firmware Arduino de referência `workotvio.ino` |
| [scripts/](scripts/) | Diagnósticos, ensaios de geometria e teleoperação |
| [config/](config/) | Configurações de apoio do repositório |
| [terminais/](terminais/) | Comandos e rotinas de operação |
| [docs/](docs/) | Manuais, arquitetura e especificações |
| [tests/](tests/) | Histórico dos testes |

## Estado do projeto

- **Referência do código-fonte:** importação de 19/10/2026, commit `be59784`, integrada à `main` pelo [PR #6](../../pull/6).
- **Pacotes de navegação e recuperação:** mantidos como referência do desenvolvimento e dos testes realizados.
- **Identificação de bancadas/estações:** trabalho em evolução. As pastas `robo_station_detector`, `robo_station_manager` e diferentes variantes de `robo_station_tracker` incluem abordagens experimentais; não presumir que todas sejam usadas juntas.
- **Validação:** a presença de um arquivo no Git não comprova compilação, teste físico ou que ele corresponda ao executável atualmente instalado no robô.

## Documentação útil

- [Manual de operação](docs/manual-operacao.md)
- [Especificações de hardware](docs/especificacoes-hardware.md)
- [Arquitetura](docs/arquitetura.md) e [geometria](docs/geometria.md)
- [Comandos por terminal](terminais/README.md)
- [Histórico de testes](tests/historico.md)
- [Versionamento e backups](docs/versionamento-backup.md)
- [Procedimento de importação](docs/importacao-codigos.md)

## Como trabalhar com segurança

O workspace operacional do notebook é `~/robo_jazzy_ws`; o repositório Git é uma cópia para versionamento. Antes de aplicar mudanças no robô, confira diferenças de código e parâmetros, faça backup e valide o comportamento no ambiente físico. Desenvolva alterações em branches separadas e integre à `main` depois de revisão.

> **Importante:** este repositório é um histórico técnico e uma referência de desenvolvimento, não um pacote certificado ou uma indicação de que todas as funcionalidades estão estáveis.
