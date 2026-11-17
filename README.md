# PantanalBots #10297 – Simulação 3D

Repositório da equipe **PantanalBots #10297** (SENAI Cáceres-MT) para as atividades
de simulação do curso do prof. **Djair Oliveira** (FRC MT / SENAI).

O código de partida é o [Simulation_3D](https://github.com/FRCMT-Repositories/Simulation_3D)
do prof. Djair (robô MT TANK, WPILib 2026, Java, Command-Based, AdvantageKit).
Cada melhoria da equipe vem em um commit separado (veja o histórico).

## Como rodar

Precisa ter instalados: **WPILib 2026** (já traz o VS Code, o AdvantageScope e o Elastic) e **Git**.

1. `git clone https://github.com/PantanalBots10297/simulacao3D_pantanalbots.git`
2. Abra a pasta no **2026 WPILib VS Code**.
3. `Ctrl+Shift+P` → **Simulate Robot Code** → marque só **Sim GUI**.
4. Na janela **Robot Simulation**: arraste o controle para o **Joystick[0]**,
   marque **Map gamepad** e clique em **Teleoperated**.
5. No **AdvantageScope**: File → Connect to Simulator → importe o layout
   [`advantagescope/layout.json`](advantagescope/layout.json) (File → Import Layout).
   Ele já traz o campo 2025, o robô MT TANK, os componentes (intake, elevador, garra)
   e as bolas (Fuels).
6. Aperte **Start** para o robô entrar no campo.

## Comandos

| Controle | O que faz |
|---|---|
| RT / LT | Frente / ré |
| Analógico direito | Gira |
| Analógico esquerdo | Aponta o robô para uma direção **do campo** (use com RT) |
| LB (segurar) | Intake: a gaveta sai e puxa a Fuel |
| D-Pad ↑ / ↓ | Elevador sobe / desce |
| D-Pad ← / → | Braço a -45° / 45° (só com o elevador em cima) |
| A / B | Braço a 135° / -135° (só com o elevador em cima) |
| Back | Braço a 0° (posição de pegar) |
| Y | Solta / arremessa a Fuel da garra |
| RB (segurar) | Leva o robô até a pose (7,8; 4,0; 60°) |
| **X** | **Liga o autônomo das três bolas. Apertar X de novo cancela** |
| Start | Coloca o robô em (2, 4) |

**AutoHand:** com uma Fuel dentro do robô e o elevador em cima, o braço vai a 0° e o
elevador desce sozinho para a garra pegar a peça.

## Autônomo

O seletor **Auto Selector** (no **Elastic**, em *Add Widget → SmartDashboard*) tem os
autônomos de `Autos.java`: o caminho **AULA** (PathPlanner), **Quadrado**, **Reto**,
**Auto ZigZag** e **Alga (3)**. O **X** do controle liga o Alga (3) sem precisar do seletor.

**Alga (3)** (pedido do prof. Djair): coleta as 3 bolas, lança 2 na **Net** (uma em cada
canto) e 1 no **Processor**. Começa no Start Point do caminho AULA (7,118; 1,402).

Como o braço joga a bola para a **esquerda do robô**:

| Alvo | Robô vira para | Elevador | Braço |
|---|---|---|---|
| Net | -90° (esquerda aponta para leste) | 400 | 160° |
| Processor | 180° (esquerda aponta para o sul) | 100 | 60° |

O robô anda em linha reta entre os pontos, então ele passa por um corredor em
x = 2,70 para não bater no **Reef** (`contornarReef` em `Autos.java`).

## O que a equipe acrescentou

| Melhoria | Onde ver no AdvantageScope |
|---|---|
| **Atividade 1** do prof. Djair: AutoHand, proteção do braço e do elevador | — |
| **Atividade 2**: autônomo das três bolas, botão até uma pose, física do campo 2025 | `Placar/*` |
| **Paredes:** robô e Fuels não saem do campo | — |
| **Obstáculos (Game Manual 2025):** Reefs, suporte central e Cages bloqueiam robô e Fuels; a Barge tem altura de treliça (1,57 a 2,57 m) | — |
| **Net e Processor:** a Fuel cai na rede da Net (1,93 m) ou entra na abertura do Processor (71 x 51 cm, a 18 cm do chão). Fuel na altura da treliça bate na Barge e volta | `Placar/FuelsNaNet`, `Placar/FuelsNoProcessor` |
| **Layout do AdvantageScope** pronto para importar | `advantagescope/layout.json` |

## Estrutura

- `src/main/java/frc/robot/Autos.java`: autônomos.
- `src/main/java/frc/robot/RobotContainer.java`: controles e seletor de autônomo.
- `src/main/java/frc/robot/TankSubsystem.java`: tração, odometria, `driveToPose`, colisões.
- `src/main/java/frc/robot/subsystems/FuelSimulation.java`: física das bolas.
- `src/main/java/frc/robot/Campo.java` e `Obstaculos.java`: medidas e obstáculos do campo 2025.
