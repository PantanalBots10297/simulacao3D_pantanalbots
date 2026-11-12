package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Campo;
import frc.robot.Obstaculos;

public class FuelSimulation {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    private static final double FUEL_RADIUS = 0.15;

    /**
     * Só para DESENHAR no AdvantageScope. O modelo 3D da Fuel tem uma haste
     * embaixo da bola, então ele parece flutuar. Esse valor (em metros) abaixa
     * o modelo para a bola ficar no chão. A física NÃO usa isso.
     *
     * Dá para ajustar AO VIVO na simulação: na Sim GUI, janela "NetworkTables",
     * em SmartDashboard > Fuel/AlturaModelo (muda o número e vê na hora).
     * Bola com haste à mostra: aumente. Bola afundada no chão: diminua.
     * Valor testado na simulação: -0,052 (negativo = sobe o modelo, a bola
     * fica inteira e apoiada no chão).
     */
    private static final double FUEL_MODELO_DESLOCAMENTO_Z = -0.052;

    private static final double ROBOT_LENGTH = 0.984674;
    private static final double ROBOT_WIDTH = 0.8498;

    private static final double COLLISION_STRENGTH = 0.70;
    private static final double FRICTION = 1.5;
    private static final double MIN_VELOCITY = 0.03;

    private static final double INTAKE_EXTENSION = 0.30;
    private static final double INTAKE_OPEN_POSITION = 0.214334 * 0.95;
    private static final double INTAKE_SPEED = 1.5;
    private static final double COLLECTED_DISTANCE = 0.03;

    private static final double HAND_CAPTURE_DISTANCE = 0.20;

    private static final double GRAVITY = 9.81;
    private static final double LAUNCH_SPEED = 4.0;

    // Quanto da velocidade a Fuel mantém ao bater na parede (0 = para, 1 = não perde nada).
    private static final double QUIQUE_PAREDE = 0.5;

    // Quanto da velocidade sobra quando uma Fuel bate na outra (0 a 1).
    private static final double QUIQUE_FUEL = 0.8;

    // =========================================================
    // FUELS
    // =========================================================

    private final List<Fuel> fuels = new ArrayList<>();

    private static class Fuel {

        double x;
        double y;
        double z;

        double velocityX;
        double velocityY;
        double velocityZ;

        double collectionX;
        double collectionY;

        boolean beingCollected;
        boolean collected;
        boolean handCollected;
        boolean launched;
        boolean naNet;

        Fuel(double x, double y) {
            this.x = x;
            this.y = y;
            this.z = FUEL_RADIUS;
        }
    }

    public FuelSimulation() {
        addFuel(1.223, 2.200); // Game piece 1
        addFuel(1.223, 4.023); // Game piece 2
        addFuel(1.223, 5.858); // Game piece 3

        // addFuel(3.0, 6.0);
        // addFuel(4.0, 6.0);
        // addFuel(5.0, 6.0);
    }

    public void addFuel(double x, double y) {
        fuels.add(new Fuel(x, y));
    }

    // =========================================================
    // UPDATE
    // =========================================================

    public void update(
            Pose2d robotPose,
            double robotVelocityX,
            double robotVelocityY,
            double intakePosition,
            boolean intakeRunning,
            double dt) {
        for (Fuel fuel : fuels) {

            // Fuel livre pode interagir com Intake e robô.
            if (!fuel.beingCollected && !fuel.collected && !fuel.launched && !fuel.naNet) {
                checkIntakeCollection(fuel, robotPose, intakePosition, intakeRunning);

                if (!fuel.beingCollected) {
                    checkRobotCollision(fuel, robotPose, robotVelocityX, robotVelocityY);
                }
            }

            // Fuel armazenada pode ser capturada pelo Hand.
            if (fuel.collected && !fuel.handCollected) {
                checkHandCollection(fuel, robotPose);
            }

            updateFuelPhysics(fuel, robotPose, dt);
        }

        colisaoEntreFuels();

        logFuels();
    }

    // =========================================================
    // INTAKE
    // =========================================================

    private void checkIntakeCollection(
            Fuel fuel,
            Pose2d robotPose,
            double intakePosition,
            boolean intakeRunning) {
        // Não coleta se o armazenamento já estiver ocupado.
        if (isFuelInsideRobot()) {
            return;
        }

        if (!intakeRunning) {
            return;
        }

        if (intakePosition < INTAKE_OPEN_POSITION) {
            return;
        }

        if (!isFuelInsideIntake(fuel, robotPose, intakePosition)) {
            return;
        }

        double[] relative = getFuelRelativeToRobot(fuel, robotPose);

        fuel.collectionX = relative[0];
        fuel.collectionY = relative[1];

        fuel.velocityX = 0.0;
        fuel.velocityY = 0.0;

        fuel.beingCollected = true;
    }

    private boolean isFuelInsideIntake(
            Fuel fuel,
            Pose2d robotPose,
            double intakePosition) {
        double[] relative = getFuelRelativeToRobot(fuel, robotPose);

        double intakePercent = intakePosition / 0.214334;
        intakePercent = Math.max(0.0, Math.min(intakePercent, 1.0));

        double currentExtension = INTAKE_EXTENSION * intakePercent;

        double robotFront = ROBOT_LENGTH / 2.0;
        double intakeFront = robotFront + currentExtension;
        double halfWidth = ROBOT_WIDTH / 2.0;

        boolean insideLength = relative[0] >= robotFront &&
                relative[0] <= intakeFront;

        boolean insideWidth = Math.abs(relative[1]) <= halfWidth;

        boolean insideIntake = insideLength && insideWidth;

        Logger.recordOutput("Simulation/IntakePosition", intakePosition);
        Logger.recordOutput("Simulation/IntakePercent", intakePercent);
        Logger.recordOutput("Simulation/IntakeExtension", currentExtension);
        Logger.recordOutput("Simulation/InsideIntake", insideIntake);

        return insideIntake;
    }

    // =========================================================
    // COLISÃO ROBÔ X FUEL
    // =========================================================

    private void checkRobotCollision(
            Fuel fuel,
            Pose2d robotPose,
            double robotVelocityX,
            double robotVelocityY) {
        double[] relative = getFuelRelativeToRobot(fuel, robotPose);

        double relativeX = relative[0];
        double relativeY = relative[1];

        double halfLength = ROBOT_LENGTH / 2.0;
        double halfWidth = ROBOT_WIDTH / 2.0;

        // Ponto do robô mais próximo da Fuel.
        double closestX = Math.max(-halfLength, Math.min(relativeX, halfLength));
        double closestY = Math.max(-halfWidth, Math.min(relativeY, halfWidth));

        double deltaX = relativeX - closestX;
        double deltaY = relativeY - closestY;

        double distance = Math.hypot(deltaX, deltaY);

        if (distance >= FUEL_RADIUS || distance <= 0.001) {
            return;
        }

        // Normal da colisão no referencial do robô.
        double normalLocalX = deltaX / distance;
        double normalLocalY = deltaY / distance;

        // Converte a normal para o referencial da arena.
        double heading = robotPose.getRotation().getRadians();

        double normalX = normalLocalX * Math.cos(heading) - normalLocalY * Math.sin(heading);
        double normalY = normalLocalX * Math.sin(heading) + normalLocalY * Math.cos(heading);

        double velocityAlongCollision = robotVelocityX * normalX + robotVelocityY * normalY;

        // Transfere parte da velocidade do robô para a Fuel.
        if (velocityAlongCollision > 0.0) {
            fuel.velocityX += normalX * velocityAlongCollision * COLLISION_STRENGTH;

            fuel.velocityY += normalY * velocityAlongCollision * COLLISION_STRENGTH;
        }

        // Remove a penetração entre robô e Fuel.
        double penetration = FUEL_RADIUS - distance;

        fuel.x += normalX * penetration;
        fuel.y += normalY * penetration;
    }

    // =========================================================
    // FÍSICA
    // =========================================================

    private void updateFuelPhysics(Fuel fuel, Pose2d robotPose, double dt) {
        // ---------------------------------------------------------
        // Fuel arremessada
        // ---------------------------------------------------------

        // ---------------------------------------------------------
        // Fuel que caiu na Net: fica parada lá em cima
        // ---------------------------------------------------------

        if (fuel.naNet) {
            return;
        }

        if (fuel.launched) {
            double alturaAntes = fuel.z;
            double xAntes = fuel.x;

            fuel.velocityZ -= GRAVITY * dt;

            fuel.x += fuel.velocityX * dt;
            fuel.y += fuel.velocityY * dt;
            fuel.z += fuel.velocityZ * dt;

            quicarNasParedes(fuel);

            if (caiuNaNet(fuel, alturaAntes)) {
                return;
            }

            bateuNaLateralDaBarge(fuel, xAntes);

            if (fuel.z <= FUEL_RADIUS) {
                fuel.z = FUEL_RADIUS;
                fuel.velocityZ = 0.0;
                fuel.launched = false;
            }

            return;
        }

        // ---------------------------------------------------------
        // Fuel no Hand
        // ---------------------------------------------------------

        if (fuel.handCollected) {
            Pose3d handCenter = SimHand.getWheelCenter();

            setFuelRelativeToRobot(fuel, robotPose, handCenter.getX(), handCenter.getY(), handCenter.getZ());

            return;
        }

        // ---------------------------------------------------------
        // Fuel armazenada dentro do robô
        // ---------------------------------------------------------

        if (fuel.collected) {
            setFuelRelativeToRobot(fuel, robotPose, 0.0, 0.0, 0.30);

            return;
        }

        // ---------------------------------------------------------
        // Fuel sendo puxada pelo Intake
        // ---------------------------------------------------------

        if (fuel.beingCollected) {
            fuel.collectionX = moveTowards(fuel.collectionX, 0.0, INTAKE_SPEED * dt);
            fuel.collectionY = moveTowards(fuel.collectionY, 0.0, INTAKE_SPEED * dt);

            setFuelRelativeToRobot(fuel, robotPose, fuel.collectionX, fuel.collectionY, FUEL_RADIUS);

            if (Math.hypot(fuel.collectionX, fuel.collectionY) < COLLECTED_DISTANCE) {

                fuel.collectionX = 0.0;
                fuel.collectionY = 0.0;

                fuel.beingCollected = false;
                fuel.collected = true;
            }

            return;
        }

        // ---------------------------------------------------------
        // Fuel livre no chão
        // ---------------------------------------------------------

        double resistance = Math.exp(-FRICTION * dt);

        fuel.velocityX *= resistance;
        fuel.velocityY *= resistance;

        if (Math.hypot(fuel.velocityX, fuel.velocityY) < MIN_VELOCITY) {

            fuel.velocityX = 0.0;
            fuel.velocityY = 0.0;
        }

        fuel.x += fuel.velocityX * dt;
        fuel.y += fuel.velocityY * dt;
        fuel.z = FUEL_RADIUS;

        quicarNasParedes(fuel);
        quicarNosObstaculos(fuel);
    }

    // =========================================================
    // NET
    // =========================================================

    private int fuelsNaNet = 0;

    /**
     * A Fuel arremessada caiu na Net? Isso acontece quando ela está em
     * cima da Barge e, descendo, passa pela altura da rede.
     * Se caiu, ela para lá em cima e conta no placar.
     */
    private boolean caiuNaNet(Fuel fuel, double alturaAntes) {
        double alturaDaRede = Campo.NET_ALTURA + FUEL_RADIUS;

        boolean emCimaDaBarge = fuel.x >= Campo.BARGE_X_MIN && fuel.x <= Campo.BARGE_X_MAX;
        boolean passouPelaRede = alturaAntes >= alturaDaRede && fuel.z < alturaDaRede;

        if (!emCimaDaBarge || !passouPelaRede) {
            return false;
        }

        fuel.z = alturaDaRede;
        fuel.velocityX = 0.0;
        fuel.velocityY = 0.0;
        fuel.velocityZ = 0.0;
        fuel.launched = false;
        fuel.naNet = true;

        fuelsNaNet++;
        return true;
    }

    /**
     * Fuel arremessada na altura da treliça (entre 1,57 m e o topo, 2,57 m)
     * bate na lateral da Barge e volta. Mais baixa, passa por baixo; mais
     * alta, passa por cima e pode cair na Net.
     *
     * Só conta como batida quando a Fuel ATRAVESSA a face da Barge neste
     * ciclo (estava fora e entrou). Fuel que já está em cima da Barge,
     * caindo na rede, não bate na lateral.
     */
    private void bateuNaLateralDaBarge(Fuel fuel, double xAntes) {
        double faceAzul = Campo.BARGE_X_MIN - FUEL_RADIUS;
        double faceVermelha = Campo.BARGE_X_MAX + FUEL_RADIUS;

        boolean naAlturaDaTrelica = fuel.z > Campo.BARGE_TRELICA_ALTURA && fuel.z < Campo.BARGE_TOPO_ALTURA;
        boolean estavaFora = xAntes <= faceAzul || xAntes >= faceVermelha;
        boolean entrou = fuel.x > faceAzul && fuel.x < faceVermelha;

        if (!naAlturaDaTrelica || !estavaFora || !entrou) {
            return;
        }

        if (fuel.velocityX > 0.0) {
            // Vinha do lado azul (x menor): volta para lá.
            fuel.x = Campo.BARGE_X_MIN - FUEL_RADIUS;
            fuel.velocityX = -fuel.velocityX * QUIQUE_PAREDE;
        } else {
            // Vinha do lado vermelho (x maior): volta para lá.
            fuel.x = Campo.BARGE_X_MAX + FUEL_RADIUS;
            fuel.velocityX = -fuel.velocityX * QUIQUE_PAREDE;
        }
    }

    /** Quantas Fuels já caíram na Net. */
    public int getFuelsNaNet() {
        return fuelsNaNet;
    }

    // =========================================================
    // OBSTÁCULOS (Reefs e Barge)
    // =========================================================

    /**
     * Se a Fuel no chão entrou num Reef ou na Barge, volta para fora e
     * quica: perde a velocidade que ia contra o obstáculo e volta com
     * parte dela (QUIQUE_PAREDE).
     *
     * Fuel arremessada não entra aqui: ela passa por cima da Barge até a Net.
     */
    private void quicarNosObstaculos(Fuel fuel) {
        Obstaculos.Batida batida = Obstaculos.empurrarParaFora(fuel.x, fuel.y, 0.0, 0.0, FUEL_RADIUS);

        if (batida == null) {
            return;
        }

        fuel.x = batida.x;
        fuel.y = batida.y;

        // Velocidade na direção do obstáculo (negativa = indo contra ele).
        double contraOObstaculo = fuel.velocityX * batida.normalX + fuel.velocityY * batida.normalY;

        if (contraOObstaculo < 0.0) {
            fuel.velocityX -= (1.0 + QUIQUE_PAREDE) * contraOObstaculo * batida.normalX;
            fuel.velocityY -= (1.0 + QUIQUE_PAREDE) * contraOObstaculo * batida.normalY;
        }
    }

    // =========================================================
    // FUEL x FUEL
    // =========================================================

    /** A Fuel está solta no chão? (não está no robô, na garra nem no ar) */
    private boolean estaLivreNoChao(Fuel fuel) {
        return !fuel.beingCollected && !fuel.collected && !fuel.handCollected && !fuel.launched && !fuel.naNet;
    }

    /**
     * Confere cada par de Fuels soltas no chão. Se as duas estão se
     * sobrepondo (distância menor que dois raios), afasta uma da outra
     * e troca a velocidade entre elas, como bolas de sinuca.
     */
    private void colisaoEntreFuels() {
        double distanciaMinima = 2.0 * FUEL_RADIUS;

        for (int i = 0; i < fuels.size(); i++) {
            for (int j = i + 1; j < fuels.size(); j++) {
                Fuel a = fuels.get(i);
                Fuel b = fuels.get(j);

                if (!estaLivreNoChao(a) || !estaLivreNoChao(b)) {
                    continue;
                }

                double dx = b.x - a.x;
                double dy = b.y - a.y;
                double distancia = Math.hypot(dx, dy);

                if (distancia >= distanciaMinima || distancia < 0.0001) {
                    continue; // não estão encostando
                }

                // Direção de A para B (vetor de tamanho 1).
                double nx = dx / distancia;
                double ny = dy / distancia;

                // 1. Afasta as duas, metade para cada lado.
                double sobreposicao = distanciaMinima - distancia;
                a.x -= nx * sobreposicao / 2.0;
                a.y -= ny * sobreposicao / 2.0;
                b.x += nx * sobreposicao / 2.0;
                b.y += ny * sobreposicao / 2.0;

                // 2. Se estão se aproximando, troca velocidade na direção do choque.
                double aproximacao = (b.velocityX - a.velocityX) * nx + (b.velocityY - a.velocityY) * ny;

                if (aproximacao < 0.0) {
                    double impulso = -(1.0 + QUIQUE_FUEL) * aproximacao / 2.0;

                    a.velocityX -= impulso * nx;
                    a.velocityY -= impulso * ny;
                    b.velocityX += impulso * nx;
                    b.velocityY += impulso * ny;
                }
            }
        }
    }

    // =========================================================
    // PAREDES
    // =========================================================

    /**
     * Se a Fuel passou de uma parede, volta para dentro e quica:
     * a velocidade naquela direção troca de sinal e perde força.
     */
    private void quicarNasParedes(Fuel fuel) {
        double minimo = FUEL_RADIUS;
        double maximoX = Campo.COMPRIMENTO - FUEL_RADIUS;
        double maximoY = Campo.LARGURA - FUEL_RADIUS;

        if (fuel.x < minimo) {
            fuel.x = minimo;
            fuel.velocityX = Math.abs(fuel.velocityX) * QUIQUE_PAREDE;
        } else if (fuel.x > maximoX) {
            fuel.x = maximoX;
            fuel.velocityX = -Math.abs(fuel.velocityX) * QUIQUE_PAREDE;
        }

        if (fuel.y < minimo) {
            fuel.y = minimo;
            fuel.velocityY = Math.abs(fuel.velocityY) * QUIQUE_PAREDE;
        } else if (fuel.y > maximoY) {
            fuel.y = maximoY;
            fuel.velocityY = -Math.abs(fuel.velocityY) * QUIQUE_PAREDE;
        }
    }

    // =========================================================
    // HAND
    // =========================================================

    private void checkHandCollection(Fuel fuel, Pose2d robotPose) {
        if (!fuel.collected || fuel.handCollected) {
            return;
        }

        if (isFuelInsideHand(fuel, robotPose)) {
            fuel.handCollected = true;
        }
    }

    private boolean isFuelInsideHand(Fuel fuel, Pose2d robotPose) {
        Pose3d handCenter = SimHand.getWheelCenter();

        double[] fuelRelative = getFuelRelativeToRobot(fuel, robotPose);

        double deltaX = fuelRelative[0] - handCenter.getX();
        double deltaY = fuelRelative[1] - handCenter.getY();
        double deltaZ = fuel.z - handCenter.getZ();

        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

        Logger.recordOutput("Simulation/Hand/DistanceToFuel", distance);

        return distance <= HAND_CAPTURE_DISTANCE;
    }

    // =========================================================
    // LANÇAMENTO
    // =========================================================

    public void deposit(Pose2d robotPose) {

        for (Fuel fuel : fuels) {

            // Somente uma Fuel que está no Hand pode ser lançada.
            if (!fuel.handCollected) {
                continue;
            }

            double[] direction = SimHand.getLaunchDirection();

            double localVelocityX = direction[0] * LAUNCH_SPEED;
            double localVelocityY = direction[1] * LAUNCH_SPEED;
            double localVelocityZ = direction[2] * LAUNCH_SPEED;

            double heading = robotPose.getRotation().getRadians();

            // Converte velocidade local para coordenadas da arena.
            fuel.velocityX = localVelocityX * Math.cos(heading) - localVelocityY * Math.sin(heading);
            fuel.velocityY = localVelocityX * Math.sin(heading) + localVelocityY * Math.cos(heading);
            fuel.velocityZ = localVelocityZ;

            fuel.collected = false;
            fuel.handCollected = false;
            fuel.beingCollected = false;
            fuel.launched = true;

            Logger.recordOutput("Simulation/Launch/HandAngle", Math.toDegrees(SimHand.getRawAngle()));
            Logger.recordOutput("Simulation/Launch/DirectionY", direction[1]);
            Logger.recordOutput("Simulation/Launch/DirectionZ", direction[2]);

            // Apenas uma Fuel é lançada por chamada.
            return;
        }
    }

    // =========================================================
    // ESTADOS
    // =========================================================

    public boolean isHandCollected() {
        for (Fuel fuel : fuels) {
            if (fuel.handCollected) {
                return true;
            }
        }

        return false;
    }

    public boolean isFuelInsideRobot() {
        for (Fuel fuel : fuels) {
            if (fuel.collected && !fuel.handCollected) {
                return true;
            }
        }

        return false;
    }

    // =========================================================
    // TRANSFORMAÇÕES
    // =========================================================

    private double[] getFuelRelativeToRobot(Fuel fuel, Pose2d robotPose) {
        double deltaX = fuel.x - robotPose.getX();
        double deltaY = fuel.y - robotPose.getY();

        double heading = robotPose.getRotation().getRadians();

        double relativeX = deltaX * Math.cos(heading) + deltaY * Math.sin(heading);

        double relativeY = -deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

        return new double[] {
                relativeX,
                relativeY
        };
    }

    private void setFuelRelativeToRobot(
            Fuel fuel,
            Pose2d robotPose,
            double relativeX,
            double relativeY,
            double relativeZ) {
        double heading = robotPose.getRotation().getRadians();

        fuel.x = robotPose.getX() + relativeX * Math.cos(heading) - relativeY * Math.sin(heading);
        fuel.y = robotPose.getY() + relativeX * Math.sin(heading) + relativeY * Math.cos(heading);
        fuel.z = relativeZ;
    }

    // =========================================================
    // UTILIDADES
    // =========================================================

    private double moveTowards(double current, double target, double maxDelta) {
        double difference = target - current;

        if (Math.abs(difference) <= maxDelta) {
            return target;
        }

        return current + Math.copySign(maxDelta, difference);
    }

    // =========================================================
    // ADVANTAGESCOPE
    // =========================================================

    private void logFuels() {
        double deslocamentoModelo = SmartDashboard.getNumber("Fuel/AlturaModelo", FUEL_MODELO_DESLOCAMENTO_Z);
        SmartDashboard.putNumber("Fuel/AlturaModelo", deslocamentoModelo);

        Pose3d[] fuelPoses = new Pose3d[fuels.size()];

        for (int i = 0; i < fuels.size(); i++) {
            Fuel fuel = fuels.get(i);

            fuelPoses[i] = new Pose3d(fuel.x, fuel.y, fuel.z - deslocamentoModelo, new Rotation3d());
        }

        Logger.recordOutput("Simulation/Fuels", fuelPoses);
        Logger.recordOutput("Placar/FuelsNaNet", fuelsNaNet);
    }
}