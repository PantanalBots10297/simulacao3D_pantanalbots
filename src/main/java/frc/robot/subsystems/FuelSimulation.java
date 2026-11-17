
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

    private static final double QUIQUE_PAREDE = 0.5;
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
        boolean noProcessor;

        Fuel(double x, double y) {
            this.x = x;
            this.y = y;
            this.z = FUEL_RADIUS;
        }
    }

    public FuelSimulation() {
        addFuel(1.223, 2.200);
        addFuel(1.223, 4.023);
        addFuel(1.223, 5.858);
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

            if (!fuel.beingCollected && !fuel.collected
                    && !fuel.launched && !fuel.naNet && !fuel.noProcessor) {

                checkIntakeCollection(fuel, robotPose, intakePosition, intakeRunning);

                if (!fuel.beingCollected) {
                    checkRobotCollision(fuel, robotPose, robotVelocityX, robotVelocityY);
                }
            }

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

        boolean insideLength = relative[0] >= robotFront
                && relative[0] <= intakeFront;

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

        double closestX = Math.max(-halfLength, Math.min(relativeX, halfLength));
        double closestY = Math.max(-halfWidth, Math.min(relativeY, halfWidth));

        double deltaX = relativeX - closestX;
        double deltaY = relativeY - closestY;

        double distance = Math.hypot(deltaX, deltaY);

        if (distance >= FUEL_RADIUS || distance <= 0.001) {
            return;
        }

        double normalLocalX = deltaX / distance;
        double normalLocalY = deltaY / distance;

        double heading = robotPose.getRotation().getRadians();

        double normalX = normalLocalX * Math.cos(heading)
                - normalLocalY * Math.sin(heading);

        double normalY = normalLocalX * Math.sin(heading)
                + normalLocalY * Math.cos(heading);

        double velocityAlongCollision =
                robotVelocityX * normalX + robotVelocityY * normalY;

        if (velocityAlongCollision > 0.0) {
            fuel.velocityX += normalX * velocityAlongCollision * COLLISION_STRENGTH;
            fuel.velocityY += normalY * velocityAlongCollision * COLLISION_STRENGTH;
        }

        double penetration = FUEL_RADIUS - distance;

        fuel.x += normalX * penetration;
        fuel.y += normalY * penetration;
    }

    // =========================================================
    // FÍSICA
    // =========================================================

    private void updateFuelPhysics(Fuel fuel, Pose2d robotPose, double dt) {

        // Fuel que já pontuou no Processor.
        if (fuel.noProcessor) {
            return;
        }

        // Fuel que caiu na Net.
        if (fuel.naNet) {
            return;
        }

        // -----------------------------------------------------
        // FUEL ARREMESSADA
        // -----------------------------------------------------

        if (fuel.launched) {

            double alturaAntes = fuel.z;
            double xAntes = fuel.x;
            double yAntes = fuel.y;
            double zAntes = fuel.z;

            fuel.velocityZ -= GRAVITY * dt;

            fuel.x += fuel.velocityX * dt;
            fuel.y += fuel.velocityY * dt;
            fuel.z += fuel.velocityZ * dt;

            // IMPORTANTE:
            // Verifica o Processor antes da colisão com a parede.
            if (verificarProcessorAzul(fuel, xAntes, yAntes, zAntes)) {
                return;
            }

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

        // -----------------------------------------------------
        // FUEL NO HAND
        // -----------------------------------------------------

        if (fuel.handCollected) {
            Pose3d handCenter = SimHand.getWheelCenter();

            setFuelRelativeToRobot(
                    fuel,
                    robotPose,
                    handCenter.getX(),
                    handCenter.getY(),
                    handCenter.getZ());

            return;
        }

        // -----------------------------------------------------
        // FUEL ARMAZENADA DENTRO DO ROBÔ
        // -----------------------------------------------------

        if (fuel.collected) {
            setFuelRelativeToRobot(fuel, robotPose, 0.0, 0.0, 0.30);
            return;
        }

        // -----------------------------------------------------
        // FUEL SENDO PUXADA PELO INTAKE
        // -----------------------------------------------------

        if (fuel.beingCollected) {

            fuel.collectionX = moveTowards(
                    fuel.collectionX, 0.0, INTAKE_SPEED * dt);

            fuel.collectionY = moveTowards(
                    fuel.collectionY, 0.0, INTAKE_SPEED * dt);

            setFuelRelativeToRobot(
                    fuel,
                    robotPose,
                    fuel.collectionX,
                    fuel.collectionY,
                    FUEL_RADIUS);

            if (Math.hypot(fuel.collectionX, fuel.collectionY) < COLLECTED_DISTANCE) {

                fuel.collectionX = 0.0;
                fuel.collectionY = 0.0;

                fuel.beingCollected = false;
                fuel.collected = true;
            }

            return;
        }

        // -----------------------------------------------------
        // FUEL LIVRE NO CHÃO
        // -----------------------------------------------------

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
    // PROCESSOR AZUL
    // =========================================================

    private int fuelsNoProcessorAzul = 0;

    /**
     * Detecta a entrada da Fuel no Processor azul.
     *
     * A parede azul fica em Y = 0.
     * O centro da Fuel chega à parede quando Y = FUEL_RADIUS.
     *
     * A posição X e a altura Z são interpoladas no instante
     * em que a Fuel alcança essa posição.
     */
    private boolean verificarProcessorAzul(
            Fuel fuel,
            double xAntes,
            double yAntes,
            double zAntes) {

        double limiteParede = FUEL_RADIUS;

        // A Fuel precisa estar indo em direção à parede azul.
        boolean aproximouDaParede =
                yAntes > limiteParede && fuel.y <= limiteParede;

        if (!aproximouDaParede) {
            return false;
        }

        double distanciaY = yAntes - fuel.y;

        if (distanciaY <= 0.000001) {
            return false;
        }

        // Fração do movimento em que o centro alcançou a parede.
        double fracao = (yAntes - limiteParede) / distanciaY;
        fracao = Math.max(0.0, Math.min(fracao, 1.0));

        // Posição exata aproximada no instante da colisão.
        double xNaParede = xAntes + (fuel.x - xAntes) * fracao;
        double zNaParede = zAntes + (fuel.z - zAntes) * fracao;

        // A Fuel inteira precisa caber na largura da abertura.
        double xMin = Campo.PROCESSOR_AZUL_X
                - Campo.PROCESSOR_LARGURA / 2.0
                + FUEL_RADIUS;

        double xMax = Campo.PROCESSOR_AZUL_X
                + Campo.PROCESSOR_LARGURA / 2.0
                - FUEL_RADIUS;

        // A Fuel inteira precisa caber na altura da abertura.
        double zMin = Campo.PROCESSOR_ALTURA_MIN + FUEL_RADIUS;
        double zMax = Campo.PROCESSOR_ALTURA_MAX - FUEL_RADIUS;

        boolean dentroDaLargura =
                xNaParede >= xMin && xNaParede <= xMax;

        boolean dentroDaAltura =
                zNaParede >= zMin && zNaParede <= zMax;

        Logger.recordOutput("Processor/Azul/XNaParede", xNaParede);
        Logger.recordOutput("Processor/Azul/ZNaParede", zNaParede);
        Logger.recordOutput("Processor/Azul/DentroDaLargura", dentroDaLargura);
        Logger.recordOutput("Processor/Azul/DentroDaAltura", dentroDaAltura);

        if (!dentroDaLargura || !dentroDaAltura) {
            Logger.recordOutput("Processor/Azul/Pontuou", false);
            return false;
        }

        // A Fuel atravessou a abertura.
        fuel.x = xNaParede;
        fuel.y = -FUEL_RADIUS;
        fuel.z = zNaParede;

        fuel.velocityX = 0.0;
        fuel.velocityY = 0.0;
        fuel.velocityZ = 0.0;

        fuel.launched = false;
        fuel.noProcessor = true;

        fuelsNoProcessorAzul++;

        Logger.recordOutput("Processor/Azul/Pontuou", true);
        Logger.recordOutput("Placar/FuelsNoProcessorAzul", fuelsNoProcessorAzul);

        return true;
    }

    /** Retorna quantas Fuels pontuaram no Processor azul. */
    public int getFuelsNoProcessorAzul() {
        return fuelsNoProcessorAzul;
    }

    // =========================================================
    // NET
    // =========================================================

    private int fuelsNaNet = 0;

    private boolean caiuNaNet(Fuel fuel, double alturaAntes) {

        double alturaDaRede = Campo.NET_ALTURA + FUEL_RADIUS;

        boolean emCimaDaBarge =
                fuel.x >= Campo.BARGE_X_MIN && fuel.x <= Campo.BARGE_X_MAX;

        boolean passouPelaRede =
                alturaAntes >= alturaDaRede && fuel.z < alturaDaRede;

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

    private void bateuNaLateralDaBarge(Fuel fuel, double xAntes) {

        double faceAzul = Campo.BARGE_X_MIN - FUEL_RADIUS;
        double faceVermelha = Campo.BARGE_X_MAX + FUEL_RADIUS;

        boolean naAlturaDaTrelica =
                fuel.z > Campo.BARGE_TRELICA_ALTURA
                        && fuel.z < Campo.BARGE_TOPO_ALTURA;

        boolean estavaFora =
                xAntes <= faceAzul || xAntes >= faceVermelha;

        boolean entrou =
                fuel.x > faceAzul && fuel.x < faceVermelha;

        if (!naAlturaDaTrelica || !estavaFora || !entrou) {
            return;
        }

        if (fuel.velocityX > 0.0) {
            fuel.x = Campo.BARGE_X_MIN - FUEL_RADIUS;
            fuel.velocityX = -fuel.velocityX * QUIQUE_PAREDE;
        } else {
            fuel.x = Campo.BARGE_X_MAX + FUEL_RADIUS;
            fuel.velocityX = -fuel.velocityX * QUIQUE_PAREDE;
        }
    }

    public int getFuelsNaNet() {
        return fuelsNaNet;
    }

    // =========================================================
    // OBSTÁCULOS
    // =========================================================

    private void quicarNosObstaculos(Fuel fuel) {

        Obstaculos.Batida batida = Obstaculos.empurrarParaFora(
                fuel.x, fuel.y, 0.0, 0.0, FUEL_RADIUS);

        if (batida == null) {
            return;
        }

        fuel.x = batida.x;
        fuel.y = batida.y;

        double contraOObstaculo =
                fuel.velocityX * batida.normalX
                        + fuel.velocityY * batida.normalY;

        if (contraOObstaculo < 0.0) {
            fuel.velocityX -=
                    (1.0 + QUIQUE_PAREDE) * contraOObstaculo * batida.normalX;

            fuel.velocityY -=
                    (1.0 + QUIQUE_PAREDE) * contraOObstaculo * batida.normalY;
        }
    }

    // =========================================================
    // FUEL X FUEL
    // =========================================================

    private boolean estaLivreNoChao(Fuel fuel) {
        return !fuel.beingCollected
                && !fuel.collected
                && !fuel.handCollected
                && !fuel.launched
                && !fuel.naNet
                && !fuel.noProcessor;
    }

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
                    continue;
                }

                double nx = dx / distancia;
                double ny = dy / distancia;

                double sobreposicao = distanciaMinima - distancia;

                a.x -= nx * sobreposicao / 2.0;
                a.y -= ny * sobreposicao / 2.0;

                b.x += nx * sobreposicao / 2.0;
                b.y += ny * sobreposicao / 2.0;

                double aproximacao =
                        (b.velocityX - a.velocityX) * nx
                                + (b.velocityY - a.velocityY) * ny;

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

        double distance = Math.sqrt(
                deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

        Logger.recordOutput("Simulation/Hand/DistanceToFuel", distance);

        return distance <= HAND_CAPTURE_DISTANCE;
    }

    // =========================================================
    // LANÇAMENTO
    // =========================================================

    public void deposit(Pose2d robotPose) {

        for (Fuel fuel : fuels) {

            if (!fuel.handCollected) {
                continue;
            }

            double[] direction = SimHand.getLaunchDirection();

            double localVelocityX = direction[0] * LAUNCH_SPEED;
            double localVelocityY = direction[1] * LAUNCH_SPEED;
            double localVelocityZ = direction[2] * LAUNCH_SPEED;

            double heading = robotPose.getRotation().getRadians();

            fuel.velocityX =
                    localVelocityX * Math.cos(heading)
                            - localVelocityY * Math.sin(heading);

            fuel.velocityY =
                    localVelocityX * Math.sin(heading)
                            + localVelocityY * Math.cos(heading);

            fuel.velocityZ = localVelocityZ;

            fuel.collected = false;
            fuel.handCollected = false;
            fuel.beingCollected = false;
            fuel.launched = true;

            Logger.recordOutput(
                    "Simulation/Launch/HandAngle",
                    Math.toDegrees(SimHand.getRawAngle()));

            Logger.recordOutput(
                    "Simulation/Launch/DirectionY", direction[1]);

            Logger.recordOutput(
                    "Simulation/Launch/DirectionZ", direction[2]);

            Logger.recordOutput(
                    "Simulation/Launch/InitialPosition",
                    new double[] {fuel.x, fuel.y, fuel.z});

            Logger.recordOutput(
                    "Simulation/Launch/Velocity",
                    new double[] {
                            fuel.velocityX,
                            fuel.velocityY,
                            fuel.velocityZ
                    });

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

    private double[] getFuelRelativeToRobot(
            Fuel fuel,
            Pose2d robotPose) {

        double deltaX = fuel.x - robotPose.getX();
        double deltaY = fuel.y - robotPose.getY();

        double heading = robotPose.getRotation().getRadians();

        double relativeX =
                deltaX * Math.cos(heading) + deltaY * Math.sin(heading);

        double relativeY =
                -deltaX * Math.sin(heading) + deltaY * Math.cos(heading);

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

        fuel.x = robotPose.getX()
                + relativeX * Math.cos(heading)
                - relativeY * Math.sin(heading);

        fuel.y = robotPose.getY()
                + relativeX * Math.sin(heading)
                + relativeY * Math.cos(heading);

        fuel.z = relativeZ;
    }

    // =========================================================
    // UTILIDADES
    // =========================================================

    private double moveTowards(
            double current,
            double target,
            double maxDelta) {

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

        double deslocamentoModelo = SmartDashboard.getNumber(
                "Fuel/AlturaModelo",
                FUEL_MODELO_DESLOCAMENTO_Z);

        SmartDashboard.putNumber(
                "Fuel/AlturaModelo",
                deslocamentoModelo);

        Pose3d[] fuelPoses = new Pose3d[fuels.size()];

        for (int i = 0; i < fuels.size(); i++) {

            Fuel fuel = fuels.get(i);

            fuelPoses[i] = new Pose3d(
                    fuel.x,
                    fuel.y,
                    fuel.z - deslocamentoModelo,
                    new Rotation3d());
        }

        Logger.recordOutput("Simulation/Fuels", fuelPoses);
        Logger.recordOutput("Placar/FuelsNaNet", fuelsNaNet);
        Logger.recordOutput(
                "Placar/FuelsNoProcessorAzul",
                fuelsNoProcessorAzul);
    }
}
