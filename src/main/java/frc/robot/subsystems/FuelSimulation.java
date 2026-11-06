package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class FuelSimulation {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    private static final double FUEL_RADIUS = 0.15;

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

        Fuel(double x, double y) {
            this.x = x;
            this.y = y;
            this.z = FUEL_RADIUS;
        }
    }

    public FuelSimulation() {
        addFuel(3.0, 2.0);
        addFuel(4.0, 2.0);
        addFuel(5.0, 2.0);

        addFuel(3.0, 6.0);
        addFuel(4.0, 6.0);
        addFuel(5.0, 6.0);
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
            if (!fuel.beingCollected && !fuel.collected && !fuel.launched) {
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

        if (fuel.launched) {
            fuel.velocityZ -= GRAVITY * dt;

            fuel.x += fuel.velocityX * dt;
            fuel.y += fuel.velocityY * dt;
            fuel.z += fuel.velocityZ * dt;

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
        Pose3d[] fuelPoses = new Pose3d[fuels.size()];

        for (int i = 0; i < fuels.size(); i++) {
            Fuel fuel = fuels.get(i);

            fuelPoses[i] = new Pose3d(fuel.x, fuel.y, fuel.z, new Rotation3d());
        }

        Logger.recordOutput("Simulation/Fuels", fuelPoses);
    }
}