package frc.robot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FuelSimulation;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class TankSubsystem extends SubsystemBase {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    private static final double TRACK_WIDTH = 0.60;
    private static final double MAX_SPEED = 3.0;

    private static final double HEADING_KP = 0.035;
    private static final double MAX_ROTATION = 0.5;
    private static final double ANGLE_TOLERANCE = 2.0;

    private static final double POSITION_KP = 0.5;
    private static final double MAX_FORWARD = 0.5;
    private static final double POSITION_TOLERANCE = 0.05;

    // Tamanho do robô com para-choque (metros), igual ao da FuelSimulation.
    private static final double ROBO_COMPRIMENTO = 0.984674;
    private static final double ROBO_LARGURA = 0.8498;

    // Alturas medidas no modelo 3D do MT TANK (metros).
    private static final double ROBO_ALTURA_TORRE = 1.057;  // topo da torre do elevador, elevador embaixo
    private static final double RAIO_RODA_GARRA = 0.051;    // rodinhas na ponta da garra

    // =========================================================
    // CONTROLE E SIMULAÇÃO
    // =========================================================

    private final PIDController rotationPID = new PIDController(HEADING_KP, 0.0, 0.0);
    private final FuelSimulation fuelSimulation = new FuelSimulation();

    private double leftVelocity = 0.0;
    private double rightVelocity = 0.0;

    private double leftDistanceMeters = 0.0;
    private double rightDistanceMeters = 0.0;

    private Rotation2d heading = new Rotation2d();
    private double previousTime = Timer.getFPGATimestamp();

    // =========================================================
    // ODOMETRIA
    // =========================================================

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
            heading,
            leftDistanceMeters,
            rightDistanceMeters,
            new Pose2d());

    public TankSubsystem() {
        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setTolerance(ANGLE_TOLERANCE);
    }

    // =========================================================
    // DRIVE
    // =========================================================

    public void drive(double forward, double backward, double rotation) {
        forward = MathUtil.applyDeadband(forward, 0.08);
        backward = MathUtil.applyDeadband(backward, 0.08);
        rotation = MathUtil.applyDeadband(rotation, 0.08);

        setDrive(forward + backward, rotation);
    }

    private void setDrive(double movement, double rotation) {
        double leftCommand = movement - rotation;
        double rightCommand = movement + rotation;

        leftCommand = MathUtil.clamp(leftCommand, -1.0, 1.0);
        rightCommand = MathUtil.clamp(rightCommand, -1.0, 1.0);

        leftVelocity = leftCommand * MAX_SPEED;
        rightVelocity = rightCommand * MAX_SPEED;
    }

    public void stop() {
        leftVelocity = 0.0;
        rightVelocity = 0.0;
    }

    // =========================================================
    // POSE / ODOMETRIA
    // =========================================================

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        leftDistanceMeters = 0.0;
        rightDistanceMeters = 0.0;

        heading = pose.getRotation();

        odometry.resetPosition(heading, leftDistanceMeters, rightDistanceMeters, pose);
    }

    public Command CMDresetPose(Pose2d pose){
        return Commands.runOnce(() -> resetPose(pose));
    }

    // =========================================================
    // CÁLCULOS DE POSIÇÃO
    // =========================================================

    private double getDistanceToPoint(double targetX, double targetY) {
        Pose2d currentPose = getPose();

        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();

        return Math.hypot(deltaX, deltaY);
    }

    private double getAngleToPoint(double targetX, double targetY) {
        Pose2d currentPose = getPose();

        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();

        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    private double getAngleError(double targetAngle) {
        double currentAngle = getPose().getRotation().getDegrees();

        return MathUtil.inputModulus(targetAngle - currentAngle, -180.0, 180.0);
    }

    // =========================================================
    // CONTROLE DE HEADING
    // =========================================================

    public double getRotationToAngle(double targetAngle) {
        double currentAngle = getPose().getRotation().getDegrees();

        double output = rotationPID.calculate(currentAngle, targetAngle);

        output = MathUtil.clamp(output, -MAX_ROTATION, MAX_ROTATION);

        Logger.recordOutput("Heading/Target", targetAngle);
        Logger.recordOutput("Heading/Current", currentAngle);
        Logger.recordOutput("Heading/Error", rotationPID.getError());
        Logger.recordOutput("Heading/Output", output);

        return output;
    }

    // =========================================================
    // DRIVE ORIENTADO À ARENA
    // =========================================================

    public Command driveWithAngle(DoubleSupplier angleSupplier, DoubleSupplier throttleSupplier) {
        return run(() -> {

            double targetAngle = angleSupplier.getAsDouble();
            double throttle = throttleSupplier.getAsDouble();

            double currentAngle = getPose().getRotation().getDegrees();

            double angleError = MathUtil.inputModulus(targetAngle - currentAngle, -180.0, 180.0);

            // Se o alvo estiver "atrás" do robô,
            // mantém a orientação mais próxima e inverte a tração.
            if (Math.abs(angleError) > 90.0) {
                throttle *= -1.0;

                targetAngle = MathUtil.inputModulus(targetAngle + 180.0, -180.0, 180.0);
            }

            double rotation = getRotationToAngle(targetAngle);

            setDrive(throttle, rotation);

        }).finallyDo(() -> {
            stop();
            rotationPID.reset();
        });
    }

    public void driveWithAngleControl(double targetAngle, double throttle) {

        double currentAngle = getPose().getRotation().getDegrees();

        double angleError = MathUtil.inputModulus(targetAngle - currentAngle, -180.0, 180.0);

        // Se for mais eficiente andar de ré, inverte automaticamente.
        if (Math.abs(angleError) > 90.0) {

            throttle *= -1.0;

            targetAngle = MathUtil.inputModulus(targetAngle + 180.0, -180.0, 180.0);
        }

        double rotation = getRotationToAngle(targetAngle);

        setDrive(throttle, rotation);
    }

    // =========================================================
    // TURN TO ANGLE
    // =========================================================

    public Command turnToAngle(double targetAngle) {
        return run(() -> {
            double rotation = getRotationToAngle(targetAngle);

            setDrive(0.0, rotation);

        }).until(() -> Math.abs(getAngleError(targetAngle)) < ANGLE_TOLERANCE

        ).finallyDo(() -> {
            stop();
            rotationPID.reset();
        });
    }

    // =========================================================
    // TURN TO POINT
    // =========================================================

    public Command turnToPoint(double targetX, double targetY) {
        return run(() -> {
            double targetAngle = getAngleToPoint(targetX, targetY);
            double rotation = getRotationToAngle(targetAngle);

            setDrive(0.0, rotation);

        }).until(() -> {
            double targetAngle = getAngleToPoint(targetX, targetY);

            return Math.abs(getAngleError(targetAngle)) < ANGLE_TOLERANCE;

        }).finallyDo(() -> {
            stop();
            rotationPID.reset();
        });
    }

    // =========================================================
    // DRIVE TO POINT
    // =========================================================

    public Command driveToPoint(double targetX, double targetY) {
        return run(() -> {
            double distance = getDistanceToPoint(targetX, targetY);
            double targetAngle = getAngleToPoint(targetX, targetY);

            double angleError = getAngleError(targetAngle);
            boolean driveBackward = Math.abs(angleError) > 90.0;

            // Usa a traseira do robô quando for o caminho mais curto.
            if (driveBackward) {
                targetAngle = MathUtil.inputModulus(targetAngle + 180.0, -180.0, 180.0);
            }

            double forward = MathUtil.clamp(distance * POSITION_KP, 0.0, MAX_FORWARD);

            if (driveBackward) {
                forward = -forward;
            }

            double rotation = getRotationToAngle(targetAngle);

            setDrive(forward, rotation);

            Logger.recordOutput("DriveToPoint/TargetX", targetX);
            Logger.recordOutput("DriveToPoint/TargetY", targetY);
            Logger.recordOutput("DriveToPoint/Distance", distance);
            Logger.recordOutput("DriveToPoint/TargetAngle", targetAngle);
            Logger.recordOutput("DriveToPoint/Forward", forward);
            Logger.recordOutput("DriveToPoint/Rotation", rotation);
            Logger.recordOutput("DriveToPoint/Backward", driveBackward);

        }).until(() -> getDistanceToPoint(targetX, targetY) < POSITION_TOLERANCE

        ).finallyDo(() -> {
            stop();
            rotationPID.reset();
        });
    }

    // =========================================================
    // DRIVE TO POSE
    // =========================================================

    public Command driveToPose(double x, double y, double rotation) {
        return Commands.sequence(driveToPoint(x, y), turnToAngle(rotation));
    }

    // =========================================================
    // VELOCIDADE NO CAMPO
    // =========================================================

    public double getFieldVelocityX() {
        double linearVelocity = (leftVelocity + rightVelocity) / 2.0;
        double angle = heading.getRadians();

        return linearVelocity * Math.cos(angle);
    }

    public double getFieldVelocityY() {
        double linearVelocity = (leftVelocity + rightVelocity) / 2.0;
        double angle = heading.getRadians();

        return linearVelocity * Math.sin(angle);
    }

    // =========================================================
    // FUEL
    // =========================================================

    public void depositFuel() {
        fuelSimulation.deposit(getPose());
    }

    public Command depositFuelCommand() {
        return runOnce(this::depositFuel);
    }

    public boolean isHandCollected() {
        return fuelSimulation.isHandCollected();
    }

    public boolean isFuelInsideRobot() {
        return fuelSimulation.isFuelInsideRobot();
    }

    // =========================================================
    // PAREDES DO CAMPO
    // =========================================================

    /**
     * Não deixa o robô atravessar as paredes do campo nem os obstáculos
     * do meio (Reefs e Barge).
     *
     * Se alguma parte do robô passou da parede ou entrou num obstáculo,
     * coloca o robô de volta encostado nele e corrige a odometria.
     *
     * Para os obstáculos, o robô é tratado como um retângulo alinhado ao
     * campo (metadeX x metadeY). Na diagonal ele fica um pouco "maior" do
     * que é de verdade e para alguns centímetros antes.
     */
    private Pose2d manterDentroDoCampo(Pose2d pose) {
        double angulo = pose.getRotation().getRadians();

        // Quanto o robô ocupa a partir do centro, em X e em Y do campo.
        // Muda com o giro: virado de lado, ele ocupa mais espaço em Y.
        double metadeX = Math.abs(ROBO_COMPRIMENTO / 2.0 * Math.cos(angulo))
                + Math.abs(ROBO_LARGURA / 2.0 * Math.sin(angulo));
        double metadeY = Math.abs(ROBO_COMPRIMENTO / 2.0 * Math.sin(angulo))
                + Math.abs(ROBO_LARGURA / 2.0 * Math.cos(angulo));

        // "Se X passou do limite, X = limite" (o mesmo para Y).
        double x = MathUtil.clamp(pose.getX(), metadeX, Campo.COMPRIMENTO - metadeX);
        double y = MathUtil.clamp(pose.getY(), metadeY, Campo.LARGURA - metadeY);

        boolean bateuNaParede = x != pose.getX() || y != pose.getY();
        Logger.recordOutput("Campo/RoboNaParede", bateuNaParede);

        // Robô mais alto que a treliça da Barge (1,57 m) não passa por baixo dela.
        double altura = alturaDoRobo();
        boolean bateNaTrelica = altura > Campo.BARGE_TRELICA_ALTURA;
        Logger.recordOutput("Campo/AlturaDoRobo", altura);
        Logger.recordOutput("Campo/RoboAltoDemaisParaBarge", bateNaTrelica);

        // Reefs, Cages e suporte da Barge: se entrou em algum, empurra para fora.
        Obstaculos.Batida batida = Obstaculos.empurrarParaFora(
                x, y, metadeX, metadeY, 0.0, Campo.BARGE_BLOQUEIA_ROBO || bateNaTrelica);
        Logger.recordOutput("Campo/RoboNoObstaculo", batida != null);

        if (batida != null) {
            x = batida.x;
            y = batida.y;
        }

        if (!bateuNaParede && batida == null) {
            return pose;
        }

        Pose2d corrigida = new Pose2d(x, y, pose.getRotation());
        odometry.resetPosition(heading, leftDistanceMeters, rightDistanceMeters, corrigida);
        return corrigida;
    }

    /**
     * Altura do ponto mais alto do robô, em metros, agora.
     *
     * É o maior entre:
     * - o topo da torre, que sobe junto com o elevador (até +0,40 m);
     * - a ponta da garra, que depende do elevador e do ângulo do braço
     *   (0° = para baixo; 135° = quase de ponta-cabeça, bem alto).
     */
    public static double alturaDoRobo() {
        double topoDaTorre = ROBO_ALTURA_TORRE + SimElevator.getRawPosition();
        double pontaDaGarra = SimHand.getWheelCenter().getZ() + RAIO_RODA_GARRA;
        return Math.max(topoDaTorre, pontaDaGarra);
    }

    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousTime;
        previousTime = currentTime;

        // Simula os encoders.
        leftDistanceMeters += leftVelocity * dt;
        rightDistanceMeters += rightVelocity * dt;

        // Calcula velocidades do chassi.
        double linearVelocity = (leftVelocity + rightVelocity) / 2.0;
        double angularVelocity = (rightVelocity - leftVelocity) / TRACK_WIDTH;

        // Simula o giroscópio.
        heading = heading.plus(Rotation2d.fromRadians(angularVelocity * dt));

        // Atualiza odometria e não deixa o robô sair do campo.
        Pose2d pose = manterDentroDoCampo(
                odometry.update(heading, leftDistanceMeters, rightDistanceMeters));

        // Atualiza as game pieces.
        fuelSimulation.update(
                getPose(),
                getFieldVelocityX(),
                getFieldVelocityY(),
                SimIntake.getRawPosition(),
                SimIntake.isRunning(),
                dt);

        // Logs.
        Logger.recordOutput("ODOMETRY", pose);

        Logger.recordOutput("Tank/LeftDistance", leftDistanceMeters);
        Logger.recordOutput("Tank/RightDistance", rightDistanceMeters);

        Logger.recordOutput("Tank/LeftVelocity", leftVelocity);
        Logger.recordOutput("Tank/RightVelocity", rightVelocity);

        Logger.recordOutput("Tank/LinearVelocity", linearVelocity);
        Logger.recordOutput("Tank/AngularVelocity", angularVelocity);
        Logger.recordOutput("Tank/Heading", heading.getDegrees());

        Logger.recordOutput("GAMEPIECE", isHandCollected());
        Logger.recordOutput("GAMEPIECE robot", isFuelInsideRobot());
    }
}