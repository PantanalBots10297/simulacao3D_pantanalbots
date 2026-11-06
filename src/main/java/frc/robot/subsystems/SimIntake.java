package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimIntake extends SubsystemBase {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    // Posição física do Intake no CAD, em metros.
    private static final double INTAKE_ZERO = 0.0;
    private static final double INTAKE_FORWARD = 0.214334;

    // Ângulo de deslocamento do Intake no CAD.
    private static final double INTAKE_ANGLE = 5.71;

    // Posições equivalentes do encoder.
    private static double encoderZero = 0.0;
    private static double encoderForward = 50.0;

    // =========================================================
    // CONTROLE
    // =========================================================

    private final PIDController positionPID = new PIDController(1.5, 0.0, 0.0);
    private final PIDController velocityPID = new PIDController(1.5, 0.0, 0.0);

    private static double setPosition = 0.0;
    private static double currentPosition = 0.0;

    private static double setVelocity = 0.0;
    private static double currentVelocity = 0.0;

    // Rotação visual do rolete no AdvantageScope.
    private static double rollerRotation = 0.0;

    // =========================================================
    // CONSTRUTOR
    // =========================================================

    public SimIntake(
            double realForward,
            double positionKP,
            double wheelKP) {
        encoderForward = realForward;

        positionPID.setP(positionKP);
        velocityPID.setP(wheelKP);
    }

    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {
        updatePosition();
        updateVelocity();
        updateVisualization();
    }

    // =========================================================
    // POSIÇÃO
    // =========================================================

    /**
     * Define a posição desejada do Intake usando a escala do encoder.
     */
    public void setPosition(double position, double kP) {
        positionPID.setP(kP);

        setPosition = map(position, encoderZero, encoderForward, INTAKE_ZERO, INTAKE_FORWARD);
    }

    /**
     * Retorna a posição física do Intake no CAD, em metros.
     */
    public static double getRawPosition() {
        return currentPosition;
    }

    /**
     * Retorna a posição do Intake convertida para a escala do encoder.
     */
    public static double getRealPosition() {
        return map(currentPosition, INTAKE_ZERO, INTAKE_FORWARD, encoderZero, encoderForward);
    }

    private void updatePosition() {
        double output = positionPID.calculate(currentPosition, setPosition);

        currentPosition += output * 0.02;
        currentPosition = MathUtil.clamp(currentPosition, INTAKE_ZERO, INTAKE_FORWARD);
    }

    // =========================================================
    // VELOCIDADE DO ROLETE
    // =========================================================

    /**
     * Define a velocidade desejada do rolete do Intake.
     */
    public void setVelocity(double velocity) {
        setVelocity = velocity;
    }

    /**
     * Retorna a velocidade atual simulada do rolete.
     */
    public static double getRawVelocity() {
        return currentVelocity;
    }

    /**
     * Indica se o rolete do Intake está em movimento.
     */
    public static boolean isRunning() {
        return Math.abs(currentVelocity) > 0.05;
    }

    private void updateVelocity() {
        double output = velocityPID.calculate(currentVelocity, setVelocity);

        currentVelocity += output * 0.02;

        // Atualiza somente a representação visual do rolete.
        rollerRotation += 0.1 * currentVelocity;
        rollerRotation = MathUtil.inputModulus(rollerRotation, -Math.PI, Math.PI);
    }

    // =========================================================
    // COMMANDS
    // =========================================================

    public Command CMDintakeOn() {
        return Commands.sequence(
                Commands.runOnce(() -> setPosition(20, 3)),     // Avança o Intake.
                Commands.waitUntil(() -> getRealPosition() > 15),           // Aguarda o mecanismo avançar.
                Commands.runOnce(() -> setVelocity(-3)));                   // Liga o rolete.
    }

    public Command CMDintakeOff() {
        return Commands.sequence(
                Commands.runOnce(() -> setPosition(0, 3)),  // Recolhe o Intake.
                Commands.waitUntil(() -> getRealPosition() <= 15),      // Aguarda o mecanismo recolher.
                Commands.runOnce(() -> setVelocity(0)));       // Desliga o rolete.
    }

    // =========================================================
    // VISUALIZAÇÃO
    // =========================================================

    private void updateVisualization() {
        double intakeCO = currentPosition * Math.sin(Math.toRadians(INTAKE_ANGLE));
        double intakeCA = currentPosition * Math.cos(Math.toRadians(INTAKE_ANGLE));

        Logger.recordOutput("SubSystemSim/Intake/Velocity", currentVelocity);

        Logger.recordOutput("SubSystemSim/Intake/Gaveta", new Pose3d[] {
                new Pose3d(intakeCA, 0.0, -intakeCO, new Rotation3d()) });

        Logger.recordOutput("SubSystemSim/Intake/Coletor", new Pose3d[] {
                new Pose3d(intakeCA + 0.336804, 0.0, -intakeCO + 0.238576, new Rotation3d(0.0, rollerRotation, 0.0)) });
    }

    // =========================================================
    // UTILIDADES
    // =========================================================

    /**
     * Converte linearmente um valor de uma escala para outra.
     */
    private static double map(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }
}