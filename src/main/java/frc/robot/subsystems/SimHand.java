package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimHand extends SubsystemBase {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    // Limites físicos da articulação no CAD, em radianos.
    private static final double HAND_MIN_ANGLE = -Math.PI;
    private static final double HAND_MAX_ANGLE = Math.PI;

    // Limites equivalentes do encoder real, em graus.
    private static double encoderMin = -180.0;
    private static double encoderMax = 180.0;

    // Geometria do Hand no CAD.
    private static final double HAND_LENGTH = 0.715675;
    private static final double WHEEL_OFFSET_ANGLE = 13.97;

    private static final double WHEEL_X = 0.028800;
    private static final double HAND_HEIGHT = 1.032257;
    private static final double HAND_X = -0.162433;

    // =========================================================
    // CONTROLE
    // =========================================================

    private final PIDController anglePID = new PIDController(1.5, 0.0, 0.0);

    private final PIDController velocityPID = new PIDController(1.5, 0.0, 0.0);

    private static double currentAngle = 0.0;
    private static double setAngle = 0.0;

    private static double currentVelocity = 0.0;
    private static double setVelocity = 0.0;

    // Rotação visual dos roletes.
    private static double rollerRotation = 0.0;

    // =========================================================
    // CONSTRUTOR
    // =========================================================

    public SimHand(
            double articulationMin,
            double articulationMax,
            double angleKP,
            double wheelKP) {
        encoderMin = articulationMin;
        encoderMax = articulationMax;

        anglePID.setP(angleKP);
        velocityPID.setP(wheelKP);

        // Permite ao PID escolher o menor caminho entre -180° e +180°.
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {
        updateAngle();
        updateVelocity();
        updateVisualization();
    }

    // =========================================================
    // ARTICULAÇÃO
    // =========================================================

    /**
     * Define o ângulo desejado usando a escala do encoder real.
     */
    public void setAngle(double angle, double kP) {
        anglePID.setP(kP);

        setAngle = map(angle, encoderMin, encoderMax, HAND_MIN_ANGLE, HAND_MAX_ANGLE);
    }

    /**
     * Retorna o ângulo atual do Hand no CAD, em radianos.
     */
    public static double getRawAngle() {
        return currentAngle;
    }

    /**
     * Retorna o ângulo atual convertido para a escala do encoder.
     */
    public static double getRealAngle() {
        return map(currentAngle, HAND_MIN_ANGLE, HAND_MAX_ANGLE, encoderMin, encoderMax);
    }

    private void updateAngle() {
        double output = anglePID.calculate(currentAngle, setAngle);

        currentAngle += output * 0.02;
        currentAngle = MathUtil.inputModulus(currentAngle, -Math.PI, Math.PI);
    }

    // =========================================================
    // VELOCIDADE DOS ROLETES
    // =========================================================

    /**
     * Define a velocidade desejada dos roletes do Hand.
     */
    public void setVelocity(double velocity) {
        setVelocity = velocity;
    }

    /**
     * Retorna a velocidade atual simulada dos roletes.
     */
    public static double getRawVelocity() {
        return currentVelocity;
    }

    private void updateVelocity() {
        double output = velocityPID.calculate(currentVelocity, setVelocity);

        currentVelocity += output * 0.02;

        // Rotação utilizada apenas para animação no AdvantageScope.
        rollerRotation += 0.1 * currentVelocity;
        rollerRotation = MathUtil.inputModulus(rollerRotation, -Math.PI, Math.PI);
    }

    // =========================================================
    // GEOMETRIA DO HAND
    // =========================================================

    /**
     * Calcula a posição da roda 1 em relação ao robô.
     */
    private static Pose3d getWheel1Pose() {
        double angle = WHEEL_OFFSET_ANGLE + Math.toDegrees(currentAngle);

        double co = HAND_LENGTH * Math.sin(Math.toRadians(angle));
        double ca = HAND_LENGTH * Math.cos(Math.toRadians(angle));

        return new Pose3d(WHEEL_X, co, SimElevator.getRawPosition() + HAND_HEIGHT - ca,
                new Rotation3d(rollerRotation, 0.0, 0.0));
    }

    /**
     * Calcula a posição da roda 2 em relação ao robô.
     */
    private static Pose3d getWheel2Pose() {
        double angle = -WHEEL_OFFSET_ANGLE + Math.toDegrees(currentAngle);

        double co = HAND_LENGTH * Math.sin(Math.toRadians(angle));
        double ca = HAND_LENGTH * Math.cos(Math.toRadians(angle));

        return new Pose3d(WHEEL_X, co, SimElevator.getRawPosition() + HAND_HEIGHT - ca,
                new Rotation3d(-rollerRotation, 0.0, 0.0));
    }

    /**
     * Retorna o ponto central entre as duas rodas do Hand.
     * Esse ponto é utilizado pela FuelSimulation para detectar a coleta.
     */
    public static Pose3d getWheelCenter() {
        Pose3d wheel1 = getWheel1Pose();
        Pose3d wheel2 = getWheel2Pose();

        double centerX = (wheel1.getX() + wheel2.getX()) / 2.0;
        double centerY = (wheel1.getY() + wheel2.getY()) / 2.0;
        double centerZ = (wheel1.getZ() + wheel2.getZ()) / 2.0;

        return new Pose3d(centerX, centerY, centerZ, new Rotation3d());
    }

    // =========================================================
    // LANÇAMENTO
    // =========================================================

    /**
     * Retorna a direção de lançamento da Fuel no referencial do robô.
     *
     * [0] = X
     * [1] = Y
     * [2] = Z
     */
    public static double[] getLaunchDirection() {
        double angle = currentAngle;

        return new double[] {
                0.0,
                Math.sin(angle),
                -Math.cos(angle)
        };
    }

    // =========================================================
    // VISUALIZAÇÃO
    // =========================================================

    private void updateVisualization() {
        Pose3d wheel1 = getWheel1Pose();
        Pose3d wheel2 = getWheel2Pose();

        Logger.recordOutput("SubSystemSim/Hand", new Pose3d[] {
                new Pose3d(HAND_X, 0.0, SimElevator.getRawPosition() + HAND_HEIGHT,
                        new Rotation3d(currentAngle, 0.0, 0.0)) });

        Logger.recordOutput("SubSystemSim/Hand/wheel1", new Pose3d[] { wheel1 });
        Logger.recordOutput("SubSystemSim/Hand/wheel2", new Pose3d[] { wheel2 });
        Logger.recordOutput("SubSystemSim/HandVelocity", currentVelocity);
        Logger.recordOutput("SubSystemSim/Hand/InputAngle", getRealAngle());
        Logger.recordOutput("SubSystemSim/Hand/CurrentAngle", Math.toDegrees(currentAngle));
        Logger.recordOutput("SubSystemSim/Hand/SetAngle", Math.toDegrees(setAngle));
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