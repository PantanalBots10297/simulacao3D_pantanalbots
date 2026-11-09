package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

    private static SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
    private static SimElevator mSimElevator = new SimElevator(400, 1.5);
    private static SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

    // private static RobotContainer m_robotContainer;

    public Autos() {
        // m_robotContainer = new RobotContainer();
    }

    public static Command quadrado(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(4.0, 2.0, 90),
                tank.driveToPose(4.0, 5.0, 180),
                tank.driveToPose(2.0, 5.0, -90));
    }

    public static Command reto(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(5.0, 2.0, 0));
    }

    public static Command autoZigZag(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                mSimIntake.CMDintakeOn(),
                tank.driveToPose(4.0, 3.0, 45),
                tank.driveToPose(6.0, 2.0, -45));
    }

    /**
     * Atividade 2 - Desafio 2: navegar pela odometria ate a pose solicitada.
     * O comando parte da pose atual, sem reiniciar o odometro.
     * Pode ser chamado no modo teleoperado por um botao do controle.
     */
    public static Command navegarParaPoseAlvo(TankSubsystem tank) {
        return Commands.sequence(
                tank.driveToPose(7.8, 4.0, 60.0),
                Commands.runOnce(tank::stop, tank));
    }

    // Atividade 2: pontos de coleta obtidos de FuelSimulation.java.
    // Pontos NET/PROCESSOR abaixo sao somente aproximacoes para bancada;
    // validar com o mapa escolhido e com o tamanho do robo antes da entrega.
    private static final double COLETA_X = 1.95;
    private static final double NET_X = 7.45;
    private static final double NET_Y = 5.70;
    private static final double PROCESSOR_X = 5.90;
    private static final double PROCESSOR_Y = 0.95;

    /** Rota experimental: 2 algas para NET e 1 para PROCESSOR. */
    public static Command algaeTres(
            TankSubsystem tank, SimIntake intake, SimElevator elevator, SimHand hand) {
        return Commands.sequence(
                tank.CMDresetPose(new Pose2d(7.2, 1.0, Rotation2d.fromDegrees(0))),
                coletarAlga(tank, intake, elevator, 2.200),
                pontuar(tank, elevator, hand, NET_X, NET_Y, 135),
                coletarAlga(tank, intake, elevator, 4.023),
                pontuar(tank, elevator, hand, NET_X, NET_Y, 135),
                coletarAlga(tank, intake, elevator, 5.858),
                pontuar(tank, elevator, hand, PROCESSOR_X, PROCESSOR_Y, 100))
                .finallyDo(tank::stop);
    }

    /** Aproxima pelo corredor lateral, liga o intake e espera a transferencia. */
    private static Command coletarAlga(
            TankSubsystem tank, SimIntake intake, SimElevator elevator, double y) {
        return Commands.sequence(
                elevator.CMDsetPosition(400, 3),
                Commands.waitUntil(() -> SimElevator.getRealPosition() > 210).withTimeout(4),
                // O intake fica ativo antes da aproximacao final para capturar a peça.
                intake.CMDintakeOn().withTimeout(4),
                tank.driveToPose(COLETA_X, y, 180),
                Commands.waitUntil(() -> tank.isFuelInsideRobot() || tank.isHandCollected())
                        .withTimeout(5),
                intake.CMDintakeOff().withTimeout(4),
                Commands.waitUntil(tank::isHandCollected).withTimeout(8));
    }

    /** So pontua quando uma alga foi efetivamente transferida para o Hand. */
    private static Command pontuar(
            TankSubsystem tank, SimElevator elevator, SimHand hand,
            double x, double y, double anguloLancamento) {
        return Commands.sequence(
                // Corredor inferior reduz travessias diretas pelo centro do campo;
                // NAO substitui uma trajetoria validada no PathPlanner.
                tank.driveToPose(COLETA_X, 1.0, 0),
                tank.driveToPose(x, 1.0, 0),
                tank.driveToPose(x, y, 90),
                Commands.either(
                        Commands.sequence(
                                elevator.CMDsetPosition(400, 3),
                                Commands.waitUntil(() -> SimElevator.getRealPosition() > 210)
                                        .withTimeout(4),
                                hand.CMDsetAngle(anguloLancamento, 3),
                                Commands.waitUntil(
                                        () -> Math.abs(SimHand.getRealAngle() - anguloLancamento) < 5)
                                        .withTimeout(4),
                                depositGamePiece(tank),
                                hand.CMDsetAngle(0, 3)),
                        Commands.none(),
                        tank::isHandCollected));
    }

    public static Command depositGamePiece(TankSubsystem tank) {
        return Commands.sequence(
                mSimHand.CMDsetVelocity(1),
                tank.depositFuelCommand(),
                Commands.waitSeconds(1),
                mSimHand.CMDsetVelocity(0));
    }

}
