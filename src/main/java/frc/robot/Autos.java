
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

    private static SimIntake mSimIntake;
    private static SimElevator mSimElevator;
    private static SimHand mSimHand;

    private static final String EXAMPLE_PATH = "AULA";

    // Posição inicial assumida do robô
    private static final Pose2d POSE_INICIAL =
            new Pose2d(7.2, 1.0, Rotation2d.fromDegrees(0));

    // Posições das bolas no FuelSimulation
    private static final double COLETA_X = 1.90;

    // Posição inicial do caminho AULA
    private static final double PATH_START_X = 7.12725;
    private static final double PATH_START_Y = 1.178;
    private static final double PATH_START_ANGLE = -178.1118;

    private static final double TIMEOUT_NAVEGACAO = 8.0;

    private Autos() {
    }

    // =========================================================
    // CONFIGURAÇÃO
    // =========================================================

    public static void configurar(
            TankSubsystem tank,
            SimIntake intake,
            SimElevator elevator,
            SimHand hand) {

        mSimIntake = intake;
        mSimElevator = elevator;
        mSimHand = hand;

        NamedCommands.registerCommand(
                "elevadorCima",
                subirElevador()
        );

        NamedCommands.registerCommand(
                "intakeOn",
                intake.CMDintakeOn()
        );

        NamedCommands.registerCommand(
                "intakeOff",
                intake.CMDintakeOff()
        );

        NamedCommands.registerCommand(
                "depositar",
                depositGamePiece(tank)
        );
    }

    // =========================================================
    // ELEVADOR
    // =========================================================

    private static Command subirElevador() {
        return Commands.sequence(
                mSimElevator.CMDsetPosition(400, 10),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() >= 200
                )
        );
    }

    private static Command abaixarElevador() {
        return Commands.sequence(
                mSimElevator.CMDsetPosition(0, 3),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() <= 10
                )
        );
    }

    // =========================================================
    // TRANSFERÊNCIA DA BOLA PARA A GARRA
    // =========================================================

    private static Command transferirParaGarra() {

        return Commands.sequence(

                // Sobe o elevador
                mSimElevator.CMDsetPosition(400, 3),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() > 390
                ),

                // Centraliza a garra
                mSimHand.CMDsetAngle(0, 3),

                Commands.waitUntil(
                        () -> Math.abs(SimHand.getRealAngle()) < 5
                ),

                // Abaixa o elevador para aproximar a garra da bola
                mSimElevator.CMDsetPosition(0, 3),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() < 10
                ),

                // Aciona os roletes para capturar
                mSimHand.CMDsetVelocity(-1),

                Commands.waitSeconds(1),

                mSimHand.CMDsetVelocity(0),

                // Dá tempo para a simulação registrar a captura
                Commands.waitUntil(
                        () -> true
                ).withTimeout(0.2)
        );
    }

    // =========================================================
    // COLETA DE UMA BOLA
    // =========================================================

    private static Command coletarBola(
            TankSubsystem tank,
            double y) {

        return Commands.sequence(

                // Prepara o elevador
                subirElevador(),

                // Vai para perto da bola, virado para ela
                tank.driveToPose(2.70, y, 180)
                        .withTimeout(TIMEOUT_NAVEGACAO),

                // Abre e liga o intake
                mSimIntake.CMDintakeOn(),

                // Avança até a posição de coleta
                tank.driveToPoint(COLETA_X, y)
                        .withTimeout(TIMEOUT_NAVEGACAO),

                // Aguarda a bola entrar no robô
                Commands.waitUntil(
                        tank::isFuelInsideRobot
                ).withTimeout(5),

                // Desliga e recolhe o intake
                mSimIntake.CMDintakeOff()
        );
    }

    // =========================================================
    // CAMINHO AULA
    // =========================================================

    public static Command examplePath(TankSubsystem tank) {

        try {
            PathPlannerPath path =
                    PathPlannerPath.fromPathFile(EXAMPLE_PATH);

            return Commands.sequence(

                    tank.CMDresetPose(
                            path.getStartingDifferentialPose()
                    ),

                    AutoBuilder.followPath(path)

            ).finallyDo(tank::stop);

        } catch (Exception e) {

            DriverStation.reportError(
                    "ERRO AO CARREGAR PATH: " + e.getMessage(),
                    e.getStackTrace()
            );

            return Commands.none();
        }
    }

    // =========================================================
    // AUTÔNOMO COMPLETO
    // =========================================================

    public static Command autoCompleto(TankSubsystem tank) {

        try {
            PathPlannerPath path =
                    PathPlannerPath.fromPathFile(EXAMPLE_PATH);

            Command irParaInicioDoPath =
                    tank.driveToPose(
                            PATH_START_X,
                            PATH_START_Y,
                            PATH_START_ANGLE
                    ).withTimeout(TIMEOUT_NAVEGACAO);

            Command seguirPath =
                    AutoBuilder.followPath(path);

            return Commands.sequence(

                    // 1. Define a pose inicial
                    tank.CMDresetPose(POSE_INICIAL),

                    // 2. Coleta a primeira bola
                    coletarBola(tank, 2.200),

                    // 3. Transfere para a garra
                    transferirParaGarra(),

                    // 4. Só continua se a garra realmente capturou
                    Commands.either(

                            Commands.sequence(

                                    // 5. Vai até o início do caminho AULA
                                    irParaInicioDoPath,

                                    // 6. Segue o PathPlanner
                                    seguirPath,

                                    // 7. Ajusta o ângulo da garra
                                    mSimHand.CMDsetAngle(60, 3),

                                    Commands.waitUntil(
                                            () -> Math.abs(
                                                    SimHand.getRealAngle() - 60
                                            ) < 5
                                    ).withTimeout(4),

                                    // 8. Deposita
                                    depositGamePiece(tank),

                                    // 9. Retorna a garra ao centro
                                    mSimHand.CMDsetAngle(0, 3)
                            ),

                            // Se não capturou, não tenta depositar
                            Commands.none(),

                            tank::isHandCollected
                    )

            ).finallyDo(tank::stop);

        } catch (Exception e) {

            DriverStation.reportError(
                    "ERRO AO CARREGAR AUTO COMPLETO: " + e.getMessage(),
                    e.getStackTrace()
            );

            return Commands.none();
        }
    }

    // =========================================================
    // AUTÔNOMOS DE TESTE
    // =========================================================

    public static Command quadrado(TankSubsystem tank) {

        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(4.0, 2.0, 90),
                tank.driveToPose(4.0, 5.0, 180),
                tank.driveToPose(2.0, 5.0, -90)
        ).finallyDo(tank::stop);
    }

    public static Command reto(TankSubsystem tank) {

        return Commands.sequence(
                tank.driveToPose(2.0, 2.0, 0),
                tank.driveToPose(5.0, 2.0, 0)
        ).finallyDo(tank::stop);
    }

    public static Command autoZigZag(TankSubsystem tank) {

        return Commands.sequence(
                tank.CMDresetPose(POSE_INICIAL),
                coletarBola(tank, 2.200)
        ).finallyDo(tank::stop);
    }

    public static Command navegarParaPoseAlvo(TankSubsystem tank) {

        return Commands.sequence(
                tank.driveToPose(7.8, 4.0, 60),
                Commands.runOnce(tank::stop)
        );
    }

    // =========================================================
    // AUTÔNOMO DAS TRÊS BOLAS
    // =========================================================

    public static Command algaeTres(
            TankSubsystem tank,
            SimIntake intake,
            SimElevator elevator,
            SimHand hand) {

        return Commands.sequence(

                tank.CMDresetPose(POSE_INICIAL),

                coletarBola(tank, 2.200),
                transferirParaGarra(),

                Commands.either(
                        Commands.sequence(
                                pontuar(tank, elevator, hand,
                                        7.45, 5.70, 135)
                        ),
                        Commands.none(),
                        tank::isHandCollected
                ),

                coletarBola(tank, 4.023),
                transferirParaGarra(),

                Commands.either(
                        Commands.sequence(
                                pontuar(tank, elevator, hand,
                                        7.45, 5.70, 135)
                        ),
                        Commands.none(),
                        tank::isHandCollected
                ),

                coletarBola(tank, 5.858),
                transferirParaGarra(),

                Commands.either(
                        Commands.sequence(
                                pontuar(tank, elevator, hand,
                                        5.90, 0.95, 100)
                        ),
                        Commands.none(),
                        tank::isHandCollected
                )

        ).finallyDo(tank::stop);
    }

    // =========================================================
    // PONTUAÇÃO
    // =========================================================

    private static Command pontuar(
            TankSubsystem tank,
            SimElevator elevator,
            SimHand hand,
            double x,
            double y,
            double angulo) {

        return Commands.sequence(

                tank.driveToPose(x, y, 90)
                        .withTimeout(TIMEOUT_NAVEGACAO),

                elevator.CMDsetPosition(400, 3),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() > 210
                ).withTimeout(4),

                hand.CMDsetAngle(angulo, 3),

                Commands.waitUntil(
                        () -> Math.abs(
                                SimHand.getRealAngle() - angulo
                        ) < 5
                ).withTimeout(4),

                Commands.either(
                        depositGamePiece(tank),
                        Commands.none(),
                        tank::isHandCollected
                ),

                hand.CMDsetAngle(0, 3)
        );
    }

    // =========================================================
    // DEPÓSITO
    // =========================================================

    public static Command depositGamePiece(TankSubsystem tank) {

        return Commands.sequence(

                mSimHand.CMDsetVelocity(1),

                tank.depositFuelCommand(),

                Commands.waitSeconds(1),

                mSimHand.CMDsetVelocity(0)
        );
    }
}
