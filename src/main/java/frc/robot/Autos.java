
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

    private static final Pose2d POSE_INICIAL =
            new Pose2d(7.118, 1.402, Rotation2d.fromDegrees(0));

    private static final double COLETA_X = 1.90;
    private static final double COLETA_X_SEGUNDA_BOLA = 1.78;
    private static final double COLETA_X_TERCEIRA_BOLA = 1.75;

    private static final double PATH_START_X = 7.12725;
    private static final double PATH_START_Y = 1.178;
    private static final double PATH_START_ANGLE = -178.1118;

    // Corredor lateral
    private static final double LATERAL_X = 1.90;
    private static final double CORREDOR_X = 2.70;
    private static final double LINHA_NET_Y = 6.50;

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

        NamedCommands.registerCommand("elevadorCima", subirElevador());
        NamedCommands.registerCommand("intakeOn", intake.CMDintakeOn());
        NamedCommands.registerCommand("intakeOff", intake.CMDintakeOff());
        NamedCommands.registerCommand("depositar", depositGamePiece(tank));
    }

    // =========================================================
    // ELEVADOR
    // =========================================================

    private static Command subirElevador() {
        return Commands.sequence(
                mSimElevator.CMDsetPosition(400, 3),

                // Só continua quando o elevador realmente subir.
                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() >= 390
                )
        );
    }

    private static Command abaixarElevador() {
        return Commands.sequence(
                mSimElevator.CMDsetPosition(0, 3),

                // Só continua quando o elevador realmente abaixar.
                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() <= 10
                )
        );
    }

    // =========================================================
    // TRANSFERÊNCIA DA BOLA PARA A HAND
    // =========================================================

    private static Command transferirParaGarra(TankSubsystem tank) {

        return Commands.sequence(

                abaixarElevador(),

                mSimHand.CMDsetAngle(0, 3),

                Commands.waitUntil(
                        () -> Math.abs(SimHand.getRealAngle()) < 5
                ),

                // Espera a Fuel armazenada alcançar a garra.
                Commands.waitUntil(tank::isHandCollected),

                subirElevador()
        );
    }

    // =========================================================
    // COLETA DE UMA BOLA
    // =========================================================

    private static Command coletarBola(
            TankSubsystem tank,
            double y,
            double xColeta) {

        return Commands.sequence(

                subirElevador(),

                tank.driveToPose(
                        CORREDOR_X,
                        y,
                        180
                ),

                mSimIntake.CMDintakeOn(),

                tank.driveToPoint(
                        xColeta,
                        y
                ),

                // Não avança enquanto a bola não estiver armazenada.
                Commands.waitUntil(tank::isFuelInsideRobot),

                mSimIntake.CMDintakeOff()
        );
    }

    // =========================================================
    // COLETAR A PRÓXIMA BOLA
    // =========================================================

    private static Command coletarProximaBola(
            TankSubsystem tank,
            double yAnterior,
            double yProxima,
            double xColeta) {

        return Commands.sequence(

                subirElevador(),

                tank.driveToPose(
                        CORREDOR_X,
                        yAnterior,
                        180
                ),

                tank.driveToPose(
                        CORREDOR_X,
                        yProxima,
                        180
                ),

                mSimIntake.CMDintakeOn(),

                tank.driveToPoint(
                        xColeta,
                        yProxima
                ),

                Commands.waitUntil(tank::isFuelInsideRobot),

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
                    );

            Command seguirPath =
                    AutoBuilder.followPath(path);

            return Commands.sequence(

                    tank.CMDresetPose(POSE_INICIAL),

                    coletarBola(
                            tank,
                            2.200,
                            COLETA_X
                    ),

                    transferirParaGarra(tank),

                    Commands.either(

                            Commands.sequence(

                                    irParaInicioDoPath,

                                    seguirPath,

                                    prepararELancar(
                                            tank,
                                            mSimElevator,
                                            mSimHand,
                                            60
                                    )
                            ),

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

                coletarBola(
                        tank,
                        2.200,
                        COLETA_X
                )

        ).finallyDo(tank::stop);
    }

    public static Command navegarParaPoseAlvo(TankSubsystem tank) {

        return Commands.sequence(

                tank.driveToPose(7.8, 4.0, 60),

                Commands.runOnce(tank::stop)
        );
    }

    // =========================================================
    // VOLTAR DA COLETA PARA A NET
    // =========================================================

    private static Command voltarDaColetaParaNet(
            TankSubsystem tank,
            double yColeta,
            double xNet,
            double yNet,
            double direcao) {

        return Commands.sequence(

                tank.driveToPose(
                        CORREDOR_X,
                        yColeta,
                        180
                ),

                tank.driveToPose(
                        LATERAL_X,
                        LINHA_NET_Y,
                        180
                ),

                tank.driveToPose(
                        xNet,
                        yNet,
                        direcao
                )
        );
    }

    // =========================================================
    // DESLOCAMENTO PELA LATERAL ATÉ A NET
    // =========================================================

    private static Command irParaNet(
            TankSubsystem tank,
            double x,
            double y,
            double direcao) {

        return Commands.sequence(

                tank.driveToPose(
                        LATERAL_X,
                        LINHA_NET_Y,
                        180
                ),

                tank.driveToPose(
                        x,
                        y,
                        direcao
                )
        );
    }

    // =========================================================
    // CONTORNAR O REEF PARA IR AO PROCESSOR
    // =========================================================

    private static Command contornarReef(
            TankSubsystem tank,
            double yColeta,
            double xCorredor) {

        return Commands.sequence(

                tank.driveToPose(
                        CORREDOR_X,
                        yColeta,
                        180
                ),

                tank.driveToPose(
                        CORREDOR_X,
                        1.90,
                        180
                ),

                tank.driveToPose(
                        xCorredor,
                        1.90,
                        180
                ),

                tank.driveToPose(
                        6.06,
                        1.20,
                        180
                )
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

        mSimIntake = intake;
        mSimElevator = elevator;
        mSimHand = hand;

        return Commands.sequence(

                tank.CMDresetPose(POSE_INICIAL),

                // =============================================
                // BOLA 1
                // =============================================

                coletarBola(
                        tank,
                        2.200,
                        COLETA_X
                ),

                transferirParaGarra(tank),

                // =============================================
                // BOLA 2
                // A primeira continua guardada na garra.
                // A segunda fica armazenada dentro do robô.
                // =============================================

                coletarProximaBola(
                        tank,
                        2.200,
                        4.023,
                        COLETA_X_SEGUNDA_BOLA
                ),

                // =============================================
                // PONTUAR BOLA 1 NA NET
                // =============================================

                Commands.either(

                        Commands.sequence(

                                voltarDaColetaParaNet(
                                        tank,
                                        4.023,
                                        7.45,
                                        5.70,
                                        -90
                                ),

                                pontuarDepoisDeNavegar(
                                        tank,
                                        elevator,
                                        hand,
                                        160
                                )
                        ),

                        Commands.none(),

                        tank::isHandCollected
                ),

                // =============================================
                // TRANSFERIR E PONTUAR BOLA 2
                // =============================================

                transferirParaGarra(tank),

                Commands.either(

                        pontuarDepoisDeNavegar(
                                tank,
                                elevator,
                                hand,
                                160
                        ),

                        Commands.none(),

                        tank::isHandCollected
                ),

                // =============================================
                // BOLA 3
                // =============================================

                tank.driveToPose(
                        CORREDOR_X,
                        5.70,
                        180
                ),

                coletarProximaBola(
                        tank,
                        5.70,
                        5.858,
                        COLETA_X_TERCEIRA_BOLA
                ),

                transferirParaGarra(tank),

                // =============================================
                // PONTUAR BOLA 3 NO PROCESSOR AZUL
                // =============================================

                Commands.either(

                        Commands.sequence(

                                contornarReef(
                                        tank,
                                        5.858,
                                        1.90
                                ),

                                pontuarDepoisDeNavegar(
                                        tank,
                                        elevator,
                                        hand,
                                        57
                                )
                        ),

                        Commands.none(),

                        tank::isHandCollected
                )

        ).finallyDo(tank::stop);
    }

    // =========================================================
    // PREPARAR E LANÇAR
    // =========================================================

    private static Command pontuarDepoisDeNavegar(
            TankSubsystem tank,
            SimElevator elevator,
            SimHand hand,
            double angulo) {

        return prepararELancar(
                tank,
                elevator,
                hand,
                angulo
        );
    }

    private static Command prepararELancar(
            TankSubsystem tank,
            SimElevator elevator,
            SimHand hand,
            double angulo) {

        return Commands.sequence(

                // Só inicia se ainda houver bola na garra.
                Commands.waitUntil(tank::isHandCollected),

                elevator.CMDsetPosition(400, 3),

                Commands.waitUntil(
                        () -> SimElevator.getRealPosition() >= 390
                ),

                hand.CMDsetAngle(angulo, 3),

                Commands.waitUntil(
                        () -> Math.abs(
                                SimHand.getRealAngle() - angulo
                        ) < 5
                ),

                // Confere novamente antes de lançar.
                Commands.either(
                        depositGamePiece(tank),
                        Commands.none(),
                        tank::isHandCollected
                ),

                hand.CMDsetAngle(0, 3)
        );
    }

    // =========================================================
    // PONTUAÇÃO COM NAVEGAÇÃO
    // =========================================================

    private static Command pontuar(
            TankSubsystem tank,
            SimElevator elevator,
            SimHand hand,
            double x,
            double y,
            double direcao,
            double angulo) {

        Command navegar;

        if (y > 4.0) {

            navegar = irParaNet(
                    tank,
                    x,
                    y,
                    direcao
            );

        } else {

            navegar = tank.driveToPose(
                    x,
                    y,
                    direcao
            );
        }

        return Commands.sequence(

                navegar,

                pontuarDepoisDeNavegar(
                        tank,
                        elevator,
                        hand,
                        angulo
                )
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
