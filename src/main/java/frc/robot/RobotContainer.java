
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

    // =========================================================
    // SUBSISTEMAS
    // =========================================================

    private final TankSubsystem tank = new TankSubsystem();

    private final CommandXboxController driver =
            new CommandXboxController(0);

    private final XboxController driverget =
            new XboxController(0);

    private final SimIntake mSimIntake =
            new SimIntake(20, 1.5, 3);

    private final SimElevator mSimElevator =
            new SimElevator(400, 1.5);

    private final SimHand mSimHand =
            new SimHand(-180, 180, 1.5, 3);

    // =========================================================
    // SELETOR DE AUTÔNOMO
    // =========================================================

    private final SendableChooser<Command> autoChooser =
            new SendableChooser<>();

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    private static final boolean DESATIVAR_CONTROLE_PARA_TESTE = false;

    private static final double DEADBAND_ROTACAO = 0.5;

    private static final double ELEVADOR_EM_CIMA = 200;

    // =========================================================
    // CONSTRUTOR
    // =========================================================

    public RobotContainer() {

        // Configura os comandos autônomos.
        Autos.configurar(
                tank,
                mSimIntake,
                mSimElevator,
                mSimHand
        );

        configurarAutonomos();

        configureBindings();

        // Para o robô quando estiver desabilitado.
        RobotModeTriggers.disabled().onTrue(
                Commands.runOnce(tank::stop)
                        .ignoringDisable(true)
        );

        // =====================================================
        // COMANDO PADRÃO DO TANK
        // =====================================================

        if (!DESATIVAR_CONTROLE_PARA_TESTE) {

            tank.setDefaultCommand(
                    tank.run(() -> {

                        for (int i = 0; i < 6; i++) {
                            Logger.recordOutput(
                                    "Controle/Axis" + i,
                                    driver.getRawAxis(i)
                            );
                        }

                        Logger.recordOutput(
                                "Controle/Conectado",
                                driverget.isConnected()
                        );

                        if (!driverget.isConnected()) {

                            Logger.recordOutput(
                                    "Controle/Ramo",
                                    "desconectado"
                            );

                            tank.stop();
                            return;
                        }

                        double leftX = driver.getLeftX();
                        double leftY = driver.getLeftY();

                        double joystickMagnitude =
                                Math.hypot(leftX, leftY);

                        if (joystickMagnitude > 0.1) {

                            double throttle =
                                    MathUtil.applyDeadband(
                                            driver.getRawAxis(3),
                                            0.08
                                    );

                            Logger.recordOutput(
                                    "Controle/Ramo",
                                    "joystick-angulo"
                            );

                            tank.driveWithAngleControl(
                                    getLeftJoystickAngle(),
                                    throttle
                            );

                        } else {

                            double forward = driver.getRawAxis(3);
                            double backward = driver.getRawAxis(2);

                            double rotation =
                                    MathUtil.applyDeadband(
                                            driver.getRightX(),
                                            DEADBAND_ROTACAO
                                    );

                            Logger.recordOutput(
                                    "Controle/Ramo",
                                    "tank-normal"
                            );

                            Logger.recordOutput(
                                    "Controle/Rotacao",
                                    rotation
                            );

                            tank.drive(
                                    forward,
                                    -backward,
                                    -rotation
                            );
                        }
                    })
            );
        }
    }

    // =========================================================
    // AUTÔNOMOS
    // =========================================================

    private void configurarAutonomos() {

        autoChooser.setDefaultOption(
                "Nenhum",
                Commands.none()
        );

        autoChooser.addOption(
                "AULA - Seguir PathPlanner",
                Autos.examplePath(tank)
        );

        autoChooser.addOption(
                "AULA - Auto Completo",
                Autos.autoCompleto(tank)
        );

        autoChooser.addOption(
                "Quadrado",
                Autos.quadrado(tank)
        );

        autoChooser.addOption(
                "Reto",
                Autos.reto(tank)
        );

        autoChooser.addOption(
                "Auto ZigZag",
                Autos.autoZigZag(tank)
        );

        autoChooser.addOption(
                "Alga (3)",
                Autos.algaeTres(
                        tank,
                        mSimIntake,
                        mSimElevator,
                        mSimHand
                )
        );

        autoChooser.addOption(
                "Navegar para pose alvo",
                Autos.navegarParaPoseAlvo(tank)
        );

        SmartDashboard.putData(
                "Auto Selector",
                autoChooser
        );
    }

    // =========================================================
    // CONTROLES
    // =========================================================

    private void configureBindings() {

        // Reinicia a pose pelo botão START.
        driver.start().onTrue(
                tank.runOnce(
                        () -> tank.resetPose(
                                new Pose2d(
                                        2.0,
                                        4.0,
                                        new Rotation2d()
                                )
                        )
                )
        );

        // =====================================================
        // ELEVADOR
        // =====================================================

        driver.povUp().onTrue(
                mSimElevator.CMDsetPosition(400, 10)
        );

        driver.povDown().onTrue(
                Commands.sequence(

                        mSimHand.CMDsetAngle(0, 3),

                        Commands.waitUntil(
                                () -> Math.abs(
                                        SimHand.getRealAngle()
                                ) < 5
                        ),

                        mSimElevator.CMDsetPosition(0, 3)
                )
        );

        // =====================================================
        // GARRA
        // =====================================================

        driver.povLeft().onTrue(
                Commands.runOnce(
                        () -> mSimHand.setAngle(-45, 3)
                ).onlyIf(this::elevadorEmCima)
        );

        driver.povRight().onTrue(
                Commands.runOnce(
                        () -> mSimHand.setAngle(45, 3)
                ).onlyIf(this::elevadorEmCima)
        );

        driver.a().onTrue(
                Commands.runOnce(
                        () -> mSimHand.setAngle(135, 3)
                ).onlyIf(this::elevadorEmCima)
        );

        driver.b().onTrue(
                Commands.runOnce(
                        () -> mSimHand.setAngle(-135, 3)
                ).onlyIf(this::elevadorEmCima)
        );

        driver.back().onTrue(
                Commands.runOnce(
                        () -> mSimHand.setAngle(0, 3)
                )
        );

        // =====================================================
        // COLETA MANUAL
        // =====================================================

        coletarGround();

        // =====================================================
        // DEPÓSITO MANUAL
        // =====================================================

        driver.y().onTrue(deposit());

        // =====================================================
        // NAVEGAÇÃO MANUAL
        // =====================================================

        driver.rightBumper().whileTrue(
                tank.driveToPose(7.8, 4.0, 60)
        );
    }

    // =========================================================
    // ELEVADOR
    // =========================================================

    private boolean elevadorEmCima() {

        return SimElevator.getRealPosition()
                >= ELEVADOR_EM_CIMA;
    }

    // =========================================================
    // COLETA MANUAL NO CHÃO
    // =========================================================

    private void coletarGround() {

        Trigger podeColetar = new Trigger(
                () ->
                        !tank.isFuelInsideRobot()
                                && SimElevator.getRealPosition() >= 200
        );

        Trigger coletando =
                driver.leftBumper().and(podeColetar);

        coletando.whileTrue(
                mSimIntake.CMDintakeOn()
        );

        coletando.whileFalse(
                mSimIntake.CMDintakeOff()
        );
    }

    // =========================================================
    // COMANDO AUTÔNOMO
    // =========================================================

    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }

    // =========================================================
    // ÂNGULO DO JOYSTICK
    // =========================================================

    public double getLeftJoystickAngle() {

        double x = -driverget.getLeftX();
        double y = -driverget.getLeftY();

        if (Math.hypot(x, y) < 0.1) {
            return 0.0;
        }

        return Math.toDegrees(
                Math.atan2(x, y)
        );
    }

    // =========================================================
    // DEPÓSITO MANUAL
    // =========================================================

    private Command deposit() {

        return Commands.sequence(

                Commands.runOnce(
                        () -> mSimHand.setVelocity(1)
                ),

                tank.depositFuelCommand(),

                Commands.waitSeconds(1),

                Commands.runOnce(
                        () -> mSimHand.setVelocity(0)
                )
        );
    }
}
