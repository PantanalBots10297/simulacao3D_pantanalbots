// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

  private final TankSubsystem tank = new TankSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final XboxController driverget = new XboxController(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank));
    // autoChooser.addOption("Algae_3", Autos.Algae_3(tank));

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      // Joystick esquerdo sendo utilizado
      if (joystickMagnitude > 0.08) {

        tank.driveWithAngleControl(getLeftJoystickAngle(), driver.getRawAxis(3));

        // Tank normal
      } else {

        double forward = driver.getRawAxis(3);
        double backward = driver.getRawAxis(2);
        double rotation = driver.getRightX();

        tank.drive(forward, -backward, -rotation);
      }
    }));

    // =========================================================
    // DEBUG TEMPORÁRIO (pode apagar depois que o intake funcionar)
    // Veja em SmartDashboard/Debug no AdvantageScope.
    // =========================================================
    new Trigger(() -> true).whileTrue(Commands.run(() -> {
      SmartDashboard.putBoolean("Debug/LB apertado", driver.leftBumper().getAsBoolean());
      SmartDashboard.putBoolean("Debug/Fuel dentro", tank.isFuelInsideRobot());
      SmartDashboard.putNumber("Debug/Elevador", SimElevator.getRealPosition());
    }).ignoringDisable(true));
  }

  private void configureBindings() {

    driver.start().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10));
    // Descer com segurança: primeiro o braço volta a 0° (centro), espera chegar,
    // e só então o elevador desce. Assim o braço não desce virado e não bate nas caixas.
    driver.povDown().onTrue(Commands.sequence(
        mSimHand.CMDsetAngle(0, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5),
        mSimElevator.CMDsetPosition(0, 3)));

    // Proteção: o braço só gira para os lados com o elevador em cima.
    // Com o elevador embaixo, esses botões não fazem nada (o braço bateria nas caixas).
    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)).onlyIf(this::elevadorEmCima));
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)).onlyIf(this::elevadorEmCima));
    driver.a().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)).onlyIf(this::elevadorEmCima));
    driver.b().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)).onlyIf(this::elevadorEmCima));

    driver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));

    /*
     * Positivo = Depositar
     * Negativo = Coletar
     */
    coletar_Ground();
    autoHand();
    // deposit_Pose();

    driver.y().onTrue(deposit());

    // Atividade 2 (prof. Djair) – Desafio 2: segurando o RB, o robô vai sozinho até
    // X = 7,8 m, Y = 4,0 m e vira para 60°, de onde estiver.
    // whileTrue: ao soltar o botão o comando é cancelado e o piloto volta a dirigir.
    driver.rightBumper().whileTrue(tank.driveToPose(7.8, 4.0, 60));

  }

  /**
   * Intake no L1. Só funciona enquanto não há peça dentro do robô E o elevador
   * está acima de 200 (proteção: o braço não bate no intake).
   * A condição é checada continuamente (Trigger), não só ao apertar.
   */
  /** Acima desta altura do elevador (0 a 400), o braço pode girar sem bater nas caixas. */
  private static final double ELEVADOR_EM_CIMA = 200;

  /** O elevador está alto o bastante para o braço girar? */
  boolean elevadorEmCima() {
    return SimElevator.getRealPosition() >= ELEVADOR_EM_CIMA;
  }

  void coletar_Ground() {

    Trigger podeColetar = new Trigger(
        () -> !tank.isFuelInsideRobot() && SimElevator.getRealPosition() >= 200);

    Trigger coletando = driver.leftBumper().and(podeColetar);

    coletando.whileTrue(mSimIntake.CMDintakeOn());
    coletando.whileFalse(mSimIntake.CMDintakeOff());
  }

  void autoHand() {

    BooleanSupplier autoHandOk = () -> tank.isFuelInsideRobot() && !tank.isHandCollected();

    onCommandCondition(autoHandOk, Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> SimElevator.getRealPosition() > 390),
        mSimHand.CMDsetAngle(0, 3),
        Commands.waitUntil(() -> Math.abs(SimHand.getRealAngle()) < 5),
        mSimElevator.CMDsetPosition(0, 3),
        mSimHand.CMDsetVelocity(-1),
        Commands.waitSeconds(1),
        mSimHand.CMDsetVelocity(0)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public double getLeftJoystickAngle() {

    double x = -driverget.getLeftX();
    double y = -driverget.getLeftY();

    if (Math.hypot(x, y) < 0.1) {
      return 0.0;
    }

    return Math.toDegrees(Math.atan2(x, y));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * 
   * @param null
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * 
   * @param null
   */
  private void whileCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(command);
  }

  Command deposit() {
    return Commands.sequence(
        Commands.runOnce(() -> mSimHand.setVelocity(1)),
        tank.depositFuelCommand(),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> mSimHand.setVelocity(0)));
  }

}