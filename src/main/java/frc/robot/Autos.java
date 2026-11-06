package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class Autos {

    private static SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
    private static SimElevator mSimElevator = new SimElevator(400, 1.5);
    private static SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);

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

}
