package frc.robot.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;

public class IntakeFactory {
    public static Command prepareIntakeBlocking(RobotContainer robotContainer) {
        return Commands.parallel(
            Commands.runOnce(() -> robotContainer.getIntake().setPivotGoal(PivotGoal.DEPLOY)),
            Commands.waitUntil(robotContainer.getIntake()::pivotAtGoal)
                .andThen(Commands.startEnd(
                    () -> robotContainer.getIntake().setRollerGoal(RollerGoal.INTAKE),
                    () -> robotContainer.getIntake().setRollerGoal(RollerGoal.STOP)
                ))
        );
    }

    public static Command parkIntakeBlocking(RobotContainer robotContainer) {
        return Commands.parallel(
            Commands.runOnce(() -> robotContainer.getIntake().setPivotGoal(PivotGoal.PARKED))
                .alongWith(Commands.runOnce(
                    () -> robotContainer.getIntake().setRollerGoal(RollerGoal.IDLE)
                )),
            Commands.waitUntil(robotContainer.getIntake()::pivotAtGoal)
        );
    }
}
