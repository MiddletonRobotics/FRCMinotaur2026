// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSimulation;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;

public class RobotContainer {
  private final Consumer<WeightedPoseEstimate> visionEstimateConsumer = new Consumer<WeightedPoseEstimate>() {
    @Override
    public void accept(WeightedPoseEstimate estimate) {
        drivetrain.addVisionMeasurement(estimate);
    }
  };

  private final RobotState robotState = new RobotState(visionEstimateConsumer);
  private Drivetrain drivetrain;
  private Intake intake;

  private CommandSimulatedXboxController driverController = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
  private CommandSimulatedXboxController operatorController = new CommandSimulatedXboxController(ControllerConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoRegistry;

  private Drivetrain buildDrivetrain() {
    if(Robot.isSimulation()) {
      return new Drivetrain(
        new DrivetrainIOSimulation(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), SimulationTunerConstants.kFrontLeft, SimulationTunerConstants.kFrontRight, SimulationTunerConstants.kBackLeft, SimulationTunerConstants.kBackRight), 
        robotState
      );
    } else {
      return new Drivetrain(
        new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), CompetitionTunerConstants.kFrontLeft, CompetitionTunerConstants.kFrontRight, CompetitionTunerConstants.kBackLeft, CompetitionTunerConstants.kBackRight), 
        robotState
      );
    }
  }

  private Intake buildIntake() {
    if(Robot.isSimulation()) {
      return new Intake(
        new IntakeIOSimulation()
        //robotState
      );
    } else {
      return new Intake(
        new IntakeIOHardware(IntakeConstants.kRollerMotor, IntakeConstants.kPivotMotor, IntakeConstants.kPivotAbsoluteEncoder)
        //robotState
      );
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Intake getIntake() {
    return intake;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();
    intake = buildIntake();

    intake.setBrakeMode(driverController.back().and(() -> !DriverStation.isEnabled()));

    autoRegistry = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autoRegistry.addOption("Drivetrain Translation Dynamic Forward", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Dynamic Reverse", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Forward", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Reverse", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      driverController::getLeftY, 
      driverController::getLeftX, 
      driverController::getRightX, 
      true
    ));

    driverController
      .leftTrigger()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)))
      .whileTrue(Commands.runEnd(
        () -> intake.setRollerGoal(RollerGoal.INTAKE), 
        () -> intake.setRollerGoal(RollerGoal.STOP), 
        intake
      ));

    driverController
      .leftBumper()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.PARKED)))
      .onTrue(Commands.runEnd(
        () -> intake.setRollerGoal(RollerGoal.INTAKE), 
        () -> intake.setRollerGoal(RollerGoal.IDLE), 
        intake
      ).withTimeout(1));
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
