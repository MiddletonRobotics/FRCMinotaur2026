// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Consumer;

import org.dyn4j.collision.narrowphase.FallbackCondition;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import choreo.trajectory.EventMarker;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.agitator.AgitatorIOHardware;
import frc.robot.subsystems.agitator.Agitator.AgitatorGoal;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.HangIOHardware;
import frc.robot.subsystems.hang.Hang.HangPivotGoal;
import frc.robot.subsystems.hang.Hang.WinderGoal;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOHardware;
import frc.robot.subsystems.hood.HoodIOSimulation;
import frc.robot.subsystems.hood.Hood.HoodGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOCTRE;
import frc.robot.subsystems.indexer.IndexerIOHardware;
import frc.robot.subsystems.indexer.Indexer.Goal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSimulation;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterGoal;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSimulation;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.TowerIO;
import frc.robot.subsystems.tower.TowerIOHardware;
import frc.robot.subsystems.tower.Tower.TowerGoal;

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
  private Indexer indexer;
  private Tower tower;
  private Shooter shooter;
  private Hood hood;
  private Agitator agitator;

  //TODO: Check the controller axis inputs and ensure that they align with the assigned values for an Xbox Elite Controller
  private CommandSimulatedXboxController driverController = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
  private CommandSimulatedXboxController operatorController = new CommandSimulatedXboxController(ControllerConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoRegistry;

  private ShooterGoal selectedShooterRPM = ShooterGoal.STOP;

  private Drivetrain buildDrivetrain() {
    if(Robot.isSimulation()) {
      return new Drivetrain(
        new DrivetrainIOSimulation(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), SimulationTunerConstants.kFrontLeft, SimulationTunerConstants.kFrontRight, SimulationTunerConstants.kBackLeft, SimulationTunerConstants.kBackRight), 
        robotState
      );
    } else {
      return new Drivetrain(
        new DrivetrainIOHardware(robotState, DrivetrainConstants.kDrivetrain.getDriveTrainConstants(), CompetitionTunerConstants.FrontLeft, CompetitionTunerConstants.FrontRight, CompetitionTunerConstants.BackLeft, CompetitionTunerConstants.BackRight), 
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
       new IntakeIOHardware()
        //robotState
     );
   }
  }

  private Indexer buildIndexer() {
    if(Robot.isSimulation()) {
      return new Indexer(
        new IndexerIOHardware()
      );
    } else {
      return new Indexer( 
        new IndexerIOCTRE()
      );
    }
  }

  private Tower buildTower() {
    if(Robot.isSimulation()) {
      return new Tower(
        new TowerIOHardware()
      );
    } else {
      return new Tower(
        new TowerIOHardware()
      );
    }
  }

  private Shooter buildShooter() {
    if(Robot.isSimulation()) {
      return new Shooter(
        new ShooterIOSimulation()
      );
    } else {
      return new Shooter(
        new ShooterIOHardware()
      );
    }
  }

  private Hood buildHood() {
    if(Robot.isSimulation()) {
      return new Hood(
        new HoodIOSimulation()
      );
    } else {
      return new Hood(
        new HoodIOHardware()
      );
    }
  }

  private Agitator getAgitator() {
    if(Robot.isSimulation()) {
      return new Agitator(
        new AgitatorIOHardware()
      );
    } else {
      return new Agitator(
        new AgitatorIOHardware()
      );
    }
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }

  public Intake getIntake() {
   return intake;
  }

  public Indexer getIndexer() {
    return indexer;
  }

  public Tower getTower() {
    return tower;
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Hood getHood() {
    return hood;
  }

  public RobotContainer() {
    drivetrain = buildDrivetrain();
    intake = buildIntake();
    indexer = buildIndexer();
    tower = buildTower();
    shooter = buildShooter();
    hood = buildHood();
    agitator = getAgitator();

    tower.setBrakeMode(() -> false);

    autoRegistry = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autoRegistry.addOption("Drivetrain Translation Dynamic Forward", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Dynamic Reverse", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Forward", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Reverse", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    
    NamedCommands.registerCommand("DeployIntake", Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)));
    NamedCommands.registerCommand("StartIntakeRoller", Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.INTAKE)));
    NamedCommands.registerCommand("Hood Minimum Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MINIMUM)));
    NamedCommands.registerCommand("Hood Minimum-Midpoint Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MIDPOINT)));
    NamedCommands.registerCommand("Hood Midpoint Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MIDPOINT)));
    NamedCommands.registerCommand("Hood Maximum Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MAXIMUM)));
    NamedCommands.registerCommand(
      "Shooter Open Loop Close-Mid", 
      Commands.runOnce(() -> shooter.setGoal(ShooterGoal.CLOSE))
        .andThen(Commands.runOnce(() -> shooter.setManualVoltage(4.3)))
    );
    NamedCommands.registerCommand(
      "Shooter Open Loop Close", 
      Commands.runOnce(() -> shooter.setGoal(ShooterGoal.CLOSE))
        .andThen(Commands.runOnce(() -> shooter.setManualVoltage(4.5)))
    );
    NamedCommands.registerCommand(
      "Shooter Open Loop Mid", 
      Commands.runOnce(() -> shooter.setGoal(ShooterGoal.CLOSE))
        .andThen(Commands.runOnce(() -> shooter.setManualVoltage(5.4)))
    );
    NamedCommands.registerCommand("Index to Shooter", 
      Commands.runOnce(() -> {
        indexer.setGoal(Goal.INTAKE);
        tower.setGoal(TowerGoal.INTAKE);
      }));

    new EventTrigger("Shooter Open Loop Close")
      .onTrue(Commands.runOnce(() -> shooter.setGoal(ShooterGoal.CLOSE))
        .andThen(Commands.runOnce(() -> shooter.setManualVoltage(4.6))));

    NamedCommands.registerCommand("Stop Index to Shooter", 
    Commands.runOnce(() -> {
      indexer.setGoal(Goal.STOP); 
      tower.setGoal(TowerGoal.STOP);
    }));

    NamedCommands.registerCommand("Stop Intake Rollers", 
      Commands.runOnce(() -> 
        intake.setRollerGoal(RollerGoal.STOP))
    );

    NamedCommands.registerCommand("Idle Shooter", 
      Commands.runOnce(() -> 
        shooter.setGoal(ShooterGoal.IDLE))
    );

    NamedCommands.registerCommand("StopAndRetractIntakeHalf", 
      Commands.runOnce(() -> 
          intake.setPivotGoal(PivotGoal.FEED))
            .alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP))));

    new EventTrigger("DeployIntake")
    .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)));

    new EventTrigger("Index to Shooter")
    .onTrue(Commands.runOnce(() -> {
        indexer.setGoal(Goal.INTAKE);
        tower.setGoal(TowerGoal.INTAKE);
      }));

    new EventTrigger("StartIntakeRoller")
      .onTrue(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.INTAKE)));

    new EventTrigger("Stop Index to Shooter")
    .onTrue(Commands.runOnce(() -> {
      indexer.setGoal(Goal.STOP);
      tower.setGoal(TowerGoal.STOP);
    }));

    new EventTrigger("StopAndRetractIntake")
      .onTrue(
        Commands.runOnce(() -> 
          intake.setPivotGoal(PivotGoal.PARKED))
            .alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP)))
      );

    new EventTrigger("Stop Intake Rollers")
      .onTrue(
        Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP))
      );

    new EventTrigger("Shooter OpenLoop Passing Setpoint")
      .onTrue(Commands.runOnce(() -> shooter.setManualVoltage(9.5)));

    new EventTrigger("Hood Midpoint Position")
      .onTrue(
        Commands.runOnce(() -> hood.setGoal(HoodGoal.MIDPOINT))
      );

    new EventTrigger("Hood Minimum Position")
      .onTrue(
        Commands.runOnce(() -> hood.setGoal(HoodGoal.MINIMUM))
      );

    new EventTrigger("StopAndRetractIntakeHalf")
      .onTrue(
        Commands.runOnce(() -> 
          intake.setPivotGoal(PivotGoal.FEED))
            .alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP)))
      );

    NamedCommands.registerCommand("StopAndRetractIntake", Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.PARKED)).alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP))));
    
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      () -> -driverController.getLeftY() * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond), 
      () -> -driverController.getLeftX() * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond), 
      () -> -driverController.getRightX() * DrivetrainConstants.kMaximumLinearVelocity.in(MetersPerSecond), 
      true
    ));
  }

  private void configureDriverBindings() {
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

      driverController
      .start()
      .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

      driverController
      .a()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.FEED)));

      driverController
        .rightBumper()
        .whileTrue(Commands.runEnd(
          () -> {
            indexer.setGoal(Goal.INTAKE);
            tower.setGoal(TowerGoal.INTAKE);
            agitator.setGoal(AgitatorGoal.FEED);
          }, 
          () -> {
            indexer.setGoal(Goal.STOP);
            tower.setGoal(TowerGoal.STOP);
            agitator.setGoal(AgitatorGoal.STOP);
          }, 
          indexer, tower, agitator
        ));

      driverController
        .povLeft()
        .onTrue(Commands.runOnce(
          () -> hood.setGoal(HoodGoal.MINIMUM)
        ));

      driverController
        .povUp()
        .onTrue(Commands.runOnce(
          () -> hood.setGoal(HoodGoal.MIDPOINT)
        ));

      driverController
        .povRight()
        .onTrue(Commands.runOnce(
          () -> hood.setGoal(HoodGoal.MAXIMUM)
        ));
  }

  private void configureOperatorBindings() {
      operatorController
        .a()
        .whileTrue(Commands.runEnd(
          () -> indexer.setGoal(Goal.INTAKE), 
          () -> indexer.setGoal(Goal.STOP), 
          indexer
        ));

      operatorController
        .b()
        .whileTrue(Commands.runEnd(
          () -> indexer.setGoal(Goal.EXHAUST), 
          () -> indexer.setGoal(Goal.STOP), 
          indexer
        ));

        operatorController
        .x()
        .whileTrue(Commands.runEnd(
          () -> tower.setGoal(TowerGoal.INTAKE), 
          () -> tower.setGoal(TowerGoal.STOP), 
          tower
        ));

      operatorController
        .y()
        .whileTrue(Commands.runEnd(
          () -> tower.setGoal(TowerGoal.EXHAUST), 
          () -> tower.setGoal(TowerGoal.STOP), 
          tower
        ));

      operatorController
        .rightTrigger()
        .whileTrue(Commands.runEnd(
          () -> intake.setRollerGoal(RollerGoal.INTAKE), 
          () -> intake.setRollerGoal(RollerGoal.STOP), 
          intake
        ));

      operatorController
        .rightBumper()
        .whileTrue(Commands.runEnd(
          () -> intake.setRollerGoal(RollerGoal.EXHAUST), 
          () -> intake.setRollerGoal(RollerGoal.STOP), 
          intake
        ));

      operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.FEED)));

      operatorController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)));

      operatorController
        .povDown()
        .whileTrue(Commands.runOnce(
          () -> shooter.setManualVoltage(4.5),
          shooter
        ));

      operatorController
        .povLeft()
        .whileTrue(Commands.runOnce(
          () -> shooter.setManualVoltage(5.5),
          shooter
        ));

        operatorController
          .povUp()
          .whileTrue(Commands.runOnce(
            () -> shooter.setManualVoltage(7),
            shooter
          ));

        operatorController
          .povUp()
          .whileTrue(Commands.runOnce(
            () -> shooter.setManualVoltage(10),
            shooter
          ));

        driverController
          .rightStick()
          .whileTrue(Commands.runOnce(
            () -> shooter.setGoal(ShooterGoal.STOP),
            shooter
          ));
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
