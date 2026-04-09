<<<<<<< HEAD
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  private Hang hang;

  //TODO: Check the controller axis inputs and ensure that they align with the assigned values for an Xbox Elite Controller
  private CommandSimulatedXboxController driverController = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
  private CommandSimulatedXboxController operatorController = new CommandSimulatedXboxController(ControllerConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoRegistry;

  private double targetedShooterVoltage = 4.0;

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
        new IndexerIOHardware()
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

  private Hang buildHang() {
    if(Robot.isSimulation()) {
      return new Hang(
        new HangIOHardware()
      );
    } else {
      return new Hang(
        new HangIOHardware()
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

  public Hang getHang() {
    return hang;
  }
  public RobotContainer() {
    drivetrain = buildDrivetrain();
    intake = buildIntake();
    indexer = buildIndexer();
    tower = buildTower();
    shooter = buildShooter();
    hood = buildHood();
    hang = buildHang();

    tower.setBrakeMode(() -> false);
    
    NamedCommands.registerCommand("DeployIntake", Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)));
    NamedCommands.registerCommand("StartIntakeRoller", Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.INTAKE)));
    NamedCommands.registerCommand("Hood Minimum Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MINIMUM)));
    NamedCommands.registerCommand("Hood Minimum-Midpoint Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MINIMUM_MIDPOINT)));
    NamedCommands.registerCommand("Hood Midpoint Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MIDPOINT)));
    NamedCommands.registerCommand("Hood Maximum Position", Commands.runOnce(() -> hood.setGoal(HoodGoal.MAXIMUM)));
    NamedCommands.registerCommand("Shooter Open Loop Close-Mid", shooter.runVoltage(5));
    NamedCommands.registerCommand("Shooter Open Loop Mid", shooter.runVoltage(7));
    NamedCommands.registerCommand(
      "Hang Sequence", 
      Commands.runOnce(() -> hang.setPivotGoal(HangPivotGoal.DOWN))
        .andThen(Commands.waitSeconds(0.5))
        .finallyDo(() -> hang.setPivotGoal(HangPivotGoal.STOP))
    );
    NamedCommands.registerCommand("Index to Shooter", 
      Commands.runOnce(() -> {
        indexer.setGoal(Goal.INTAKE);
        tower.setGoal(TowerGoal.INTAKE);
      }));

    NamedCommands.registerCommand("Stop Index to Shooter", 
    Commands.runOnce(() -> {
      indexer.setGoal(Goal.STOP);
      tower.setGoal(TowerGoal.STOP);
    }));

    NamedCommands.registerCommand("Stop Intake Rollers", 
    Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP)));

    new EventTrigger("StopAndRetractIntake")
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.PARKED)).alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP))));

    NamedCommands.registerCommand("StopAndRetractIntake", Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.PARKED)).alongWith(Commands.runOnce(() -> intake.setRollerGoal(RollerGoal.STOP))));

    autoRegistry = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autoRegistry.addOption("Drivetrain Translation Dynamic Forward", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Dynamic Reverse", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Forward", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Reverse", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      () -> -driverController.getLeftY(), 
      () -> -driverController.getLeftX(), 
      () -> driverController.getRightX(), 
      true
    ));
  }

  private void configureDriverBindings() {
    driverController
      .leftTrigger()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.DEPLOY)))
      .whileTrue(
        Commands.runEnd(
          () -> intake.setRollerGoal(RollerGoal.INTAKE), 
          () -> intake.setRollerGoal(RollerGoal.STOP), 
          intake
        )
      );

    driverController
      .rightTrigger()
      .whileTrue(Commands.startEnd(
        () -> shooter.setGoal(ShooterGoal.VOLTAGE), 
        () -> shooter.setGoal(ShooterGoal.IDLE), 
        shooter
      ));

    driverController
      .leftBumper()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.PARKED)));
   
    driverController
      .start()
      .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

    driverController
      .x()
      .whileTrue(Commands.runOnce(
        () -> shooter.setManualVoltage(4.0),
        shooter
      ));

    driverController
      .y()
      .whileTrue(Commands.runOnce(
        () -> shooter.setManualVoltage(8.0),
        shooter
      ));

      driverController
        .b()
        .whileTrue(Commands.runOnce(
          () -> shooter.setManualVoltage(12.0),
          shooter
        ));

      driverController
      .a()
      .onTrue(Commands.runOnce(() -> intake.setPivotGoal(PivotGoal.FEED)));

      driverController
        .rightBumper()
        .whileTrue(Commands.runEnd(
          () -> {
            indexer.setGoal(Goal.INTAKE);
            tower.setGoal(TowerGoal.INTAKE);
          }, 
          () -> {
            indexer.setGoal(Goal.STOP);
            tower.setGoal(TowerGoal.STOP);
          }, 
          indexer, tower
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
        .rightBumper()
        .whileTrue(Commands.runEnd(
          () -> intake.setRollerGoal(RollerGoal.INTAKE), 
          () -> intake.setRollerGoal(RollerGoal.STOP), 
          intake
        ));

      operatorController
        .rightTrigger()
        .whileTrue(Commands.runEnd(
          () -> intake.setRollerGoal(RollerGoal.EXHAUST), 
          () -> intake.setRollerGoal(RollerGoal.STOP), 
          intake
        ));

      operatorController
        .leftBumper()
        .whileTrue(Commands.runEnd(
          () -> intake.setPivotGoal(PivotGoal.DEPLOY), 
          () -> intake.setPivotGoal(PivotGoal.DEPLOY), 
          intake
        ));

      operatorController
        .leftTrigger()
        .whileTrue(Commands.runEnd(
          () -> intake.setPivotGoal(PivotGoal.PARKED), 
          () -> intake.setPivotGoal(PivotGoal.PARKED), 
          intake
        ));

      operatorController
        .leftStick()
        .whileTrue(Commands.runEnd(
          () -> hang.setPivotGoal(HangPivotGoal.UP), 
          () -> hang.setPivotGoal(HangPivotGoal.STOP), 
          hang
        ));

      operatorController
        .rightStick()
        .whileTrue(Commands.runEnd(
          () -> hang.setPivotGoal(HangPivotGoal.DOWN), 
          () -> hang.setPivotGoal(HangPivotGoal.STOP), 
          hang
        ));

      operatorController
        .povUp()
        .whileTrue(Commands.runEnd(
          () -> hang.setWinderGoal(WinderGoal.UP), 
          () -> hang.setWinderGoal(WinderGoal.STOP), 
          hang
        ));

      operatorController
        .povDown()
        .whileTrue(Commands.runEnd(
          () -> hang.setWinderGoal(WinderGoal.DOWN), 
          () -> hang.setWinderGoal(WinderGoal.STOP), 
          hang
        ));
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Consumer;
import java.util.zip.ZipEntry;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.minolib.localization.WeightedPoseEstimate;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TowerConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.io.Controlboard;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainIOHardware;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.SimulationTunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.rollers.RollerSystemIOSimulation;
import frc.robot.subsystems.rollers.RollerSystemIOHardware;
import frc.robot.subsystems.shooter.flywheel.Shooter;
import frc.robot.subsystems.shooter.flywheel.ShooterIOHardware;
import frc.robot.subsystems.shooter.flywheel.ShooterIOSimulation;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOHardware;
import frc.robot.subsystems.shooter.hood.HoodIOSimulation;
import frc.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.slam.SlamIOSimulation;
import frc.robot.subsystems.intake.slam.SlamIOHardware;
import frc.robot.subsystems.tower.Tower;
import frc.robot.subsystems.tower.Tower.TowerGoal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSimulation;

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
  private Elevator elevator;

  private Controlboard controlboard = Controlboard.getInstance();
  private final LoggedDashboardChooser<Command> autoRegistry;

  private final Alert primaryDisconnected = new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected = new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected = new Alert("Override controller disconnected (port 5).", AlertType.kInfo);

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
       new SlamIOSimulation(),
       new RollerSystemIOSimulation()
        //robotState
     );
   } else {
     return new Intake(
      new SlamIOHardware(),
      new RollerSystemIOHardware(IntakeConstants.kRollerMotor, IntakeConstants.kRollerMotorSupplyLimit.in(Amps), IntakeConstants.kRollerMotorInverted, true, IntakeConstants.kRollerMotorReduction)
     );
   }
  }

  private Indexer buildIndexer() {
    if(Robot.isSimulation()) {
      return new Indexer(
        new RollerSystemIOSimulation(),
        new RollerSystemIOSimulation()
      );
    } else {
      return new Indexer( 
        new RollerSystemIOHardware(IndexerConstants.kLeftMotor, IndexerConstants.kMotorSupplyLimit.in(Amps), false, true, IndexerConstants.kRollerReduction),
        new RollerSystemIOHardware(IndexerConstants.kRightMotor, IndexerConstants.kMotorSupplyLimit.in(Amps), true, true, IndexerConstants.kRollerReduction)
      );
    }
  }

  private Tower buildTower() {
    if(Robot.isSimulation()) {
      return new Tower(
        new RollerSystemIOSimulation(),
        new RollerSystemIOSimulation()
      );
    } else {
      return new Tower(
        new RollerSystemIOHardware(TowerConstants.kTopMotor, TowerConstants.kTopRollerSupplyLimit.in(Amps), true, true, TowerConstants.kTopMotorReduction),
        new RollerSystemIOHardware(TowerConstants.kBottomMotor, TowerConstants.kBottomRollerSupplyLimit.in(Amps), true, false, TowerConstants.kBottomMotorReduction)
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

  private Elevator buildElevator() {
    if(Robot.isSimulation()) {
      return new Elevator(
        new ElevatorIOHardware()
      );
    } else {
      return new Elevator(
        new ElevatorIOHardware()
      );
    }
  }

  private Vision buildVision() {
    if(Robot.isSimulation()) {
      return new Vision(
        robotState,
        new VisionIOSimulation(VisionConstants.kBackLeftConfiguration, VisionConstants.kAprilTagLayout, robotState)
      );
    } else {
      return new Vision(
        robotState,
        new VisionIOPhotonVision(VisionConstants.kBackLeftConfiguration, VisionConstants.kAprilTagLayout)
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
    indexer = buildIndexer();
    tower = buildTower();
    shooter = buildShooter();
    hood = buildHood();
    elevator = buildElevator();

    autoRegistry = new LoggedDashboardChooser<Command>("Auton Choices", AutoBuilder.buildAutoChooser());
    autoRegistry.addOption("Drivetrain Translation Dynamic Forward", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Dynamic Reverse", drivetrain.sysIdDynamic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Forward", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kForward));
    autoRegistry.addOption("Drivetrain Translation Quasisatic Reverse", drivetrain.sysIdQuasistatic(Drivetrain.SysIdMechanism.SWERVE_TRANSLATION, Direction.kReverse));
    autoRegistry.addOption("Do Nothing", Commands.none());
    
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      robotState, 
      () -> controlboard.getThrottle(), 
      () -> controlboard.getStrafe(), 
      () -> controlboard.getRotation(), 
      true
    ));
  }

  private void configureDriverBindings() {
    controlboard
      .resetGyro()
      .onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

    controlboard
      .deployIntake()
      .whileTrue(IntakeFactory.intakeCommand(this))
      .onFalse(IntakeFactory.deployHalfCommand(this));

    controlboard
      .stowIntake()
      .whileTrue(IntakeFactory.stowCommand(this));

    controlboard.automaticallyShoot()
      .whileTrue(
        Commands.parallel(
          Commands.sequence(
            Commands.waitSeconds(0.75),
            Commands.runEnd(
              () -> tower.setTowerGoal(TowerGoal.FEED), 
              () -> tower.setTowerGoal(TowerGoal.STOP), 
              tower
            )
          ),
          Commands.runEnd(
            () -> shooter.runVelocity(2500), 
            () -> shooter.stop(),
            shooter
          ),
          Commands.runEnd(
              () -> indexer.setGoal(IndexerGoal.FEED), 
              () -> indexer.setGoal(IndexerGoal.STOP), 
              indexer
          ),
          Commands.runEnd(
            () -> hood.setAngle(20), 
            () -> hood.setAngle(10), 
            hood
          )
        )
      );
  }

  public void updateOnboardAlerts() {
    primaryDisconnected.set(!DriverStation.isJoystickConnected(controlboard.getPrimaryHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(controlboard.getSecondaryHID().getPort()));
  }

  private void configureOperatorBindings() {
   
  }

  public Command getAutonomousCommand() {
    return autoRegistry.get();
  }
}
>>>>>>> 3163bf53965ccc6cde13a6cbacc44f3c0a1bd18b
