package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Intake extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD");
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS");
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG");
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA");
    
    private static final LoggedTunableNumber kPivotMaximumAngle = new LoggedTunableNumber("Intake/Pivot/MaximumAngle", IntakeConstants.kIntakeMaximumPosition.in(Radians));
    private static final LoggedTunableNumber kMaximumVelocityRadiansPerSecond = new LoggedTunableNumber("Intake/Pivot/MaxVelocityRadiansPerSecond", IntakeConstants.kPivotMaximumRotationalVelocity.in(RadiansPerSecond));
    private static final LoggedTunableNumber kMaximumAccelerationRadiansPerSecond2 = new LoggedTunableNumber("Intake/Pivot/MaxAccelerationRadiansPerSecond2", IntakeConstants.kPivotMaximumRotationalAcceleration.in(RadiansPerSecondPerSecond));
    private static final LoggedTunableNumber kHomingVolts = new LoggedTunableNumber("Intake/Pivot/HomingVoltage", -3.0);
    private static final LoggedTunableNumber kHomingTimeoutSeconds = new LoggedTunableNumber("Intake/Pivot/HomingTimeSeconds", 0.4);
    private static final LoggedTunableNumber kHomingVelocityThreshold = new LoggedTunableNumber("Intake/Pivot/HomingVelocityThreshold", 0.1);

    @RequiredArgsConstructor
    public enum Goal {
        DEPLOY(new LoggedTunableNumber("Intake/Pivot/DeployedDegrees", 5.0)),
        PARKED(new LoggedTunableNumber("Intake/Pivot/ParkedDegrees", 100.0)),
        FEED(new LoggedTunableNumber("Intake/Pivot/FeedDegrees", 18)),
        CLIMB(new LoggedTunableNumber("Intake/Pivot/ClimbDegrees", 70.0));

        private final DoubleSupplier positionDegrees;

        public double getAngleRad() {
            return Units.degreesToRadians(positionDegrees.getAsDouble());
        }
    }

    static {
        switch (GlobalConstants.getRobot()) {
            default -> {
                kP.initDefault(1800);
                kD.initDefault(100);
                kS.initDefault(0);
                kG.initDefault(20);
                kA.initDefault(0);
            }
            case SIMBOT -> {
                kP.initDefault(1000.0);
                kD.initDefault(0);
                kS.initDefault(0);
                kG.initDefault(0);
                kA.initDefault(0);
            }
        }
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Debouncer pivotMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer rollerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert pivotMotorDisconnectedAlert = new Alert("Intake pivot motor disconnected!", AlertType.kError);
    private final Alert rollerMotorDisconnectedAlert = new Alert("Intake roller motor disconnected!", AlertType.kError);

    @AutoLogOutput(key = "Intake/BrakeModeEnabled")
    private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    @Getter private Goal goal = Goal.PARKED;
    private boolean stopProfile = false;

    @AutoLogOutput(key = "Intake/Pivot/HomedPositionRadians")
    private double pivotHomedPosition = 0.0;

    @AutoLogOutput(key = "Intake/Pivot/Homed")
    @Getter
    private boolean pivotHomed = false;

    @Setter 
    private double rollerVoltage = 0.0;

    private Debouncer pivotHomingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());
    private final Command pivotHomingCommand;

    @Getter
    @Accessors(fluent = true)
    @AutoLogOutput(key = "Intake/Pivot/Profile/AtGoal")
    private boolean pivotAtGoal = false;

    @Getter
    @Accessors(fluent = true)
    private boolean pivotWantsToDeploy = false;

    public Intake(IntakeIO io) {
        this.io = io;

        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                kMaximumVelocityRadiansPerSecond.get(), 
                kMaximumAccelerationRadiansPerSecond2.get()
            )
        );

        pivotHomingCommand = backupHomingSequence();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        pivotMotorDisconnectedAlert.set(!pivotMotorConnectedDebouncer.calculate(inputs.pivotMotorConnected) && !Robot.isJITing());
        rollerMotorDisconnectedAlert.set(!rollerMotorConnectedDebouncer.calculate(inputs.rollerMotorConnected) && !Robot.isJITing());

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPivotPID(kP.get(), 0.0, kD.get());
        }

        if (kMaximumVelocityRadiansPerSecond.hasChanged(hashCode()) || kMaximumAccelerationRadiansPerSecond2.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(
                    kMaximumVelocityRadiansPerSecond.get(), 
                    kMaximumAccelerationRadiansPerSecond2.get()
                )
            );
        }

        pivotWantsToDeploy = !pivotHomed || (getPivotMeasuredAngleRadians() < kPivotMaximumAngle.get() / 2.0);

        // Home on enable
        if (DriverStation.isEnabled() && !pivotHomed && !pivotHomingCommand.isScheduled()) {
            pivotHomingCommand.schedule();
        }

        // Run profile
        final boolean shouldRunProfile = !stopProfile
            && brakeModeEnabled
            && (pivotHomed || GlobalConstants.getRobot() == GlobalConstants.RobotType.SIMBOT)
            && DriverStation.isEnabled();

        Logger.recordOutput("Intake/Pivot/RunningProfile", shouldRunProfile);

        if (shouldRunProfile) {
            var goalState = new State(MathUtil.clamp(goal.getAngleRad(), 0.0, kPivotMaximumAngle.get()), 0.0);
            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(GlobalConstants.kLoopPeriodSeconds, setpoint, goalState);

            if (setpoint.position < 0.0 || setpoint.position > kPivotMaximumAngle.get()) {
                setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, kPivotMaximumAngle.get()), 0.0);
            }

            pivotAtGoal = EqualsUtility.epsilonEquals(setpoint.position, goalState.position) && EqualsUtility.epsilonEquals(setpoint.velocity, goalState.velocity);

            double accel = (setpoint.velocity - previousVelocity) / GlobalConstants.kLoopPeriodSeconds;
            io.setPivotPosition(
                setpoint.position + pivotHomedPosition,
                kS.get() * Math.signum(setpoint.velocity) + kG.get() * Math.cos(setpoint.position) + kA.get() * accel
            );

            // Log state
            Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionRadians", setpoint.position);
            Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadiansPerSecond", setpoint.velocity);
            Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRadians", goalState.position);
            Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadiansPerSecond", goalState.velocity);
        } else {
            // Reset setpoint
            setpoint = new State(getPivotMeasuredAngleRadians(), 0.0);

            // Clear logs
            Logger.recordOutput("Intake/Pivot/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Intake/Pivot/Profile/SetpointVelocityRadiansPerSecond", 0.0);
            Logger.recordOutput("Intake/Pivot/Profile/GoalPositionRadians", 0.0);
            Logger.recordOutput("Intake/Pivot/Profile/GoalVelocityRadiansPerSecond", 0.0);
        }

        io.setRollerVoltage(rollerVoltage);

        // Log state
        Logger.recordOutput("Intake/Pivot/MeasuredVelocityMetersPerSec", inputs.pivotVelocity);
        LoggedTracer.record("Intake");
    }

    public void setGoal(Goal goal) {
        if (goal == this.goal) return;
        this.goal = goal;
        pivotAtGoal = false;
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public void setHome() {
        pivotHomedPosition = inputs.pivotPosition;
        pivotHomed = true;
    }

    private Command backupHomingSequence() {
        return Commands.startRun(() -> {
            stopProfile = true;
            pivotHomed = false;
            pivotHomingDebouncer = new Debouncer(kHomingTimeoutSeconds.get());
            pivotHomingDebouncer.calculate(false);
        }, () -> {
            if (!brakeModeEnabled) return;

            io.setPivotVoltage(kHomingVolts.get());
            pivotHomed = pivotHomingDebouncer.calculate(Math.abs(inputs.pivotVelocity) <= kHomingVelocityThreshold.get() && Math.abs(inputs.pivotAppliedVoltage) >= kHomingVolts.get() * 0.7);
        }).until(() -> pivotHomed).andThen(this::setHome).finallyDo(() -> {
            stopProfile = false;
        });
    }

    public void overrideHoming() {
        pivotHomed = false;
    }

    @AutoLogOutput(key = "Intake/Pivot/MeasuredAngleRads")
    public double getPivotMeasuredAngleRadians() {
        return inputs.pivotPosition - pivotHomedPosition;
    }

    public double getRollerTorqueCurrent() {
        return inputs.rollerTorqueCurrentAmperes;
    }

    public double getRollerVelocity() {
        return inputs.rollerVelocity;
    }

    public void stopRoller() {
        rollerVoltage = 0.0;
    }
}
