<<<<<<< HEAD
package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class IndexerConstants {
    public static final MinoCANDevice kIndexerMotor = new MinoCANDevice(14, GlobalConstants.kRioBus);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double simulatedKp = 0.0;
    public static final double simulatedKi = 0.0;
    public static final double simulatedKd = 0.0;
    public static final double simulatedKs = 0.0;
    public static final double simulatedKv = 0.0;
    public static final double simulatedKa = 0.0;

    public static final boolean kIndexerMotorInverted = true;
    public static final double kIndexerMotorReduction = (24.0 / 14.0);
    public static final Current kIndexerMotorSupplyLimit = Amps.of(45);
    
    public static final DCMotor kIndexerSimulatedGearbox = DCMotor.getNEO(1);

    public static final MomentOfInertia kIndexerMOI = KilogramSquareMeters.of(0.0002);

    public static final double kIndexerMotorPositionConversionFactor = (1 / kIndexerMotorReduction) * 2 * Math.PI;
    public static final double kIndexerMotorVelocityConversionFactor = 1 / kIndexerMotorPositionConversionFactor;
}
=======
package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.minolib.hardware.MinoCANDevice;

public class IndexerConstants {
    public static final AngularVelocity kMaximumRotationalVelocity = RadiansPerSecond.of(4.2);
    public static final AngularAcceleration kMaximumRotationalAcceleration = RadiansPerSecondPerSecond.of(6.0);

    public static final MinoCANDevice kLeftMotor = new MinoCANDevice(17, GlobalConstants.kRioBus);
    public static final MinoCANDevice kRightMotor = new MinoCANDevice(18, GlobalConstants.kRioBus);

    public static final double leftkP = 0.0;
    public static final double leftkD = 0.0;
    public static final double leftkS = 0.0;
    public static final double leftkV = 0.0;
    public static final double leftkA = 0.0;

    public static final double rightkP = 0.0;
    public static final double rightkD = 0.0;
    public static final double rightkS = 0.0;
    public static final double rightkV = 0.0;
    public static final double rightkA = 0.0;

    public static final boolean kLeftMotorInverted = true;
    public static final double kLeftMotorReduction = (24.0 / 14.0);
    public static final DCMotor kLeftSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final boolean kRightMotorInverted = true;
    public static final double kRightMotorReduction = (24.0 / 14.0);
    public static final DCMotor kRightSimulatedGearbox = DCMotor.getKrakenX44Foc(1);

    public static final double kRollerReduction = (3.0 / 1.0);

    public static final Current kMotorStatorLimit = Amps.of(80);
    public static final Current kMotorSupplyLimit = Amps.of(40);

    public static final MomentOfInertia kRollerMOI = KilogramSquareMeters.of(0.0002);
}
>>>>>>> 3163bf53965ccc6cde13a6cbacc44f3c0a1bd18b
