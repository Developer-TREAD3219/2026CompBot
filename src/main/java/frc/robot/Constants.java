// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.lang.annotation.Target;
// import java.net.PortUnreachableException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean kTestMode = false;

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs (using 10-17 for drive and turning motors)
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;// 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;

    public static final double kMaxSnapSpeed = 0.2;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kGunnerDeadBand = 0.05;

    public static final double kLeftTriggerThreshold = 0.9;
    public static final double kRightTriggerThreshold = 0.9;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "limelight-tread";
  }

  /*
   * Note: CAN ID 0 used for RoboRio
   * CAN ID 10-17 used for drive and turning motors
   * CAN ID 19 PDH
   */
  // CAN ID 60 used for Pigeon IMU
  public static final class SensorConstants {
    public static final int kPigeonCanId = 60;
  }

  // CAN ID 20-21 used for launcher motors
  public static final class LauncherConstants {
    public static final int kLauncherMotorMaster = 20;
    public static final int kLauncherMotorFollower = 21;

    public static final double kLauncherMotorSpeed = 0.7;
    public static final double kLauncherReverseMotorSpeed = 1;

    public static final int kHoodControllerCanId = 62;
    public static final int kHoodPort = 0;

    public static final double kHoodMinSetpoint = 0.5;
    public static final double kHoodMaxSetpoint = 4.9;
    public static final double kPHood = 0.4;
    public static final double kIHood = 0.0;
    public static final double kDHood = 0.0;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorCanId = 25;
    public static final double kClimberMotorSpeed = 1;
    public static final double kClimberReverseMotorSpeed = 1;
    public static final double kClimberEncoderHighPoint = -40;
    // Digital input channel for the retract (bottom) limit switch
    // Set to the appropriate DIO channel on your RoboRIO
    public static final int kClimberRetractLimitChannel = 9;
    // If true the DigitalInput.get() will return true when the switch is
    // pressed. If your switch wiring is inverted (returns false when pressed),
    // set this to false.
    public static final boolean kRetractLimitPressedHigh = false;
  }

  public static final class IndexerConstants {
    public static final int kIndexerCanId = 26;
    public static final double kIndexerMotorSpeed = 1.0;
    public static final double kIndexerDelay = 1.5;
    public static final double kIndexerReverseMotorSpeed = 1.0;
  }

  public static final class UpperIndexerConstants {
    public static final int kUpperIndexerCanId = 27;
    public static final double kUpperIndexerMotorSpeed = 1.0;
    public static final double kUpperIndexerReverseMotorSpeed = 1.0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeCanId = 28;
    public static final double kIntakeMotorSpeed = 1.0;
    public static final double kIntakeReverseMotorSpeed = 1.0;
    public static final double kIntakeDelay = 1.5;
  }

  public static final class IntakeArmConstants {
    public static final int kIntakeArmCanId = 29;
    public static final double kIntakeArmMotorSpeed = 0.1;
    public static final double kIntakeArmReverseMotorSpeed = 0.1;
    public static final double kPitchGearboxRatio = 12.0; // 12:1 gear box (drive24 code used 125)
    public static final double kPitchEncoderPositionConversionFactor = 1.0 / kPitchGearboxRatio; // (drive24 code used
                                                                                                 // // 360 for the //
                                                                                                 // nominator)
    public static final double kPitchP = 0.4;
    public static final double kPitchI = 0.0;
    public static final double kPitchD = 0.0;
    // public static final double kPitchFF = 0.0;
    // Digital input channel for the retract (bottom) limit switch
    // Set to the appropriate DIO channel on your RoboRIO
    public static final int kIntakeArmRaisedLimitChannel = 8;
  }

  public static final class LedConstants {
    public static final int kPwmPort = 0;
    public static final int kTotalLeds = 39 + 25;
  }

  public static final class AimingConstants {
    public static final double RED_FAR_LEFT = 270;
    public static final double RED_MED_LEFT = 300;
    public static final double RED_NEAR_LEFT = 323;
    public static final double RED_CENTER = 0;
    public static final double RED_MED_RIGHT = 59;
    public static final double RED_FAR_RIGHT = 90;
    public static final double BLUE_FAR_LEFT = 90;
    public static final double BLUE_MED_LEFT = 121;
    public static final double BLUE_NEAR_LEFT = 149;
    public static final double BLUE_CENTER = 180;
    public static final double BLUE_MED_RIGHT = 240;
    public static final double BLUE_FAR_RIGHT = 270;

  }

  public static final class dPadConstants {
    public static final int kDPadUp = 0;
    public static final int kDPadRight = 90;
    public static final int kDPadDown = 180;
    public static final int kDPadLeft = 270;
  }

  public static final class TurretConstants {
    public static final int kTurretCanId = 30;
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kMinInput = -33;
    public static final double kMaxInput = 21;
    public static final double kTargetToleranceDegrees = 2;
    public static final double kLowClamp = -0.5;
    public static final double kHighClamp = 0.5;
    // Conversion factor from encoder rotations to turret degrees. Adjust to
    // match your gearbox: degrees = rotations * kTurretPositionConversion
    public static final double kTurretPositionConversion = 360.0; // rotations -> degrees (default 1:1)
  }

  public static class States {
    public enum State {
      Initial,
      Autonomous,
      TeleopInPosition,
      TeleopNotInPosition,
      TargetAcquired,
      NoTarget,
      Launching,
      Fueling,
      Climbing
    }

    private State m_currentState;

    States() {
      m_currentState = State.Initial;
    }

    public void setState(State newState) {
      m_currentState = newState;
    }

    public State getState() {
      return m_currentState;
    }

    public State incrementState() {
      State[] values = State.values();
      State nextState = values[(m_currentState.ordinal() + 1) % values.length];
      m_currentState = nextState;
      return m_currentState;
    }
  }

  public static final class ShootingConstants {
    // Lookup table mapping distance (meters) to launcher throttle and hood
    // setpoint.
    // Distances MUST be in ascending order.
    // Hood setpoint is normalized [0 = fully retracted, 1 = fully extended].
    // Tune these values on the practice field.
    public static final double[] kDistanceMeters = { 1.5,  2.0,  2.5,  3.0,  3.5,  4.0, 4.5, 5.0, 5.5  };
    public static final double[] kThrottleValues  = { 0.45, 0.50, 0.51, 0.55, 0.55, 0.58, 0.61, 0.65, 0.70 };
    public static final double[] kHoodAngles      = { 0.0, 0.0, 0.10, 0.15, 0.20, 0.025, 0.30, 0.34, 0.38 };

    /**
     * Finds where 'distance' falls between two entries in the distance table,
     * then returns a proportionally blended value from the corresponding output
     * table.
     * If distance is outside the table range, returns the nearest endpoint value.
     */
    public static double interpolate(double[] distanceTable, double[] outputTable, double distance) {
      // Below the minimum — use the closest known value
      if (distance <= distanceTable[0]) {
        return outputTable[0];
      }
      // Above the maximum — use the closest known value
      if (distance >= distanceTable[distanceTable.length - 1]) {
        return outputTable[outputTable.length - 1];
      }
      // Find the two table entries that bracket this distance
      for (int i = 1; i < distanceTable.length; i++) {
        if (distance <= distanceTable[i]) {
          double lowerDistance = distanceTable[i - 1];
          double upperDistance = distanceTable[i];
          double lowerOutput = outputTable[i - 1];
          double upperOutput = outputTable[i];
          // How far between the two entries (0 = at lower, 1 = at upper)
          double fraction = (distance - lowerDistance) / (upperDistance - lowerDistance);
          return lowerOutput + fraction * (upperOutput - lowerOutput);
        }
      }
      return outputTable[outputTable.length - 1];
    }
  }

  public enum Zones {
    RED_TRENCH,
    BLUE_TRENCH,
    RED_HUB,
    BLUE_HUB,
    RED_TOWER,
    BLUE_TOWER,
    RED_OUTPOST,
    BLUE_OUTPOST
  }
}
