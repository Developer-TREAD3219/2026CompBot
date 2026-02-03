// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// hello world 
package frc.robot;

import java.net.PortUnreachableException;

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

  public static final boolean kTestMode = true;

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

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
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

    public static final double kLauncherMotorSpeed = 0.05;
    public static final double kLauncherReverseMotorSpeed = 0.01;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorCanId = 25;
  }

  public static final class IndexerConstants {
    public static final int kIndexerCanId = 26;
    public static final double kIndexerMotorSpeed = 0.5;
    public static final double kIndexerDelay = 1.5;
    public static final double kIndexerReverseMotorSpeed = 0.1;
  }

  public static final class IntakeConstants {
    public static final int kIntakeCanId = 28;
    public static final double kIntakeMotorSpeed = 0.5;
    public static final double kIntakeDelay = 1.5;
  }

  public static final class LedConstants {
    public static final int kPwmPort = 9;
    public static final int kTotalLeds = 164; // 60 LEDs per Meter for 2.74 meters (27in x 4)
  }

  public static class States {
    public enum State {
      Initial,
      Autonomous,
      TeleopInPosition,
      TeleopNotInPosition,
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
}