// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_pigeon;
  private Rotation2d headingOffset = new Rotation2d();
  private SwerveDrivePoseEstimator m_poseEstimator;

  private final Field2d m_field = new Field2d();

  private final PIDController m_headingController = new PIDController(0.01, 0.0, 0.001); // likely needs tuning

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Pigeon2 pigeon) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_pigeon = pigeon;
    System.out.println("The drive has the pigeon" + m_pigeon);
    m_pigeon.clearStickyFaults();
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration());

    m_headingController.enableContinuousInput(-180.0, 180.0);
    m_headingController.setTolerance(2.0); // degrees

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getGyroRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation2d(),
        getSwervePositions(),
        new Pose2d());

    SmartDashboard.putData("Field", m_field);
    
    //swerve data for dashboard widget.
    SmartDashboard.putData("Swerve",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> m_frontLeft.getPosition().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () ->m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> m_rearRight.getState().angle.getRadians(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> m_rearRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> (getHeading())*(Math.PI / 180), null);
        });


    // -------------------------------------------------------
    // PathPlanner AutoBuilder configuration (must be last)
    // -------------------------------------------------------
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      // Fall back to a default so the robot still compiles/runs
      throw new RuntimeException("Failed to load PathPlanner RobotConfig", e);
    }

    AutoBuilder.configure(
        this::getPose, // Pose2d supplier
        this::resetPose, // Pose2d consumer – called at the start of each auto
        this::getRobotRelativeSpeeds, // Robot-relative ChassisSpeeds supplier
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Robot-relative ChassisSpeeds consumer
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID – tune as needed
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID – tune as needed
        ),
        config,
        () -> {
          // Flip paths to the red side of the field when on the red alliance.
          // The field origin always stays on the blue side.
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this // Subsystem requirement so commands can interrupt this subsystem
    );
  }

  public SwerveDrivePoseEstimator getPoseEstimator() {
    return m_poseEstimator;
  }

  public Pigeon2 getGyro() {
    return m_pigeon;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_field.setRobotPose(getPose());
    SmartDashboard.putNumber("[HEADING]", getHeading());
    SmartDashboard.putNumber("[POSE X]", getPose().getX());
    SmartDashboard.putNumber("[POSE Y]", getPose().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the robot's odometry to the given pose.
   * Called by PathPlanner at the start of every auto that has a starting pose.
   */
  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation2d(),
        getSwervePositions(),
        pose);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Returns the current robot-relative ChassisSpeeds.
   * Required by PathPlanner's AutoBuilder.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Drives the robot given robot-relative ChassisSpeeds.
   * Called by PathPlanner during autonomous path following.
   *
   * @param speeds Robot-relative ChassisSpeeds.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getGyroRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.reset();
  }

  public double calculateTurnToHeading(double targetDegrees) {
    double output = m_headingController.calculate(getHeading(), targetDegrees);
    return MathUtil.clamp(output, -1.0, 1.0);
  }

  public boolean atHeadingSetpoint() {
    return m_headingController.atSetpoint();
  }

  public void resetHeadingController() {
    m_headingController.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2d().getDegrees();
  }

  public void flipHeading() {
    double flipped = MathUtil.inputModulus(m_pigeon.getYaw().getValueAsDouble() + 180.0, -180.0, 180.0);
    m_pigeon.setYaw(flipped);
  }

  /**
   * Returns the current rotation of the gyro.
   */
  private Rotation2d getGyroRotation2d() {
    if (RobotBase.isSimulation()) {
      return new Rotation2d(0);
    }
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValueAsDouble() % 360);
  }

  /**
   * Returns the current positions of the swerve modules.
   */
  private SwerveModulePosition[] getSwervePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }
}