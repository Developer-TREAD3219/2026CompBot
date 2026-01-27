// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports for Rev Robotics MaxSwerve
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LauncherCommands.Launch;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.lang.Thread.State;
import java.util.List;

// other imports
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public CANBus m_CanBus = new CANBus();
  public Pigeon2 m_Pigeon = new Pigeon2(Constants.SensorConstants.kPigeonCanId, m_CanBus);
  private States.State m_botState = States.State.Initial;

  // The robot's subsystems and commands are defined here...

  // MOVED TO INSIDE public RobotContainer(), because need to send pigeon as
  // parameter.
  private final DriveSubsystem m_robotDrive;
  private final IntakeSubsystem m_robotIntake;
  private final TurretSubsystem m_robotTurret;
  private final ClimberSubsystem m_robotClimber;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  private final LauncherSubsystem m_launcherSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize subsystems
    m_botState = States.State.Initial;
    if (!Constants.kTestMode) {
      m_robotDrive = new DriveSubsystem(m_Pigeon);
      m_robotIntake = new IntakeSubsystem();
      m_robotTurret = new TurretSubsystem();
      m_robotClimber = new ClimberSubsystem();
    } else {
      m_robotClimber = new ClimberSubsystem();
      m_launcherSubsystem = new LauncherSubsystem(m_gunnerController, m_botState);
      m_robotDrive = null;
      m_robotIntake = new IntakeSubsystem();
      m_robotTurret = new TurretSubsystem();
    }

    // Configure the button bindings
    if (!Constants.kTestMode) {
      configureButtonBindings();
    } else {
      new JoystickButton(m_gunnerController, XboxController.Button.kY.value)
          .whileTrue(new RunCommand(() -> m_robotTurret.lockOntoHub(), m_robotTurret));
      Trigger launchTrigger = new Trigger(this::launchRequested);
      launchTrigger.whileTrue(new InstantCommand(
          () -> m_launcherSubsystem.startLauncher(LauncherConstants.kLauncherMotorSpeed), m_launcherSubsystem));
      launchTrigger.onFalse(new InstantCommand(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem));
      Trigger extendClimberTrigger = new Trigger(this::extendClimberRequested);
      extendClimberTrigger.whileTrue(new RunCommand(() -> m_robotClimber.extendClimber(0.5), m_robotClimber));
      extendClimberTrigger.onFalse(new RunCommand(() -> m_robotClimber.stopClimber(), m_robotClimber));
      Trigger retractClimberTrigger = new Trigger(this::retractClimberRequested);
      retractClimberTrigger.whileTrue(new RunCommand(() -> m_robotClimber.retractClimber(0.5), m_robotClimber));
      retractClimberTrigger.onFalse(new RunCommand(() -> m_robotClimber.stopClimber(), m_robotClimber));
      // The left trigger while held runs the intake rollers
      Trigger intakeTrigger = new Trigger(this::intakeRequested);
      intakeTrigger.onTrue(new InstantCommand(() -> m_robotIntake.startIntakeRollers(), m_robotIntake));
      intakeTrigger.onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeRollers(), m_robotIntake));
    }

    // Configure default commands
    if (m_robotDrive != null) {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                  true),
              m_robotDrive));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * DRIVER CONTROLS
     */
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    /*
     * GUNNER CONTROLS
     */
    // A button toggles intake extensions (drops or retracts)
    new JoystickButton(m_gunnerController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(() -> m_robotIntake.toggleIntakeExtentions(), m_robotIntake));
    // B button while held tracks outpost with limelight to shoot into alliance zone
    new JoystickButton(m_gunnerController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_robotTurret.setAllianceZoneLock(), m_robotTurret));
    // X button toggles latch hook on climber
    new JoystickButton(m_gunnerController, XboxController.Button.kX.value)
        .onTrue(new RunCommand(() -> m_robotClimber.toggleHookLatch(), m_robotClimber));
    // Y button while held locks onto the AprilTag on the Hub
    new JoystickButton(m_gunnerController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_robotTurret.lockOntoHub(), m_robotTurret));
    // Left Bumper while held runs intake rollers in reverse
    new JoystickButton(m_gunnerController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> m_robotIntake.reverseIntakeRollers(), m_robotIntake));
    // Start button starts/stops launcher motors
    new JoystickButton(m_gunnerController, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(() -> m_robotTurret.startStopLauncherMotors(), m_robotTurret));
    // new JoystickButton(m_gunnerController,
    // XboxController.Button.kRightStick.value)
    // .whileTrue(new RunCommand(() -> m_robotTurret.aimLauncher(), m_robotTurret));

    // DPad Up extends climber while held
    Trigger extendClimberTrigger = new Trigger(this::extendClimberRequested);
    extendClimberTrigger.whileTrue(new RunCommand(() -> m_robotClimber.extendClimber(0.5), m_robotClimber));
    extendClimberTrigger.onFalse(new RunCommand(() -> m_robotClimber.stopClimber(), m_robotClimber));
    // DPad Down retracts climber while held
    Trigger retractClimberTrigger = new Trigger(this::retractClimberRequested);
    retractClimberTrigger.whileTrue(new RunCommand(() -> m_robotClimber.retractClimber(0.5), m_robotClimber));
    retractClimberTrigger.onFalse(new RunCommand(() -> m_robotClimber.stopClimber(), m_robotClimber));
    // The right trigger while held runs the launcher motors
    Trigger launchTrigger = new Trigger(this::launchRequested);
    launchTrigger.whileTrue(new InstantCommand(
        () -> m_launcherSubsystem.startLauncher(LauncherConstants.kLauncherMotorSpeed), m_launcherSubsystem));
    launchTrigger.onFalse(new InstantCommand(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem));
    // The left trigger while held runs the intake rollers
    Trigger intakeTrigger = new Trigger(this::intakeRequested);
    intakeTrigger.onTrue(new InstantCommand(() -> m_robotIntake.startIntakeRollers(), m_robotIntake));
    intakeTrigger.onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeRollers(), m_robotIntake));

    // TODO: Driver controls
    // new JoystickButton(m_driverController,
    // XboxController.Button.kLeftTrigger.value).whileTrue(new RunCommand(() ->
    // m_robotDrive.brakeSlowDown(), m_robotDrive));
  }

  /*
   * Gunner
   * A -> Extend/Retract Intake (toggle) DONE
   * B -> Track outpost with Limelight to shoot into alliance zone (hold) DONE
   * X -> Latch Hook on climber (toggle) DONE
   * Y -> Lock onto hub (hold)
   * LB -> Reverse Intake Rollers (hold)
   * RB ->
   * LeftJoystick ->
   * LeftJoystickClick ->
   * RightJoystick -> Aim launcher
   * RightJoystickClick ->
   * DPad Up -> Climber extend (hold)
   * DPad Down -> Climber retract (hold)
   * DPad Left ->
   * DPad Right ->
   * LeftTrigger -> Start/stop Intake Rollers (toggle) DONE
   * RightTrigger -> Launch fuel (hold) DONE
   * StartButton -> Start/Stop launcher motors (toggle) DONE
   * 
   */
  /*
   * Driver
   * A ->
   * B ->
   * X ->
   * Y -> a
   * LB ->
   * RB ->
   * LeftJoystick -> Motion of robot
   * RightJoystick -> Rotation of robot
   * DPad Up ->
   * DPad Down ->
   * DPad Left ->
   * DPad Right ->
   * LeftTrigger -> Brake/Slow down (go slower when held more)? Dont really need
   * but maybe
   * RightTrigger ->
   * 
   */

  // Check if right trigger is pressed on the gunner controller
  // Called by launch Trigger in configureButtonBindings()
  public boolean launchRequested() {
    return m_gunnerController.getRightTriggerAxis() > 0.9;
  }

  // Check if left trigger is pressed on the gunner controller
  // Called by intake Trigger in configureButtonBindings()
  public boolean intakeRequested() {
    return m_gunnerController.getLeftTriggerAxis() > 0.9;
  }

  // Check if DPad Up is pressed on the gunner controller
  public boolean extendClimberRequested() {
    return m_gunnerController.getPOV() == 0;
  }

  // Check if DPad Down is pressed on the gunner controller
  public boolean retractClimberRequested() {
    return m_gunnerController.getPOV() == 180;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
