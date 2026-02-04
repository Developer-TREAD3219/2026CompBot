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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LauncherCommands.Launch;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.UpperIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final LimeLightSubsystem m_Limelight;
  private final IndexerSubsystem m_robotIndexer;
  private final LauncherSubsystem m_launcherSubsystem;
  private final UpperIndexerSubsystem m_UpperIndexerSubsystem;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

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
      m_robotIndexer = new IndexerSubsystem();
      m_UpperIndexerSubsystem = new UpperIndexerSubsystem();
      m_robotTurret = new TurretSubsystem();
      m_Limelight = new LimeLightSubsystem(m_robotDrive);
    }

    // Configure the button bindings
    configureButtonBindings();

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
    if (!Constants.kTestMode) {
      new JoystickButton(m_driverController, Button.kR1.value)
          .whileTrue(new RunCommand(
              () -> m_robotDrive.setX(),
              m_robotDrive));

      new JoystickButton(m_driverController, XboxController.Button.kStart.value)
          .onTrue(new InstantCommand(
              () -> m_robotDrive.zeroHeading(),
              m_robotDrive));
    }

    new JoystickButton(m_gunnerController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_robotTurret.lockOntoHub(), m_robotTurret));

    Trigger launchTrigger = new Trigger(this::launchRequested);
    launchTrigger.whileTrue(new InstantCommand(
        () -> m_launcherSubsystem.startLauncher(), m_launcherSubsystem)
        .andThen(new WaitCommand(IndexerConstants.kIndexerDelay))
        .andThen(new InstantCommand(
            () -> m_robotIndexer.startIndexerMotor(), m_robotIndexer)));

    launchTrigger.onFalse(new InstantCommand(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem).andThen(
        () -> m_robotIndexer.stopIndexerMotor(), m_robotIndexer));

    Trigger extendClimberTrigger = new Trigger(this::extendClimberRequested);
    extendClimberTrigger.onTrue(
        new RunCommand(() -> m_robotClimber.extendClimber(0.5), m_robotClimber)
            .withTimeout(5)
            .andThen(() -> m_robotClimber.stopClimber()));
    // DPad Down retracts climber for 5 seconds when pressed
    Trigger retractClimberTrigger = new Trigger(this::retractClimberRequested);
    retractClimberTrigger.onTrue(
        new RunCommand(() -> m_robotClimber.retractClimber(0.5), m_robotClimber)
            .withTimeout(5)
            .andThen(() -> m_robotClimber.stopClimber()));

    // Toggle on release
    new JoystickButton(m_gunnerController, XboxController.Button.kRightBumper.value)
        .onTrue(new RunCommand(() -> m_robotIndexer.buttonPress(), m_robotIndexer));
    new JoystickButton(m_gunnerController, XboxController.Button.kRightBumper.value)
        .onFalse(new RunCommand(() -> m_robotIndexer.buttonRelease(), m_robotIndexer));

    // While the left Dpad is held, the indexer runs in reverse, and when released
    // it goes to positive agin
    Trigger reverseIndexerTrigger = new Trigger(this::reverseIndexerRequested);
    reverseIndexerTrigger.whileTrue(
        new StartEndCommand(
            () -> m_robotIndexer.reverseIndexer(),
            () -> m_robotIndexer.stopIndexerMotor(),
            m_robotIndexer));

    Trigger reverseLauncherTrigger = new Trigger(this::reverseLauncherRequested);
    reverseLauncherTrigger.whileTrue(
        new StartEndCommand(
            () -> m_launcherSubsystem.reverseLauncher(),
            () -> m_launcherSubsystem.stopLauncher(),
            m_launcherSubsystem));

    // The left trigger while held runs the intake rollers
    Trigger intakeTrigger = new Trigger(this::intakeRequested);
    intakeTrigger.onTrue(new InstantCommand(() -> m_robotIntake.startIntakeRollers(), m_robotIntake));
    intakeTrigger.onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeRollers(), m_robotIntake));
    new JoystickButton(m_gunnerController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> m_robotIntake.reverseIntakeRollers(), m_robotIntake));

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
   * RB -> Indexer start/stop (toggle) DONE
   * LeftJoystick ->
   * LeftJoystickClick ->
   * RightJoystick -> Aim launcher
   * RightJoystickClick ->
   * DPad Up -> Climber extend (hold) DONE
   * DPad Down -> Climber retract (hold) DONE
   * DPad Left -> Reverse Indexer (hold)
   * DPad Right -> Reverse launcher (hold)
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

  // Check if DPad Left is pressed on the gunner controller
  public boolean reverseIndexerRequested() {
    return m_gunnerController.getPOV() == 270;
  }

  // Check if DPad Right is pressed on the gunner controller
  public boolean reverseLauncherRequested() {
    return m_gunnerController.getPOV() == 90;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//     return autoChooser.getSelected();

/*
 * Autonomouse to start launcher, move forward 2 feet, then run upper indexer, then run indexer, wait 4 seconds, then stop motors.
 */
// Command m_autonomousCommand;
//     m_autonomousCommand = 
//     // new PathPlannerAuto("Dead Ahead")
//       // .andThen(() -> m_ElevatorSubsystem.goToElevatorL2(), m_ElevatorSubsystem)
//       new InstantCommand(() -> m_launcherSubsystem.startLauncher(), m_launcherSubsystem)
//       // .andthen(m_robotDrive.)
//       .andThen(() -> m_UpperIndexerSubsystem.startUpperIndexerMotor(), m_UpperIndexerSubsystem)
//       .andThen(() -> m_robotIndexer.startIndexerMotor(), m_robotIndexer)
//       .andThen(Commands.waitSeconds(4))
//       .andThen(() -> m_robotIndexer.stopIndexerMotor(), m_robotIndexer)
//       .andThen(() -> m_UpperIndexerSubsystem.stopUpperIndexerMotor(), m_UpperIndexerSubsystem)
//       .andThen(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem);
//     return m_autonomousCommand;

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
