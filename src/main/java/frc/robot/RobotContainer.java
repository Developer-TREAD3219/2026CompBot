// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports for Rev Robotics MaxSwerve
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.dPadConstants;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  public Pigeon2 m_Pigeon;
  private States.State m_botState = States.State.Initial;

  // The robot's subsystems and commands are defined here...

  // MOVED TO INSIDE public RobotContainer(), because need to send pigeon as
  // parameter.
  private final DriveSubsystem m_robotDrive;
  private final IntakeSubsystem m_robotIntake;
  private final IntakeArmSubsystem m_robotIntakeArm;
  private final TurretSubsystem m_robotTurret;
  private final ClimberSubsystem m_robotClimber;
  private final LimeLightSubsystem m_Limelight;
  private final IndexerSubsystem m_robotIndexer;
  private final LauncherSubsystem m_launcherSubsystem;
  private final UpperIndexerSubsystem m_UpperIndexerSubsystem;

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController = new CommandXboxController(OIConstants.kGunnerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize subsystems
    m_Pigeon = new Pigeon2(Constants.SensorConstants.kPigeonCanId, m_CanBus);
    System.out.println("m_Pigeon = " + m_Pigeon.getDeviceID());
    m_botState = States.State.Initial;
    if (!Constants.kTestMode) {
      m_robotDrive = new DriveSubsystem(m_Pigeon);
      m_robotIntake = new IntakeSubsystem();
      m_robotIntakeArm = new IntakeArmSubsystem();
      m_robotTurret = new TurretSubsystem();
      m_robotClimber = new ClimberSubsystem();
      m_launcherSubsystem = new LauncherSubsystem(m_botState);
      m_robotIndexer = new IndexerSubsystem();
      m_UpperIndexerSubsystem = new UpperIndexerSubsystem();
      m_Limelight = new LimeLightSubsystem(m_robotDrive);
      configureButtonBindings();
    } else {
      m_robotDrive = null;
      m_robotIntake = null;
      m_robotIntakeArm = null;
      m_robotTurret = null;
      m_robotClimber = null;
      m_launcherSubsystem = null;
      m_robotIndexer = null;
      m_UpperIndexerSubsystem = null;
      m_Limelight = new LimeLightSubsystem(m_robotDrive);
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
   * Use this method to define your button->command mappings using
   * CommandXboxController's built-in Trigger methods.
   */
  private void configureButtonBindings() {
    /*
     * DRIVER CONTROLS
     */
    if (!Constants.kTestMode) {
      // Right bumper -> lock wheels in X pattern
      m_driverController.rightBumper()
          .whileTrue(new RunCommand(
              () -> m_robotDrive.setX(),
              m_robotDrive));

      // Start button -> zero heading
      m_driverController.start()
          .onTrue(new InstantCommand(
              () -> m_robotDrive.zeroHeading(),
              m_robotDrive));
    }

    /*
     * GUNNER CONTROLS
     */

    // Right trigger (analog) -> toggle launcher on/off
    m_gunnerController.rightTrigger(OIConstants.kRightTriggerThreshhold)
        .toggleOnTrue(
            new StartEndCommand(
                () -> m_launcherSubsystem.startLauncher(),
                () -> m_launcherSubsystem.stopLauncher(),
                m_launcherSubsystem));

    // Right trigger release -> ensure launcher and indexer stop
    m_gunnerController.rightTrigger(OIConstants.kRightTriggerThreshhold)
        .onFalse(new InstantCommand(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem)
            .andThen(() -> m_robotIndexer.stopIndexerMotor(), m_robotIndexer));

    // D-Pad Up -> extend climber (with 5-second timeout)
    m_gunnerController.pov(dPadConstants.kDPadUp)
        .onTrue(
            new RunCommand(() -> m_robotClimber.extendClimber(Constants.ClimberConstants.kClimberMotorSpeed),
                m_robotClimber)
                .withTimeout(5)
                .andThen(() -> m_robotClimber.stopClimber()));

    // D-Pad Down -> retract climber (while held)
    m_gunnerController.pov(dPadConstants.kDPadDown)
        .whileTrue(
            new StartEndCommand(
                () -> m_robotClimber.retractClimber(Constants.ClimberConstants.kClimberReverseMotorSpeed),
                () -> m_robotClimber.stopClimber(),
                m_robotClimber));

    // Right bumper -> toggle indexer on/off
    m_gunnerController.rightBumper()
        .toggleOnTrue(new StartEndCommand(
            () -> m_robotIndexer.startIndexerMotor(),
            () -> m_robotIndexer.stopIndexerMotor(),
            m_robotIndexer));

    // A button -> toggle intake arm stow/deploy
    m_gunnerController.a()
        .onTrue(new InstantCommand(() -> m_robotIntakeArm.toggleStowDeploy(),
            m_robotIntakeArm));

    // D-Pad Left -> reverse indexer (while held)
    m_gunnerController.pov(dPadConstants.kDPadLeft)
        .whileTrue(
            new StartEndCommand(
                () -> m_robotIndexer.reverseIndexer(),
                () -> m_robotIndexer.stopIndexerMotor(),
                m_robotIndexer));

    // D-Pad Right -> reverse launcher (while held)
    m_gunnerController.pov(dPadConstants.kDPadRight)
        .whileTrue(
            new StartEndCommand(
                () -> m_launcherSubsystem.reverseLauncher(),
                () -> m_launcherSubsystem.stopLauncher(),
                m_launcherSubsystem));

    // Left trigger -> start/stop intake rollers
    m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshhold)
        .onTrue(new InstantCommand(() -> m_robotIntake.startIntakeRollers(), m_robotIntake));
    m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshhold)
        .onFalse(new InstantCommand(() -> m_robotIntake.stopIntakeRollers(), m_robotIntake));

    // Left bumper -> reverse intake rollers (while held)
    m_gunnerController.leftBumper()
        .whileTrue(new RunCommand(() -> m_robotIntake.reverseIntakeRollers(), m_robotIntake));
  }

  /*
   * Gunner
   * A -> Extend/Retract Intake (toggle) DONE
   * B -> Track outpost with Limelight to shoot into alliance zone (hold) DONE
   * X -> Latch Hook on climber (toggle) DONE
   * Y -> Lock onto hub (hold)
   * LB -> Reverse Intake Rollers (hold)
   * RB -> Toggle indexer on and off (TOGGLE) DONE
   * upper and lower indexer motor, instead of just while held
   * LeftJoystick ->
   * LeftJoystickClick ->
   * RightJoystick -> Aim launcher
   * RightJoystickClick ->
   * DPad Up -> Climber extend (hold) DONE
   * DPad Down -> Climber retract (hold) DONE
   * DPad Left -> Reverse Indexer (hold)
   * DPad Right -> Reverse launcher (hold)
   * LeftTrigger -> Start/stop Intake Rollers (toggle) DONE
   * RightTrigger -> Toggle launchr on and off (TOGGLE) DONE
   * launcher motors, instead of just while held
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // /**
    // * Use this to pass the autonomous command to the main {@link Robot} class.
    // *
    // * @return the command to run in autonomous
    // */
    // return autoChooser.getSelected();

    /*
     * Autonomouse to start launcher, move forward 2 feet, then run upper indexer,
     * then run indexer, wait 4 seconds, then stop motors.
     */
    Command m_autonomousCommand;
    m_autonomousCommand =
        // new PathPlannerAuto("Dead Ahead")
        // .andThen(() -> m_ElevatorSubsystem.goToElevatorL2(), m_ElevatorSubsystem)
        new InstantCommand(() -> m_launcherSubsystem.startLauncher(), m_launcherSubsystem)
            // .andthen(m_robotDrive.)
            .andThen(() -> m_UpperIndexerSubsystem.startUpperIndexerMotor(), m_UpperIndexerSubsystem)
            .andThen(() -> m_robotIndexer.startIndexerMotor(), m_robotIndexer)
            .andThen(Commands.waitSeconds(4))
            .andThen(() -> m_robotIndexer.stopIndexerMotor(), m_robotIndexer)
            .andThen(() -> m_UpperIndexerSubsystem.stopUpperIndexerMotor(), m_UpperIndexerSubsystem)
            .andThen(() -> m_launcherSubsystem.stopLauncher(), m_launcherSubsystem);
    return m_autonomousCommand;

    // this is code from 2025. is it needed/useful?
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // m_robotDrive::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false));
  }

}
