// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// imports for Rev Robotics MaxSwerve
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.TurretCommands.AimTurretLimeLightCommand;
import frc.robot.commands.TurretCommands.AimTurretManualCommand;
import frc.robot.commands.SnapToAngle;
import frc.robot.commands.TurretCommands.SingleTagAim;
import frc.robot.Constants.States;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.commands.TurretCommands.SimpleAim;
import frc.robot.utils.RumbleHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
// other imports
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import frc.robot.Constants.States.State;
import frc.robot.Constants.VisionConstants;

import java.util.Set;

public class RobotContainer {

        public CANBus m_CanBus = new CANBus();
        public Pigeon2 m_Pigeon;
        public States m_currentState;

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
        private final LauncherHoodSubsystem m_launcherHoodSubsystem;

        //All valid Hub ID's
        private static final Set<Integer> HUB_TAG_IDS = Set.of(2, 5, 8, 9, 10, 11, 18, 21, 24, 25, 26, 27);

        //Elastic chooser for autos
        private SendableChooser<Command> m_autoChooser;

        // The driver team controllers
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController m_gunnerController = new CommandXboxController(
                        OIConstants.kGunnerControllerPort);

        private final RumbleHelper m_driverRumble;
        private final RumbleHelper m_gunnerRumble;


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Initialize subsystems
                m_Pigeon = new Pigeon2(Constants.SensorConstants.kPigeonCanId, m_CanBus);
                m_currentState = new States();
                DriverStation.silenceJoystickConnectionWarning(true);

                m_robotDrive = new DriveSubsystem(m_Pigeon);
                m_robotIntake = new IntakeSubsystem();
                m_robotIntakeArm = new IntakeArmSubsystem();
                m_Limelight = new LimeLightSubsystem(m_robotDrive);
                m_robotTurret = new TurretSubsystem(m_Limelight);
                m_robotClimber = new ClimberSubsystem();
                m_launcherSubsystem = new LauncherSubsystem(m_currentState);
                m_robotIndexer = new IndexerSubsystem();
                m_launcherHoodSubsystem = new LauncherHoodSubsystem();

                //set up rumble helpers
                m_driverRumble = new RumbleHelper(m_driverController);
                m_gunnerRumble = new RumbleHelper(m_gunnerController);

                setUpAutoCommands();

                //configure the Auto chooser AFTER we set up the Auto Commands
                m_autoChooser = AutoBuilder.buildAutoChooser("leftAuto");

                configureButtonBindings();
                
                setUpDashboard();

                setUpTriggers();

                // Hood retracts whenever no aiming command is active
                m_launcherHoodSubsystem.setDefaultCommand(
                                new RunCommand(
                                                () -> m_launcherHoodSubsystem.startRetracting(),
                                                m_launcherHoodSubsystem));

                //Default offset. double check but 0 shoulc be the fron to the bot facing away from the drive station
                m_Pigeon.setYaw(0);

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick
                                new RunCommand(
                                                () -> {
                                                        double slow = m_driverController.leftBumper()
                                                                        .getAsBoolean() ? 0.2 : 1.0;

                                                        double xSpeed = -MathUtil.applyDeadband(
                                                                        m_driverController.getLeftY(),
                                                                        OIConstants.kDriveDeadband) * slow;
                                                        double ySpeed = -MathUtil.applyDeadband(
                                                                        m_driverController.getLeftX(),
                                                                        OIConstants.kDriveDeadband) * slow;
                                                        double rot = -MathUtil.applyDeadband(
                                                                        m_driverController.getRightX(),
                                                                        OIConstants.kDriveDeadband) * slow;

                                                        m_robotDrive.drive(xSpeed, ySpeed, rot, true);
                                                },
                                                m_robotDrive));
        
        }

        private void setUpDashboard(){
                //180 flip orientaion button if bot is set up backwards
                SmartDashboard.putData("Flip Heading 180°",
                    new InstantCommand(() -> m_robotDrive.flipHeading(), m_robotDrive)
                        .ignoringDisable(true));

                // zero the heading
                SmartDashboard.putData("Zero Heading",
                        new InstantCommand(() -> m_robotDrive.zeroHeading(),
                                        m_robotDrive)
                                       .ignoringDisable(true));

                //set the autochooser
                SmartDashboard.putData("Auto Chooser", m_autoChooser);
                                       
        }

        private void setUpTriggers(){
            new Trigger(() -> isAnyHubTagVisible())
                .onTrue(new InstantCommand(() -> m_currentState.setState(State.TargetAcquired)))
                .onFalse(new InstantCommand(() -> m_currentState.setState(State.NoTarget)));
        
                // get the time to inactive
                NetworkTableEntry timeToInactive = NetworkTableInstance.getDefault()
                    .getTable("TREAD_Dashboard")
                    .getEntry("timeToInactive");
                
                NetworkTableEntry timeToActive = NetworkTableInstance.getDefault()
                    .getTable("TREAD_Dashboard")
                    .getEntry("timeToActive");

                // Trigger rumble when either transition is within 3 seconds
                new Trigger(() -> {
                    double toInactive = timeToInactive.getDouble(0);
                    double toActive = timeToActive.getDouble(0);
                    return (toInactive > 0 && toInactive <= 3.0) 
                        || (toActive > 0 && toActive <= 3.0);
                })
                .onTrue(new InstantCommand(() -> {
                    m_driverRumble.rumbleForDuration(0.3, 0.7, 3, 0.8);
                    m_gunnerRumble.rumbleForDuration(0.3, 0.7, 3, 0.8);
                }));
                
        }

        public void updateRumble() {
            if (m_driverRumble != null) {
                m_driverRumble.update();
            }
            if (m_gunnerRumble != null) {
                m_gunnerRumble.update();
            }
        }

        private boolean isAnyHubTagVisible() {
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(VisionConstants.kCameraName);
            if (fiducials == null) return false;
        
            for (RawFiducial tag : fiducials) {
                if (HUB_TAG_IDS.contains(tag.id)) {
                    return true;
                }
            }
            return false;
        }
        
        /**
         * Use this method to define your button->command mappings using
         * CommandXboxController's built-in Trigger methods.
         */
        private void configureButtonBindings() {
                /*
                 * DRIVER CONTROLS
                 */

                // Right bumper -> lock wheels in X pattern
                m_driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                if (Constants.kTestMode) {
                        // Start button -> zero heading
                        m_driverController.start()
                                        .onTrue(new InstantCommand(
                                                        () -> m_robotDrive.zeroHeading(),
                                                        m_robotDrive));
                }

                m_driverController.povUp().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(0)));

                m_driverController.povRight().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(-90)));

                m_driverController.povDown().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(180)));

                m_driverController.povLeft().onTrue(
                                new SnapToAngle(
                                                m_robotDrive,
                                                () -> -m_driverController.getLeftY(),
                                                () -> -m_driverController.getLeftX(),
                                                () -> -m_driverController.getRightX(),
                                                Rotation2d.fromDegrees(90)));

                /*
                 * GUNNER CONTROLS
                 */

                // Right bumper (analog) -> run launcher (hold)
                m_gunnerController.rightBumper()
                                .whileTrue(new StartEndCommand(
                                                () -> m_launcherSubsystem.startLauncher(),
                                                () -> m_launcherSubsystem.stopLauncher(),
                                                m_launcherSubsystem));

                // Right bumper -> run indexer (hold)
                // Upper indexer is set to follow lower indexer on the Rev Hardware Client
                m_gunnerController.rightTrigger(OIConstants.kRightTriggerThreshold)
                                .whileTrue(new StartEndCommand(
                                                () -> m_robotIndexer.startIndexerMotor(),
                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                m_robotIndexer));

                // D-Pad Up -> extend climber
                m_gunnerController.povUp().onTrue(
                                new InstantCommand(
                                                () -> m_robotClimber.startExtending(),
                                                m_robotClimber));

                m_gunnerController.povUp().onFalse(
                                new InstantCommand(
                                                () -> m_robotClimber.stopClimber(),
                                                m_robotClimber));
                // D-Pad Down -> retract climber
                m_gunnerController.povDown().onTrue(
                                new InstantCommand(
                                                () -> m_robotClimber.startRetracting(),
                                                m_robotClimber));

                m_gunnerController.povDown().onFalse(
                                new InstantCommand(
                                                () -> m_robotClimber.stopClimber(),
                                                m_robotClimber));

                m_gunnerController.x().onTrue(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stow(),
                                                m_robotIntakeArm));

                m_gunnerController.x().onFalse(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stopArm(),
                                                m_robotIntakeArm));

                m_gunnerController.b().onTrue(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.deploy(),
                                                m_robotIntakeArm));

                m_gunnerController.b().onFalse(
                                new InstantCommand(
                                                () -> m_robotIntakeArm.stopArm(),
                                                m_robotIntakeArm));

                m_gunnerController.y()
                                .whileTrue(
                                                new AimTurretLimeLightCommand(m_robotTurret, m_Limelight,
                                                                m_currentState, m_launcherSubsystem, m_launcherHoodSubsystem));
                m_gunnerController.a().whileTrue(
                                new SingleTagAim(m_robotTurret, m_Limelight, m_currentState, m_launcherSubsystem, m_launcherHoodSubsystem));

                m_gunnerController.povLeft()
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIndexer.reverseIndexer(),
                                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                                m_robotIndexer));

                m_gunnerController.leftTrigger(OIConstants.kLeftTriggerThreshold)
                                .whileTrue(
                                                new StartEndCommand(
                                                                () -> m_robotIntake.startIntakeRollers(),
                                                                () -> m_robotIntake.stopIntakeRollers(),
                                                                m_robotIntake));

                m_gunnerController.leftBumper()
                                .whileTrue(new StartEndCommand(() -> m_robotIntake.reverseIntakeRollers(),
                                        () -> m_robotIntake.stopIntakeRollers(),
                                        m_robotIntake));

                //
                m_robotTurret.setDefaultCommand(new AimTurretManualCommand(
                                m_robotTurret,
                                () -> MathUtil.applyDeadband(m_gunnerController.getLeftX(),
                                                OIConstants.kGunnerDeadBand)));
        }

        private void setUpAutoCommands() {
                NamedCommands.registerCommand("lowerIntake",
                                new InstantCommand(
                                                () -> m_robotIntakeArm.deploy(),
                                                m_robotIntakeArm));

                NamedCommands.registerCommand("startIndexer",
                                new InstantCommand(
                                                () -> m_robotIndexer.startIndexerMotor(),
                                                m_robotIndexer));

                NamedCommands.registerCommand("stopIndexer",
                                new InstantCommand(
                                                () -> m_robotIndexer.stopIndexerMotor(),
                                                m_robotIndexer));

                NamedCommands.registerCommand("startLauncher",
                                new InstantCommand(
                                                () -> m_launcherSubsystem.startLauncher(),
                                                m_launcherSubsystem));

                NamedCommands.registerCommand("stopLauncher",
                                new InstantCommand(
                                                () -> m_launcherSubsystem.stopLauncher(),
                                                m_launcherSubsystem));

                NamedCommands.registerCommand("retractClimber",
                                new InstantCommand(
                                                () -> m_robotClimber.startRetracting(),
                                                m_robotClimber));
                // Set up any autonomous commands or command groups here
        }

        /*
         * Gunner
         * A -> Single tag aim (hold)
         * B -> Arm down (hold)
         * X -> Arm manual up (hold)
         * Y -> Lock onto hub (hold) TO DO
         * LB -> Reverse Intake Rollers (hold)
         * RB -> Launcher on and off (hold)
         * upper and lower indexer motor, instead of just while held
         * LeftJoystick ->
         * LeftJoystickClick ->
         * RightJoystick -> Aim launcher
         * RightJoystickClick ->
         * DPad Up -> Climber extend (hold)
         * DPad Down -> Climber retract (hold)
         * DPad Left -> Reverse Indexer (hold)
         * DPad Right -> Reverse launcher (hold)
         * LeftTrigger -> Start/stop Intake Rollers (toggle)
         * RightTrigger -> Toggle indexers on and off (TOGGLE)
         * launcher motors, instead of just while held
         * StartButton -> Start/Stop launcher motors (toggle)
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

                return m_autoChooser.getSelected();
                
        }
}
