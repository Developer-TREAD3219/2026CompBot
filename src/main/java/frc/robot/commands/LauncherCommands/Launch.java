// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LauncherCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  LauncherSubsystem launcherSubsystem;

  /** Creates a new Launch. */
  public Launch() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     launcherSubsystem.startLauncher();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.stopLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// May be usefull later since its has a lot of things.
// // Simplified Example: Turret Aiming & Shooting Logic
// public class Turret {
//     // Assume you have hardware interfaces for turret motor, shooter motor, and vision
//     private TurretMotor turretMotor;
//     private Shooter flywheel;
//     private VisionSystem vision; // e.g., PhotonVision, Limelight

//     public void periodic() {
//         // 1. Get target data from vision system
//         Target target = vision.getTarget(); // Returns target's x, y, distance

//         if (target != null) {
//             // 2. Calculate necessary turret angle (yaw) and shooter speed
//             double targetYaw = calculateYawToTarget(target); // Use PID for this
//             double shooterVelocity = calculateShooterVelocity(target.distance); // Use PID/Feedforward

//             // 3. Set hardware commands
//             turretMotor.setAngle(targetYaw); // Rotate turret
//             flywheel.setTargetVelocity(shooterVelocity); // Spin up shooter

//             // 4. Check if ready to fire
//             if (isTurretAtAngle(targetYaw) && flywheel.isReady()) {
//                 fire();
//             }
//         } else {
//             // No target, stop motors or return to default position
//             turretMotor.stop();
//             flywheel.stop();
//         }
//     }

//     private void fire() {
//         // Trigger projectile release (e.g., Pneumatics, second motor)
//         // Add delay/reload logic here
//     }