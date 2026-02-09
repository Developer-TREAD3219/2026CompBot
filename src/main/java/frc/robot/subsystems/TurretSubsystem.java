// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants; // Assuming you have a Constants file for PID and motor IDs
import frc.robot.utils.LimelightHelpers; // Import the LimelightHelpers class
import frc.robot.subsystems.LimeLightSubsystem;

public class TurretSubsystem extends SubsystemBase {
    // Establish motor controller object for the turret and encoder
    private final SparkMax m_turretMotor = new SparkMax(TurretConstants.kTurretCanId, SparkMax.MotorType.kBrushless);
    private final RelativeEncoder m_turretMotorEncoder = m_turretMotor.getEncoder();

    // A PID controller for the turret's rotational movement
    private final PIDController m_turretPID = new PIDController(
        TurretConstants.kP, TurretConstants.kI, TurretConstants.kD
    );

    public TurretSubsystem() {
        // Configure PID controller for continuous input (turret can spin 360 degrees)
        m_turretPID.enableContinuousInput(-180.0, 180.0); // Adjust limits based on your turret's design and wiring
        // output is clamped in the turnTurret method, so we don't set output range here
        //m_turretPID.setOutputRange(-0.5, 0.5);
    }

    public void turnTurret(double speed) {
        double clampSpeed = MathUtil.clamp(m_turretPID.calculate(0.0, speed), TurretConstants.kLowClamp, TurretConstants.kHighClamp);
        m_turretMotor.set(clampSpeed);
    }

    public double getTx() {
        // Get the horizontal offset from the Limelight (returns 0 if no target)
        
        System.out.println("[TURRET] limelight tx: " + LimelightHelpers.getTX("limelight")); // Debug print to check tx value

        return LimelightHelpers.getTX("limelight"); // Use your Limelight's name if different
    }

    public boolean hasTarget() {
        // Check if the Limelight has a valid target (tv returns 1.0 or 0.0)
        
        System.out.println("[TURRET] limelight tv: " + LimelightHelpers.getTV("limelight")); // Debug print to check tv value

        return LimelightHelpers.getTV("limelight");
    }
  
  public void lockOntoHub(){
    //This function is to lock onto the AprilTag on the Hub
    System.out.println("[TURRET] Set to AprilTag on Hub");
    //LimeLightSubsystem.attemptHubLockon();
    
    //Translation2d m_targetFieldPos = LimeLightSubsystem.getTargetFieldPosition(1); // TO DO: update to use the hub tag ID for the appropriate alliance

    // the commented out code is moved to the LimeLightSubsystem, as it is more appropriate to calculate the target angle there based on the robot's pose and the target's position on the field. The turret subsystem should just receive the desired angle or motor command from the LimeLightSubsystem.
        // // 1. Calculate vector from robot to target
        // Translation2d robotToTarget = m_targetFieldPos.minus(robotPose.getTranslation());
        
        // // 2. Calculate desired angle relative to field (field-oriented)
        // double desiredAngle = Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
        
        // // 3. Adjust for current robot heading (gyro) to get absolute field angle
        // double angleToTarget = desiredAngle - robotPose.getRotation().getDegrees();
        
        // 4. Move turret
        //m_turretMotor.set(m_pid.calculate(m_encoder.getPosition(), angleToTarget));

  } 

    public double calculateTurretCommand() {
        if (hasTarget()) {
            // The 'tx' value is the error (difference from center, in degrees)
            // The PID controller calculates a motor output to make this error zero
            double tx = getTx();
            double output = m_turretPID.calculate(0.0, -tx); // Aiming at tx=0
            
            // Optional: Add a feedforward value to overcome static friction
            // output += Constants.Turret.kS; 

            // Make sure output is within acceptable limits (e.g., -1.0 to 1.0)
            return Math.copySign(Math.min(Math.abs(output), 1.0), output);
        } else {
            // No target, stop the motor or use a default behavior
            return 0.0;
        }
    }

    @Override
    public void periodic() {
        // This is where you might update Shuffleboard/SmartDashboard with data
    }
}





//   public void setAllianceZoneLock() {
//     // Code to set turret to alliance zone lock position
//     System.out.println("[TURRET] Set to Alliance Zone Lock position");
//   }


// public void startStopLauncherMotors(){
//   //Starts or stops the launcher motors
//   System.out.println("[TURRET] Launcher motors started/stopped");
//   }
