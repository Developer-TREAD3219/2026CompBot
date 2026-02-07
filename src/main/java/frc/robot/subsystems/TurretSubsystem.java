// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants; // Assuming you have a Constants file for PID and motor IDs
import frc.robot.utils.LimelightHelpers; // Import the LimelightHelpers class

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
        // TO DO use clamp to Set an output range to prevent excessive speeds during initial tuning
        //m_turretPID.setOutputRange(-0.5, 0.5);
    }

    public void turnTurret(double speed) {
        double clampSpeed = MathUtil.clamp(m_turretPID.calculate(0.0, speed), TurretConstants.kLowClamp, TurretConstants.kHighClamp);
        m_turretMotor.set(clampSpeed);
    }

    public double getTx() {
        // Get the horizontal offset from the Limelight (returns 0 if no target)
        return LimelightHelpers.getTX("limelight"); // Use your Limelight's name if different
    }

    public boolean hasTarget() {
        // Check if the Limelight has a valid target (tv returns 1.0 or 0.0)
        return LimelightHelpers.getTV("limelight");
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
  
//   public void lockOntoHub(){
//     //This function is to lock onto the AprilTag on the Hub
//     System.out.println("[TURRET] Set to AprilTag on Hub");
//   } 

// public void startStopLauncherMotors(){
//   //Starts or stops the launcher motors
//   System.out.println("[TURRET] Launcher motors started/stopped");
//   }
