// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private boolean m_intakeExtended = false;
  private SparkMax m_intakeMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    System.out.println("new Intake");
    m_intakeMotor = new SparkMax(IntakeConstants.kIntakeCanId, MotorType.kBrushless);
  }

  public void toggleIntakeExtentions() {
    // Code to toggle the intake mechanism
    if (m_intakeExtended == true) {
          System.out.println("[INTAKE] Retract intake");
          m_intakeExtended = false;
    } else {
          System.out.println("[INTAKE] Extend intake");
          m_intakeExtended = true;
    }
  }

public void startIntakeRollers(){
  //Starts the Intake rollers to intake fuel
  System.out.println("[INTAKE] Rollers started");
  m_intakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
}

public void stopIntakeRollers(){
  //Stops the Intake rollers
  m_intakeMotor.set(0);
}

public void reverseIntakeRollers(){
  //Reverses the Intake rollers to get stuff unstuck
  m_intakeMotor.set(-IntakeConstants.kIntakeReverseMotorSpeed);
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
