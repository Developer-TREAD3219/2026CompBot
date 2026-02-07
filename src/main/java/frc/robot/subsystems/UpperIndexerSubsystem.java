// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperIndexerConstants;;

public class UpperIndexerSubsystem extends SubsystemBase {
  private SparkMax m_UpperindexerMotor;

  /** UpperIndexerSubsystem. */
  public UpperIndexerSubsystem() {
    System.out.println("Starting Upper Indexer");
    m_UpperindexerMotor = new SparkMax(UpperIndexerConstants.kUpperIndexerCanId, MotorType.kBrushless);
  }

  public void startUpperIndexerMotor() {
    m_UpperindexerMotor.set(UpperIndexerConstants.kUpperIndexerMotorSpeed);
  }

  public void stopUpperIndexerMotor() {
    m_UpperindexerMotor.set(0);
  }

  // Run indexer in reverse if needed AND CONNECT TO XBOX BUTTON
  public void reverseUpperIndexer() {
    m_UpperindexerMotor.set(-UpperIndexerConstants.kUpperIndexerReverseMotorSpeed);
  }

  @Override
  public void periodic() {

  }
}