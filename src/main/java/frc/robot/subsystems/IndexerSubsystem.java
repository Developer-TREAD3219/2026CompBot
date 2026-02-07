// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;;

public class IndexerSubsystem extends SubsystemBase {
  private SparkMax m_indexerMotor;
  private boolean m_indexerOn = false;
  private boolean m_buttonState = false;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    System.out.println("new Indexer");
    m_indexerMotor = new SparkMax(IndexerConstants.kIndexerCanId, MotorType.kBrushless);
  }

  public void buttonPress() {
      m_buttonState = true;
  }

  public void buttonRelease() {
      if (m_buttonState) {
          m_buttonState = false;
          if (m_indexerOn) {
              m_indexerOn = false;
              m_indexerMotor.set(0);
          } else {
              m_indexerOn = true;
              m_indexerMotor.set(IndexerConstants.kIndexerMotorSpeed);
          }
      }
  }

  public void startIndexerMotor() {
    m_buttonState = false;
    m_indexerOn = true;
    m_indexerMotor.set(IndexerConstants.kIndexerMotorSpeed);
  }

  public void stopIndexerMotor() {
    m_buttonState = false;
    m_indexerOn = false;
    m_indexerMotor.set(0);
  }

  // Run indexer in reverse if needed  AND CONNECT TO XBOX BUTTON
  public void reverseIndexer() {
    m_buttonState = false;
    m_indexerOn = true;
    m_indexerMotor.set(-IndexerConstants.kIndexerReverseMotorSpeed);
  }

  @Override
  public void periodic() {

  }
}