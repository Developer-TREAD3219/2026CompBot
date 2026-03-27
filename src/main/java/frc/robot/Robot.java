// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.AllianceHelpers;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants;
import com.ctre.phoenix6.SignalLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Boolean m_isInactiveFirst = null;

  public Robot() {
    SignalLogger.stop();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (!Constants.kTestMode) {
      AllianceHelpers.setAllianceColor();
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    AllianceHelpers.setAllianceColor();
    m_isInactiveFirst = null; // Reset so it gets re-read from FMS once available
  }

  @Override
  public void teleopPeriodic() {
    if (m_isInactiveFirst == null) {
      m_isInactiveFirst = AllianceHelpers.isInactiveFirst();
    }
    AllianceHelpers.updateHubStatus(m_isInactiveFirst);
    m_robotContainer.updateRumble();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
