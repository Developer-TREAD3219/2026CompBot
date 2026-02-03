// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.States;



public class LauncherSubsystem extends SubsystemBase {
  /** Creates a new LauncherSubsystem. */

  // Declare the TalonFX motor controller object with a specific CAN ID
  private final TalonFX m_krakenMotorMaster = new TalonFX(LauncherConstants.kLauncherMotorMaster); // Replace 10 with your motor's CAN ID
  private final TalonFX m_krakenMotorFollower = new  TalonFX (LauncherConstants.kLauncherMotorFollower);

  public LauncherSubsystem(XboxController controller, States.State botState) {

    States.State m_botState = botState;
    XboxController m_controller = controller;
    m_krakenMotorFollower.setControl(new Follower(m_krakenMotorMaster.getDeviceID(), MotorAlignmentValue.Opposed));

    // we may not need to use the Config lines
    // Optional: Configure motor inversion if needed
    MotorOutputConfigs motorOutputMaster = new MotorOutputConfigs();
    motorOutputMaster.Inverted = InvertedValue.Clockwise_Positive; // or
    // CounterClockwise_Positive
    m_krakenMotorMaster.getConfigurator().apply(motorOutputMaster);

    MotorOutputConfigs motorOutputFollower = new MotorOutputConfigs();
    motorOutputFollower.Inverted = InvertedValue.Clockwise_Positive;
    m_krakenMotorFollower.getConfigurator().apply(motorOutputFollower);
  }

  public void startLauncher() {
    // Starts the motor to the set value
    System.out.println("Launcher Motor Speed Set to: " + LauncherConstants.kLauncherMotorSpeed);
    m_krakenMotorMaster.set(LauncherConstants.kLauncherMotorSpeed);
  }

  public void stopLauncher() {
    // Stops the motor
    m_krakenMotorMaster.set(0);
  }

  // TO DO: Add method to run launcher in reverse if needed AND CONNECT TO XBOX BUTTON

  public void reverseLauncher() {
    System.out.println("Launcher Motor Reversing");
    m_krakenMotorMaster.set(-LauncherConstants.kLauncherMotorSpeed);
  }

  @Override
  public void periodic() {
    // if (RobotController.getUserButton()) {
    //   startLauncher(LauncherMotorConstants.kLauncherMotorSpeed);
    //   System.out.println("Launcher Motor Running");
    // } else if (!RobotController.getUserButton()) {
    //   System.out.println("Launcher Motor Stopped");
    //   stopLauncher();
    // }
  }
}
