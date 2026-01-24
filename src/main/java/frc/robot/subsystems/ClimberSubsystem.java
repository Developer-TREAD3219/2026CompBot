// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

private boolean m_hookLatched = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  public void toggleHookLatch() {
    // Code to toggle the hook latch mechanism
    if (m_hookLatched == true) {
          System.out.println("[CLIMBER] Unlatch hook");
          m_hookLatched = false;
    } else {
          System.out.println("[CLIMBER] Latch hook");
          m_hookLatched = true;
    }
  }
public void extendClimber(double speed) {
    // Code to extend the climber mechanism
    System.out.println("[CLIMBER] Extending climber");
  }
  
  public void retractClimber(double speed) {
    // Code to retract the climber mechanism
    System.out.println("[CLIMBER] Retracting climber");
  }

  public void stopClimber() {
    // Code to stop the climber mechanism // Set Speed to 0
    System.out.println("[CLIMBER] Stopping climber");
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
