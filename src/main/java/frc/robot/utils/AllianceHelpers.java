// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AllianceHelpers {
    private AllianceHelpers() {}
     // TO DO: Create function to get alliance color tags from field management
  // System (FMS) and store variable in network tables as hex, so it can be
  // displayed on the Elastic dashboard using a Color View widget
  public static void setAllianceColor() {
    NetworkTableEntry allianceColorHexEntry = NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("AllianceColorHex");

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        System.out.println("Red Alliance");
        allianceColorHexEntry.setString("#FF0000");
      } else if (ally.get() == Alliance.Blue) {
        System.out.println("Blue Alliance");
        allianceColorHexEntry.setString("#0000FF");
      }
    } else {
      System.out.println("No Alliance Color Yet");
      allianceColorHexEntry.setString("#808080"); // Default to gray if no alliance
    }
  }

  public static String getAllianceColor() {
    return NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("AllianceColorHex").getString("#808080");
  }
}
