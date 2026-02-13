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
  private AllianceHelpers() {
  }

  // System (FMS) and store variable in network tables as hex, so it can be
  // displayed on the Elastic dashboard using a Color View widget
  public static void setAllianceColor() {
    NetworkTableEntry allianceColorHexEntry = NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("AllianceColorHex");

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // System.out.println("Red Alliance");
        allianceColorHexEntry.setString("#FF0000");
      } else if (ally.get() == Alliance.Blue) {
        // System.out.println("Blue Alliance");
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

  public static Boolean isInactiveFirst() {
    String gameData;
    String currentAllianceColorHex = getAllianceColor();// #FF0000 red, #0000FF Blue
    char currentAllianceColorChar;

    gameData = DriverStation.getGameSpecificMessage();
    if (currentAllianceColorHex.equals("#FF0000")) {
      currentAllianceColorChar = 'R';
    } else if (currentAllianceColorHex.equals("#0000FF")) {
      currentAllianceColorChar = 'B';
    } else {
      currentAllianceColorChar = 'G';
    }

    if (gameData.length() > 0 && currentAllianceColorChar != 'G') {
      char inactiveFirstChar = gameData.charAt(0);
      return currentAllianceColorChar == inactiveFirstChar;
    } else {
      return null;
    }
  }

  public static void updateHubStatus(Boolean isInactiveFirst) {
    // AUTO AUTO 20 Seconds 0:20 – 0:00
    // TELEOP TRANSITION SHIFT 10 Seconds 2:20 – 2:10
    // SHIFT 1 25 Seconds 2:10 – 1:45
    // SHIFT 2 25 Seconds 1:45 – 1:20
    // SHIFT 3 25 Seconds 1:20 – 0:55
    // SHIFT 4 25 Seconds 0:55 – 0:30
    // END GAME 30 Seconds 0:30 – 0:00

    // Get current match time
    double timeRemaining = DriverStation.getMatchTime();
    double timeToInactive = 0;
    double timeToActive = 0;

    // Hub is initially set to active, since it will be active for the 10 second period after autonomous.
    boolean isHubActive = true;
    // Determine the current shift
    boolean isInShift1 = (timeRemaining > 105 && timeRemaining <= 130);
    boolean isInShift2 = (timeRemaining > 80 && timeRemaining <= 105);
    boolean isInShift3 = (timeRemaining > 55 && timeRemaining <= 80);
    boolean isInShift4 = (timeRemaining > 30 && timeRemaining <= 55);

    // If we already have the alliance shift assignment, we don't need to look again.
    // This information isn't available until 3 seconds after autonomous ends.
    if (isInactiveFirst == null) {
      isInactiveFirst = isInactiveFirst();
    }

    if (isInactiveFirst) {
      if (isInShift2) {
        isHubActive = true;
        timeToInactive = timeRemaining - 80;
      }
      else if (isInShift4) {
        isHubActive = true;
        timeToInactive = timeRemaining - 30;
      }
    } else if (!isInactiveFirst) {
      if (isInShift1) {
        isHubActive = true;
        timeToInactive = timeRemaining - 105;
      }
      else if (isInShift3) {
        isHubActive = true;
        timeToInactive = timeRemaining - 55;
      }
    }

    // if (isInactiveFirst && (isInShift2 || isInShift4)) {
    //   isHubActive = true;
    // } else if (!isInactiveFirst && (isInShift1 || isInShift3)) {
    //   isHubActive = true;
    // } else {
    //   isHubActive = false;
    // }

    // Store active status on Network Tables
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("hubStatus").setBoolean(isHubActive);
    // Store time-to-active and time-to-inactive on Network Tables
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToActive").setDouble(timeToActive);
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToInactive").setDouble(timeToInactive);
  }
}
