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
    double timeRemaining = DriverStation.getMatchTime();
    double timeToInactive = 0;
    double timeToActive = 0;

    boolean isHubActive = false;
    boolean isInShift1 = (timeRemaining > 105 && timeRemaining <= 130);
    boolean isInShift2 = (timeRemaining > 80 && timeRemaining <= 105);
    boolean isInShift3 = (timeRemaining > 55 && timeRemaining <= 80);
    boolean isInShift4 = (timeRemaining > 30 && timeRemaining <= 55);

    if (isInactiveFirst == null) {
        isInactiveFirst = isInactiveFirst();
    }

    // Guard against null if alliance/game data still isn't available
    if (isInactiveFirst == null) {
        isHubActive = true; // default safe value while waiting for data

    } else if (isInactiveFirst) {
        // This alliance is inactive in shifts 1 & 3, active in shifts 2 & 4
        isHubActive = isInShift2 || isInShift4;
        if (isInShift2) {
            timeToInactive = timeRemaining - 80;
            timeToActive = 0;
        } else if (isInShift4) {
            timeToInactive = timeRemaining - 30;
            timeToActive = 0;
        } else if (isInShift1) {
            timeToInactive = 0;
            timeToActive = timeRemaining - 105; // time until shift 2 starts
        } else if (isInShift3) {
            timeToInactive = 0;
            timeToActive = timeRemaining - 55; // time until shift 4 starts
        }

    } else {
        // This alliance is active in shifts 1 & 3, inactive in shifts 2 & 4
        isHubActive = isInShift1 || isInShift3;
        if (isInShift1) {
            timeToInactive = timeRemaining - 105;
            timeToActive = 0;
        } else if (isInShift3) {
            timeToInactive = timeRemaining - 55;
            timeToActive = 0;
        } else if (isInShift2) {
            timeToInactive = 0;
            timeToActive = timeRemaining - 80; // time until shift 3 starts
        } else if (isInShift4) {
            timeToInactive = 0;
            timeToActive = timeRemaining - 30; // never actually becomes active, so 0 might be better here
        }
    }

    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("hubStatus").setBoolean(isHubActive);
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToActive").setDouble(timeToActive);
    NetworkTableInstance.getDefault()
        .getTable("TREAD_Dashboard")
        .getEntry("timeToInactive").setDouble(timeToInactive);
}
}
