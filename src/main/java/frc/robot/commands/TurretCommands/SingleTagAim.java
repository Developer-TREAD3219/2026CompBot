package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States;
import frc.robot.Constants.States.State;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.ZoneData;
import frc.robot.subsystems.LimeLightSubsystem.HubZone;
import frc.robot.utils.LimelightHelpers.RawFiducial;

import java.util.ArrayList;

public class SingleTagAim extends Command {
  private TurretSubsystem m_turret;
  private LimeLightSubsystem m_limelight;
  private LauncherSubsystem m_launcher;
  private LauncherHoodSubsystem m_hood;
  public boolean m_isRed;
  public States m_currentState;

  public SingleTagAim(TurretSubsystem turret, LimeLightSubsystem limelight, States state,
      LauncherSubsystem launcher, LauncherHoodSubsystem hood) {
    m_turret = turret;
    m_limelight = limelight;
    m_launcher = launcher;
    m_hood = hood;
    m_currentState = state;
    m_isRed = m_limelight.isRedAlliance();
    addRequirements(m_turret, m_hood);
  }

  public boolean checkForTag(int tagID) {
    ArrayList<Integer> activeTags = m_limelight.getVisibleAprilTagIDs();
    return activeTags.contains(tagID);
  }

  private boolean leftTagClosest(ZoneData data) {
    double leftAngle = data.leftAngle();
    double rightAngle = data.rightAngle();
    double botAngle = m_limelight.getAngleToHub();
    return (Math.abs(leftAngle - botAngle) < Math.abs(rightAngle - botAngle));
  }

  private void aimAtTag(int tagID) {
    RawFiducial tagData = m_limelight.getRawFiducialById(tagID);
    if (tagData == null) {
      m_turret.turnTurret(0);
      return;
    }
    double tx = tagData.txnc;
    double turnSpeed = m_turret.calculateTurretCommandFromTx(tx);
    m_turret.turnTurret(-turnSpeed); // negate to correct drive direction
  }

  @Override
  public void execute() {
    // Get the calculated motor speed from the subsystem logic
    // ZoneData contains (HubZone zone, double leftAngle, int leftTag, double
    // rightAngle, int rightTag)
    ZoneData data = m_limelight.getHubZoneData();

    // check that we are actually able to score
    if (data.zone() == HubZone.NONE) {
      m_turret.turnTurret(0);
      return;
    }

    // Check if both tags are being tracked
    boolean leftTagAiming = leftTagClosest(data);
    if (leftTagAiming && checkForTag(data.leftTag())) {
      double distance = m_limelight.getDistanceToHub();
      m_launcher.setThrottleForDistance(distance);
      m_hood.setHoodForDistance(distance);
      aimAtTag(data.leftTag());
      m_currentState.setState(State.TargetAcquired); // Set LEDs a color to indicate we have a firing solution - solid
                                                     // yellow
    } else if (!leftTagAiming && checkForTag(data.rightTag())) {
      double distance = m_limelight.getDistanceToHub();
      m_launcher.setThrottleForDistance(distance);
      m_hood.setHoodForDistance(distance);
      aimAtTag(data.rightTag());
      m_currentState.setState(State.TargetAcquired); // Set LEDs a color to indicate we have a firing solution - solid
                                                     // yellow
    } else {
      m_currentState.setState(State.NoTarget); // Set LEDs a color to indicate we don't have a firing solution - solid
                                               // red
    }
  }

  @Override
  public boolean isFinished() {
    // Don't finish on our own — let the trigger (holding Y) control lifetime.
    // While the command is scheduled it will continue to call execute() so
    // the turret keeps updating. The Trigger.whileTrue(...) mapping will
    // cancel this command when the Y button is released. return false;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.turnTurret(0); // Stop the turret when command ends
  }
}
