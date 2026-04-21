package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.States;
import frc.robot.Constants.States.State;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LimeLightSubsystem.ZoneData;
import frc.robot.utils.LimelightHelpers.RawFiducial;

import java.util.ArrayList;

public class AimTurretLimeLightCommand extends Command {
    private TurretSubsystem m_turret;
    private LimeLightSubsystem m_limelight;
    private LauncherSubsystem m_launcher;
    private LauncherHoodSubsystem m_hood;
    public boolean m_isRed;
    public States m_currentState;

    public AimTurretLimeLightCommand(TurretSubsystem turret, LimeLightSubsystem limelight, States state,
            LauncherSubsystem launcher, LauncherHoodSubsystem hood) {
        m_turret = turret;
        m_limelight = limelight;
        m_launcher = launcher;
        m_hood = hood;
        m_currentState = state;
        m_isRed = m_limelight.isRedAlliance();
        addRequirements(m_turret, m_hood);
    }

    public boolean checkForTags(int tag1, int tag2) {
        ArrayList<Integer> activeTags = m_limelight.getVisibleAprilTagIDs();
        return activeTags.contains(tag1) && activeTags.contains(tag2);
    }

    public double interpolate(ZoneData data) {
        double averageAngle = (data.leftAngle() + data.rightAngle()) / 2.0;
        double distance = m_limelight.getDistanceToHub();
        double currentAngle = m_limelight.getAngleToHub();
        double angleInterp = (currentAngle - data.leftAngle()) / (data.rightAngle() - data.leftAngle());
        return MathUtil.clamp(angleInterp, 0, 1);
    }

    public double getInterpolatedTargetTx(ZoneData data) {
        RawFiducial leftTagData = m_limelight.getRawFiducialById(data.leftTag());
        RawFiducial rightTagData = m_limelight.getRawFiducialById(data.rightTag());

        if (leftTagData == null || rightTagData == null) {
            return 0.0;
        }

        double leftTx = leftTagData.txnc;
        double rightTx = rightTagData.txnc;
        double t = interpolate(data);

        return leftTx + t * (rightTx - leftTx);
    }

    private void leftTagAiming(ZoneData data) {

    }

    private void rightTagAiming(ZoneData data) {

    }

    @Override
    public void execute() {
        // Get the calculated motor speed from the subsystem logic
        // ZoneData contains (HubZone zone, double leftAngle, int leftTag, double
        // rightAngle, int rightTag)
        ZoneData data = m_limelight.getHubZoneData();
        // Check if both tags are being tracked
        boolean bothTagsSeen = checkForTags(data.leftTag(), data.rightTag());
        if (bothTagsSeen) {
            m_currentState.setState(State.TargetAcquired);

            double distance = m_limelight.getDistanceToHub();
            m_launcher.setThrottleForDistance(distance);
            m_hood.setHoodForDistance(distance);

            double targetTx = getInterpolatedTargetTx(data);
            double speed = m_turret.calculateTurretCommandFromTx(targetTx);
            m_turret.turnTurret(speed);
        } else { // logic for single tag locks
            ArrayList<Integer> activeTags = m_limelight.getVisibleAprilTagIDs();
            // left tag check
            if (activeTags.contains(data.leftTag())) {
                leftTagAiming(data);
            } else if (activeTags.contains(data.rightTag())) {
                rightTagAiming(data);
            } else {
                m_turret.turnTurret(0);
                m_currentState.setState(State.Initial);
            } // If we don't see both tags, don't move the turret (or you could choose to use
              // one tag if you want)
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