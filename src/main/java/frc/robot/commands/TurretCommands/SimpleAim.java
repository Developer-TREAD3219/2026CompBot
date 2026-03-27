package frc.robot.commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import java.util.Set;

public class SimpleAim extends Command {
    private final TurretSubsystem m_turret;
    private final LimeLightSubsystem m_limelight;
    private final LauncherSubsystem m_launcher;
    private final LauncherHoodSubsystem m_hood;

    private static final Set<Integer> HUB_TAG_IDS = Set.of(2, 5, 8, 9, 10, 11, 18, 21, 24, 25, 26, 27);

    public SimpleAim(TurretSubsystem turret, LimeLightSubsystem limelight,
            LauncherSubsystem launcher, LauncherHoodSubsystem hood) {
        m_turret = turret;
        m_limelight = limelight;
        m_launcher = launcher;
        m_hood = hood;
        addRequirements(m_turret, m_hood);
    }

    /**
     * Returns the best visible hub tag by target area (ta).
     * Larger ta = closer/more reliable detection.
     */
    private RawFiducial getBestHubTag() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(VisionConstants.kCameraName);
        if (fiducials == null) return null;

        RawFiducial best = null;
        double bestTa = -1;

        for (RawFiducial tag : fiducials) {
            if (HUB_TAG_IDS.contains(tag.id) && tag.ta > bestTa) {
                bestTa = tag.ta;
                best = tag;
            }
        }

        return best;
    }

    @Override
    public void execute() {
        RawFiducial target = getBestHubTag();

        if (target == null) {
            m_turret.turnTurret(0);
            return;
        }

        double distance = m_limelight.getDistanceToHub();
        m_launcher.setThrottleForDistance(distance);
        m_hood.setHoodForDistance(distance);

        double turnSpeed = m_turret.calculateTurretCommandFromTx(target.txnc);
        m_turret.turnTurret(-turnSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.turnTurret(0);
    }
}
