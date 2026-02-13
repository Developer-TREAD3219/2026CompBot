package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Arrays;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.hardware.Pigeon2;

//import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.Zones;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.AllianceHelpers;

public class LimeLightSubsystem extends SubsystemBase {
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Pigeon2 m_gyro; // Add this line to declare m_gyro
    // Limelight for reading AprilTags
    private final String m_limelightCam = VisionConstants.kCameraName;
    private LimelightHelpers.LimelightResults m_result;
    private LimelightHelpers.LimelightTarget_Fiducial m_currentLock;
    private double m_currentLockDistance = Double.MAX_VALUE;

    // Unsure if we want to use the line on top or leave it like this

    // List of Hub Tags with their respective sides
    private final Set<Integer> RedHubTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private final Set<Integer> BlueHubTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    private final Set<Integer> RedTowerTags = Set.of(15, 16);
    private final Set<Integer> BlueTowerTags = Set.of(31, 32);
    private final Set<Integer> RedOutpostTags = Set.of(13, 14);
    private final Set<Integer> BlueOutpostTags = Set.of(29, 30);
    private final Set<Integer> RedTrenchTags = Set.of(1, 6, 7, 12);
    private final Set<Integer> BlueTrenchTags = Set.of(17, 22, 23, 28);

    private final Map<Zones, Set<Integer>> m_zoneMap = Map.of(
            Zones.RED_HUB, RedHubTags,
            Zones.BLUE_HUB, BlueHubTags,
            Zones.RED_TOWER, RedTowerTags,
            Zones.BLUE_TOWER, BlueTowerTags,
            Zones.RED_OUTPOST, RedOutpostTags,
            Zones.BLUE_OUTPOST, BlueOutpostTags,
            Zones.RED_TRENCH, RedTrenchTags,
            Zones.BLUE_TRENCH, BlueTrenchTags);

    private Map<Zones, Boolean> m_visibleZones = new java.util.HashMap<>();

    private Set<Integer> m_activeHubTags;
    private Set<Integer> m_activetrenchTags;
    private ArrayList<Integer> m_allActiveTags = new ArrayList<>();
    private Set<Integer> m_allValidTagIds = new HashSet<>();
    private String activeAllianceColorHex;

    // Target lock buffer duration: how long we're ok with keeping an old result
    private final double bufferTime = 1.0;
    private double lastUpdateTime = 0.0;

    // Target lock on maxDistance: the farthest away in meters that we want to lock
    // on from
    private final double maxLockOnDistance = 4.0;
    // private final double maxLockOnDistance = 0.5;
    // 0.5 meters = 1.64042 feet

    private void initTagsAndZones() {
        for (Zones zone : Zones.values()) {
            m_visibleZones.put(zone, false);
            m_allValidTagIds.addAll(m_zoneMap.get(zone));
        }
    }

    public LimeLightSubsystem(DriveSubsystem driveSubsystem) {
        // Initialize Limelight settings if needed
        initTagsAndZones();
        detectAprilTags();
        this.m_poseEstimator = null;
        this.m_gyro = null;
        if (driveSubsystem != null) {
            this.m_poseEstimator = driveSubsystem.getPoseEstimator();
            this.m_gyro = driveSubsystem.getGyro(); // Initialize with appropriate parameters
        }

        // Determine alliance color and set HubTags and TrenchTags accordingly
        activeAllianceColorHex = AllianceHelpers.getAllianceColor();
        if (activeAllianceColorHex.equals("#FF0000")) { // Red Alliance
            m_activeHubTags = RedHubTags;
            m_activetrenchTags = RedTrenchTags;
        } else if (activeAllianceColorHex.equals("#0000FF")) { // Blue Alliance
            m_activeHubTags = BlueHubTags;
            m_activetrenchTags = BlueTrenchTags;
        } else {
            m_activeHubTags = null;
            m_activetrenchTags = null;
        }
    }

    public void driverMode() {
        LimelightHelpers.setLEDMode_ForceOff(m_limelightCam);
        LimelightHelpers.setStreamMode_Standard(m_limelightCam);
    }

    public void detectAprilTags() {
        LimelightHelpers.setLEDMode_ForceOn(m_limelightCam);
        LimelightHelpers.setPipelineIndex(m_limelightCam, 0);
    }

    private void printVisibleZones() {
        for (Map.Entry<Zones, Boolean> entry : m_visibleZones.entrySet()) {
            Zones zone = entry.getKey();
            Boolean isVisible = entry.getValue();
            if (isVisible) {
                System.out.println(zone + " is visible");
            }
        }
    }

    private void updateVisibleZones(Set<Integer> detectedTags) {
        for (Map.Entry<Zones, Set<Integer>> entry : m_zoneMap.entrySet()) {
            Zones zone = entry.getKey();
            Set<Integer> tags = entry.getValue();
            if (detectedTags.containsAll(tags)) {
                m_visibleZones.put(zone, true);
            } else {
                m_visibleZones.put(zone, false);
            }
        }
    }

    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        m_result = LimelightHelpers.getLatestResults(m_limelightCam);
        RawFiducial[] tags = LimelightHelpers.getRawFiducials(m_limelightCam);
        m_allActiveTags.clear();
        for (RawFiducial fiducial : tags) {
            m_allActiveTags.add((int) fiducial.id);
        }
        updateVisibleZones(new HashSet<Integer>(m_allActiveTags));
        // printVisibleZones();
        // Clear our lock on if it's been too long
        if (m_currentLock != null && (currentTime - lastUpdateTime > bufferTime)) {
            m_currentLock = null;
            m_currentLockDistance = Double.MAX_VALUE;
        }
        System.out.println("active tags = " + m_allActiveTags);
        // Look for the last result that has a valid target
        if (m_result != null && m_result.targets_Fiducials.length > 0) {
            int nearestTag = m_allActiveTags.get(0);
            for (LimelightHelpers.LimelightTarget_Fiducial target : m_result.targets_Fiducials) {
                if (m_allValidTagIds.contains((int) target.fiducialID) && (int) target.fiducialID == nearestTag) {
                    m_currentLock = target;
                    lastUpdateTime = currentTime;
                    break;
                }
            }
            System.out.println("target found");
        }
        System.out.println(" Yaw=" + getYaw());
        System.out.println(" Skew=" + getSkew());
        System.out.println(" Pitch=" + getPitch());
    }

    // called from Drive subsystem
    //
    public void updateRobotOrientation() {

        // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
        // 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightCam);
        boolean doRejectUpdate = false;

        // if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater
        // than 720 degrees per second, ignore vision updates
        if (m_gyro != null && Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular
                                                                                                    // velocity is
                                                                                                    // greater
        // than 720 degrees per second, ignore
        // vision updates
        {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }
    }

    // public void attemptHubLockon() {
    // System.out.println("fiducials length=" + result.targets_Fiducials.length);
    // if (result != null && result.targets_Fiducials.length > 0) {
    // for (LimelightHelpers.LimelightTarget_Fiducial target :
    // result.targets_Fiducials) {
    // if (Arrays.stream(activeHubTags).anyMatch(id -> id == target.fiducialID)) {
    // double distance = get2dDistance(target);
    // if (distance < maxLockOnDistance && distance < currentLockDistance) {
    // currentLock = target;
    // currentLockDistance = distance;
    // }
    // }
    // }
    // }
    // }

    public int getApriltagID() {
        return m_currentLock != null ? (int) m_currentLock.fiducialID : -1;
    }

    public double getSkew() {
        return m_currentLock != null ? m_currentLock.ts : 0.0;
    }

    public double getYaw() {
        return m_currentLock != null ? m_currentLock.tx : 0.0;
    }

    public double getPitch() {
        return m_currentLock != null ? m_currentLock.ty : 0.0;
    }

    // public boolean isAmbiguousPose() {
    // return currentLock != null && currentLock.ambiguity > 0.1;
    // }

    public double get2dDistance(LimelightTarget_Fiducial target) {
        return target.getTargetPose_RobotSpace().getTranslation().toTranslation2d().getNorm();
    }

    @Override
    public void periodic() {
        // System.out.println(
        // "id=" +
        // NetworkTableInstance.getDefault().getTable(m_limelightCam).getEntry("tid").getInteger(0));
        update();
        if (m_currentLock != null) {
            // SmartDashboard.putNumber("Apriltag ID", getApriltagID());
            // SmartDashboard.putNumber("Skew", getSkew());
            // SmartDashboard.putNumber("Yaw", getYaw());
            // SmartDashboard.putNumber("Pitch", getPitch());
            // // SmartDashboard.putBoolean("Ambiguous Pose", isAmbiguousPose());
            // SmartDashboard.putNumber("Distance", get2dDistance(currentLock));
        }
        // This method will be called once per scheduler run

        // tv int 1 if valid target exists. 0 if no valid targets exist
        // tx double Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27
        // degrees / LL2: -29.8 to 29.8 degrees)klm
        NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraName).getEntry("tv").getDouble(0);
        // double tx =
        // NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraName).getEntry("tx").getDouble(0);
        // double ty =
        // NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraName).getEntry("ty").getDouble(0);
        // double ta =
        // NetworkTableInstance.getDefault().getTable(VisionConstants.kCameraName).getEntry("ta").getDouble(0);

    }

    // NetworkTableInstance.getDefault().getTable("m_limelightCam").getEntry("<variablename>").getDouble(0);
}