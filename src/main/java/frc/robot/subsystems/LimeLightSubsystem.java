package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Arrays;
import java.util.HashSet;
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
    private final String limelightCam = VisionConstants.kCameraName;
    private LimelightHelpers.LimelightResults result;
    private LimelightHelpers.LimelightTarget_Fiducial currentLock;
    private double currentLockDistance = Double.MAX_VALUE;

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

    private Set<Integer> activeHubTags;
    private Set<Integer> activetrenchTags;
    private Set<Integer> m_allActiveTags = new HashSet<>();
    private String activeAllianceColorHex;

    // Target lock buffer duration: how long we're ok with keeping an old result
    private final double bufferTime = 1.0;
    private double lastUpdateTime = 0.0;

    // Target lock on maxDistance: the farthest away in meters that we want to lock
    // on from
    private final double maxLockOnDistance = 4.0;
    // private final double maxLockOnDistance = 0.5;
    // 0.5 meters = 1.64042 feet

    private void initZones() {
        for (Zones zone : Zones.values()) {
            m_visibleZones.put(zone, false);
        }
    }

    public LimeLightSubsystem(DriveSubsystem driveSubsystem) {
        // Initialize Limelight settings if needed
        initZones();
        this.m_poseEstimator = null;
        this.m_gyro = null;
        if (driveSubsystem != null) {
            this.m_poseEstimator = driveSubsystem.getPoseEstimator();
            this.m_gyro = driveSubsystem.getGyro(); // Initialize with appropriate parameters
        }

        // Determine alliance color and set HubTags and TrenchTags accordingly
        activeAllianceColorHex = AllianceHelpers.getAllianceColor();
        if (activeAllianceColorHex.equals("#FF0000")) { // Red Alliance
            activeHubTags = RedHubTags;
            activetrenchTags = RedTrenchTags;
        } else if (activeAllianceColorHex.equals("#0000FF")) { // Blue Alliance
            activeHubTags = BlueHubTags;
            activetrenchTags = BlueTrenchTags;
        } else {
            activeHubTags = null;
            activetrenchTags = null;
        }
    }

    public void driverMode() {
        LimelightHelpers.setLEDMode_ForceOff(limelightCam);
        LimelightHelpers.setStreamMode_Standard(limelightCam);
    }

    public void detectAprilTags() {
        LimelightHelpers.setLEDMode_ForceOn(limelightCam);
        LimelightHelpers.setPipelineIndex(limelightCam, 0);
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
        result = LimelightHelpers.getLatestResults(limelightCam);
        HashSet<Integer> validTagIDs = new HashSet<>();
        for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
            validTagIDs.add((int) target.fiducialID);
        }
        m_allActiveTags = validTagIDs;
        updateVisibleZones(validTagIDs);
        // Look for the last result that has a valid target
        // number of tags
        System.out.println("Detected tags: " + m_allActiveTags);
        updateVisibleZones(m_allActiveTags);
        printVisibleZones();
        if (result.targets_Fiducials.length == 0)
            return;
        System.out.println(" result = " + result.targets_Fiducials.length);
        System.out.println(" Yaw=" + getYaw());
        System.out.println(" Skew=" + getSkew());
        System.out.println(" Pitch=" + getPitch());

        // Clear our lock on if it's been too long
        if (currentLock != null && (currentTime - lastUpdateTime > bufferTime)) {
            currentLock = null;
            currentLockDistance = Double.MAX_VALUE;
        }
    }

    // called from Drive subsystem
    //
    public void updateRobotOrientation() {
        LimelightHelpers.SetRobotOrientation("limelight",
                m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
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
        return currentLock != null ? (int) currentLock.fiducialID : -1;
    }

    public double getSkew() {
        return currentLock != null ? currentLock.ts : 0.0;
    }

    public double getYaw() {
        return currentLock != null ? currentLock.tx : 0.0;
    }

    public double getPitch() {
        return currentLock != null ? currentLock.ty : 0.0;
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
        // NetworkTableInstance.getDefault().getTable("limelight-tread").getEntry("tid").getInteger(0));
        update();
        if (currentLock != null) {
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
        // degrees / LL2: -29.8 to 29.8 degrees)
        // ty double Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to
        // 20.5 degrees / LL2: -24.85 to 24.85 degrees)
        // double tv =
        NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("tv").getDouble(0);
        // double tx =
        // NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("tx").getDouble(0);
        // double ty =
        // NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("ty").getDouble(0);
        // double ta =
        // NetworkTableInstance.getDefault().getTable("limelightCam").getEntry("ta").getDouble(0);

    }

    // NetworkTableInstance.getDefault().getTable("limeLightCam").getEntry("<variablename>").getDouble(0);
}