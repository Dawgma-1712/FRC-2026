package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.Constants.VisionConstants;
import frc.Constants.VisionConstants.SimulationConstants;

public class VisionSim implements VisionInterface {
    
    CommandSwerveDrivetrain drivetrain;

    StructPublisher<Pose3d> limelightLeftPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightLeft", Pose3d.struct).publish();
    StructPublisher<Pose3d> limelightRightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightRight", Pose3d.struct).publish();
    StructPublisher<Pose3d> blueHubPublisher = NetworkTableInstance.getDefault().getStructTopic("blueHub", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> aprilTagLeftPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ApriltagPoseArrayLeft", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> aprilTagRightPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ApriltagPoseArrayRight", Pose3d.struct).publish();

    public VisionSim(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // buffer arrays to avoid using arraylists
    private final Pose3d[] visibleTagBufferLeft = new Pose3d[32];
    private final Pose3d[] visibleTagBufferRight = new Pose3d[32];

    @Override
    public void addVisionMeasurements() {

        Pose3d drivetrainPose = new Pose3d(drivetrain.getState().Pose);

        // field relative limelight positions based on the transforms in Constants
        Pose3d limelightLeftPose = drivetrainPose.transformBy(VisionConstants.LIMELIGHT_LEFT_TO_ROBOT);
        Pose3d limelightRightPose = drivetrainPose.transformBy(VisionConstants.LIMELIGHT_RIGHT_TO_ROBOT);

        AprilTagFieldLayout aprilTagPoses = VisionConstants.APRIL_TAG_POSES;

        // this stuff is just so I can see where it thinks the limelights are
        limelightLeftPublisher.set(limelightLeftPose);
        limelightRightPublisher.set(limelightRightPose);

        int visibleCountLeft = 0;
        int visibleCountRight = 0;
        // check each aprilTag to see if it's visible
        for (int i = 1; i <= 32; i++) {
        Pose3d tagPose = aprilTagPoses.getTagPose(i).orElse(new Pose3d());  // the orElse is because getTagPoses returns an optional, and should never be called if the tagIds are correct
        
        // sets the pose in the buffer array to the tag's position
        if (isAprilTagVisible(limelightLeftPose, tagPose, i)) {
            visibleTagBufferLeft[visibleCountLeft] = tagPose;
            visibleCountLeft++;
        }
        if (isAprilTagVisible(limelightRightPose, tagPose, i)) {
            visibleTagBufferRight[visibleCountRight] = tagPose;
            visibleCountRight++;
        }
        }
        aprilTagLeftPosePublisher.set(Arrays.copyOfRange(visibleTagBufferLeft, 0, visibleCountLeft));
        aprilTagRightPosePublisher.set(Arrays.copyOfRange(visibleTagBufferRight, 0, visibleCountRight));

        blueHubPublisher.set(VisionConstants.BLUE_HUB_POSE);
    }

    public boolean isAprilTagVisible(Pose3d limelightPose, Pose3d tagPose, int tagId) {

        Pose3d relativeTagPose = tagPose.relativeTo(limelightPose);
        Translation3d relativeTagTranslation = relativeTagPose.getTranslation();

        // distance check
        double distance = relativeTagPose.getTranslation().getNorm();
        if (distance > SimulationConstants.LIMELIGHT_VIEW_RANGE) return false;

        // fov check
        double horizontal = Math.atan2(relativeTagTranslation.getY(), relativeTagTranslation.getX());  // all values are in radians!
        double vertical = Math.atan2(relativeTagTranslation.getZ(), relativeTagTranslation.getX());

        if (Math.abs(horizontal) >= SimulationConstants.LIMELIGHT_HORIZONTAL_FOV / 2 ||
            Math.abs(vertical) >= SimulationConstants.LIMELIGHT_VERTICAL_FOV / 2) {
                return false;
        }

        // orientation check

        Rotation3d tagRotation = relativeTagPose.getRotation();
        Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagRotation);

        // vector from tag to limelight
        Translation3d tagToLimelight = relativeTagTranslation.unaryMinus().div(distance);  // divided by distance to get a length of 1

        // calculate angle between tag normal and direction to limelight
        double dotProduct = tagNormal.getX() * tagToLimelight.getX() + 
                            tagNormal.getY() * tagToLimelight.getY() + 
                            tagNormal.getZ() * tagToLimelight.getZ();
        double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dotProduct))); // sometimes acos returns values outside of -1, 1

        // check if angle is within 60 degrees
        if (angle > Math.PI / 3) return false;

        return true;
    }

}
