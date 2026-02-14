package frc.robot.subsystems.Vision;

import limelight.Limelight;
import limelight.networktables.LimelightSettings.LEDMode;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import frc.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionReal implements VisionInterface {
    
    Limelight limelightLeft = new Limelight(VisionConstants.LIMELIGHT_LEFT_ID);
    Limelight limelightRight = new Limelight(VisionConstants.LIMELIGHT_RIGHT_ID);

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;

    // configures the target limelight initially, only needed once in the initializer
    private void configureLimelight(Limelight limelight, Pose3d offset) {

      limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)  // the LEDs will be on or off depending on the pipeline
        .withCameraOffset(offset)  // where the limelight is relative to the robot center
        .save();

    }

    public VisionReal(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = drivetrain;
        this.pigeon = drivetrain.getPigeon2();

        configureLimelight(limelightLeft, new Pose3d().transformBy(VisionConstants.LIMELIGHT_LEFT_TO_ROBOT));
        configureLimelight(limelightRight, new Pose3d().transformBy(VisionConstants.LIMELIGHT_RIGHT_TO_ROBOT));
    }

    // configures the target limelight for MegaTag2 detection, needed before fetching pose in periodic
    private void configureLimelightMegatag(Limelight limelight) {

        limelight.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    drivetrain.getRotation3d(), 
                    new AngularVelocity3d(
                        pigeon.getAngularVelocityXWorld().getValue(), 
                        pigeon.getAngularVelocityYWorld().getValue(), 
                        pigeon.getAngularVelocityZWorld().getValue())
                )
            )
            .save();
    }


    // gets the MegaTag2 estimate for each limelight and adds it to the odometry
    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelightLeft);
        configureLimelightMegatag(limelightRight);

        Optional<PoseEstimate> visionEstimateLeft = limelightLeft.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
        visionEstimateLeft.ifPresent((PoseEstimate poseEstimate) -> {
            drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        });

        Optional<PoseEstimate> visionEstimateRight = limelightRight.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
        visionEstimateRight.ifPresent((PoseEstimate poseEstimate) -> {
            drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
        });

    }

}


