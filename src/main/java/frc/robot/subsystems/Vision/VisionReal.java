package frc.robot.subsystems.Vision;

import limelight.Limelight;
import limelight.networktables.LimelightSettings.LEDMode;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import frc.Constants.IdConstants;
import frc.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class VisionReal implements VisionInterface {
    
   Limelight limelightBack = new Limelight(IdConstants.LIMELIGHT_BACK_ID);
   Limelight limelightFront = new Limelight(IdConstants.LIMELIGHT_FRONT_ID);

    private final LimelightPoseEstimator estimatorBack;
    private final LimelightPoseEstimator estimatorFront;
    private boolean poseEstimated = false;
    private boolean poseEstimatedFront = false;
    private StructPublisher<Pose3d> limelightPoseEstimationPublisher = NetworkTableInstance.getDefault().getStructTopic("VisionPoseEstimate", Pose3d.struct).publish();
    private StructPublisher<Pose3d> limelightFrontPoseEstimationPublisher = NetworkTableInstance.getDefault().getStructTopic("VisionPoseEstimate/Front", Pose3d.struct).publish();

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

       //configureLimelight(limelightBack, new Pose3d().transformBy(VisionConstants.LIMELIGHT_BACK_TO_ROBOT));
        estimatorBack = limelightBack.createPoseEstimator(EstimationMode.MEGATAG1);
        estimatorFront = limelightFront.createPoseEstimator(EstimationMode.MEGATAG1);
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

    private boolean rejectPose(PoseEstimate pose, boolean back) {
        double ageSeconds = Timer.getFPGATimestamp() - pose.timestampSeconds;
        boolean rejectPose =
            pose.tagCount == 0 // Must have at least one tag
                || ageSeconds > 0.1
                || (pose.tagCount == 1
                    && pose.getAvgTagAmbiguity() > 0.5) // Cannot be high ambiguity
                || Math.abs(pose.pose.getZ())
                    > 1 // Must have realistic Z coordinate

                // Must be within the field boundaries
                || pose.pose.getX() < 0.0
                || pose.pose.getX() > VisionConstants.APRIL_TAG_POSES.getFieldLength()
                || pose.pose.getY() < 0.0
                || pose.pose.getY() > VisionConstants.APRIL_TAG_POSES.getFieldWidth();
        return rejectPose;
    }

    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelightBack);
        configureLimelightMegatag(limelightFront);

        estimatorFront.getPoseEstimate().ifPresentOrElse((PoseEstimate poseEstimate1) -> {
            poseEstimatedFront = true;
            Pose2d newPoseFront = poseEstimate1.pose.toPose2d();
            if (!rejectPose(poseEstimate1, false)) {
                drivetrain.addVisionMeasurement(newPoseFront, poseEstimate1.timestampSeconds); 
            } else {
                poseEstimatedFront = false;
            }
            limelightFrontPoseEstimationPublisher.set(poseEstimate1.pose);
        }, () -> {
            poseEstimatedFront = false;
        });

        // Timmy: Mom...I'm Hungary,
        // Mum: Why don't you Czech the fridge.
        // Timmy: OK. I'm Russian to the kitchen.
        // Mum: Hmmm...Maybe you'll find some Turkey
        // Timmy: Yeah...But its all covered in Greece. Yuck!
        // Mum: There's Norway you can eat that.
        // Timmy: I know, I guess I'll just have a can of Chile
        // Mum: Denmark your name on the can.
        // Timmy: Kenya do it for me?
        // Mum: OK. I'm Ghana do it.
        // Timmy: Thanks, I'm so tired...Iran for an hour today
        // Mum: It Tokyo long enough
        // Timmy: Yeah. Israeli hard sometimes.
        estimatorBack.getPoseEstimate().ifPresentOrElse((PoseEstimate poseEstimateBack) -> {
            poseEstimated = true;
            Pose2d newPose = poseEstimateBack.pose.toPose2d();
            if (!rejectPose(poseEstimateBack, true)) {
                drivetrain.addVisionMeasurement(newPose, poseEstimateBack.timestampSeconds);
            } else {
                poseEstimated = false;
            }
            limelightPoseEstimationPublisher.set(poseEstimateBack.pose);

        }, () -> {
            poseEstimated = false;
        });

        SmartDashboard.putBoolean("Vision/Pose Estimated", poseEstimated);
        SmartDashboard.putBoolean("Vision/Pose Estimated Front", poseEstimatedFront);
    }
}


