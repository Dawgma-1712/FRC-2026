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

    private final LimelightPoseEstimator estimatorBack;
    private boolean poseEstimated = false;
   private StructPublisher<Pose3d> limelightPoseEstimationPublisher = NetworkTableInstance.getDefault().getStructTopic("VisionPoseEstimate", Pose3d.struct).publish();

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



    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelightBack);

        estimatorBack.getPoseEstimate().ifPresentOrElse((PoseEstimate poseEstimate) -> {
            poseEstimated = true;
            limelightPoseEstimationPublisher.set(poseEstimate.pose);
            boolean rejectPose =
            poseEstimate.tagCount == 0 // Must have at least one tag
                || (poseEstimate.tagCount == 1
                    && poseEstimate.getAvgTagAmbiguity() > 0.5) // Cannot be high ambiguity
                || Math.abs(poseEstimate.pose.getZ())
                    > 1 // Must have realistic Z coordinate

                // Must be within the field boundaries
                || poseEstimate.pose.getX() < 0.0
                || poseEstimate.pose.getX() > VisionConstants.APRIL_TAG_POSES.getFieldLength()
                || poseEstimate.pose.getY() < 0.0
                || poseEstimate.pose.getY() > VisionConstants.APRIL_TAG_POSES.getFieldWidth();
            
            //Pose2d newPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? poseEstimate.pose.toPose2d() : poseEstimate.pose.toPose2d().rotateBy(new Rotation2d(Units.Degrees.of(180)));
            Pose2d newPose = poseEstimate.pose.toPose2d();
            if (!rejectPose) drivetrain.addVisionMeasurement(newPose, poseEstimate.timestampSeconds);
        }, () -> {
            poseEstimated = false;
        });

        SmartDashboard.putBoolean("Vision/Pose Estimated", poseEstimated);

    }

}


