package frc.robot.subsystems.Vision;

import limelight.Limelight;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
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

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;

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

    private boolean rejectPose(PoseEstimate pose) {
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

    private PoseEstimate betterPoseEstimate(PoseEstimate back, PoseEstimate front) {

        // compare tag counts
        if (back.tagCount != front.tagCount) {
            return back.tagCount > front.tagCount ? back : front;
        }

        // then ambiguity
        if (back.getAvgTagAmbiguity() != front.getAvgTagAmbiguity()) {
            return back.getAvgTagAmbiguity() < front.getAvgTagAmbiguity() ? back : front;
        }

        // then timestamps
        return back.timestampSeconds > front.timestampSeconds ? back : front;
    }

    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelightBack);
        configureLimelightMegatag(limelightFront);

        Optional<PoseEstimate> backPoseEstimate =  estimatorBack.getPoseEstimate().filter(p -> !rejectPose(p));
        Optional<PoseEstimate> frontPoseEstimate =  estimatorFront.getPoseEstimate().filter(p -> !rejectPose(p));
        poseEstimated = true;

        if (backPoseEstimate.isPresent() && frontPoseEstimate.isPresent()) {

            PoseEstimate bestPoseEstimate = betterPoseEstimate(backPoseEstimate.get(), frontPoseEstimate.get());
            drivetrain.addVisionMeasurement(bestPoseEstimate.pose.toPose2d(), bestPoseEstimate.timestampSeconds);

        } else if (backPoseEstimate.isPresent()) {
            
            PoseEstimate poseEstimate = backPoseEstimate.get();
            drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);

        } else if (frontPoseEstimate.isPresent()) {

            PoseEstimate poseEstimate = frontPoseEstimate.get();
            drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);

        } else {
            poseEstimated = false;
        }

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

        SmartDashboard.putBoolean("Vision/Pose Estimated", poseEstimated);
    }
}


