package frc.robot.subsystems.Vision;

import limelight.Limelight;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.results.RawFiducial;
import frc.Constants.IdConstants;
import frc.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class VisionReal implements VisionInterface {
    
   Limelight limelight = new Limelight(IdConstants.LIMELIGHT_BACK_ID);

    private double lastFrontTimestamp = 0;
    private boolean poseEstimated = false;

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;

    public VisionReal(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = drivetrain;
        this.pigeon = drivetrain.getPigeon2();

    }

    // configures the target limelight for MegaTag2 detection, needed before fetching pose in periodic
    private void configureLimelightMegatag(Limelight limelight) {

        limelight.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    new Rotation3d(drivetrain.getState().Pose.getRotation()), 
                    new AngularVelocity3d(
                        pigeon.getAngularVelocityXWorld().getValue(), 
                        pigeon.getAngularVelocityYWorld().getValue(), 
                        pigeon.getAngularVelocityZWorld().getValue())
                )
            )
            .save();
    }

    private boolean rejectPose(PoseEstimate pose) {
        boolean rejectPose =
            pose.tagCount == 0 // Must have at least one tag
                || (pose.tagCount == 1
                    && pose.getAvgTagAmbiguity() > 0.5) // Cannot be high ambiguity
                || Math.abs(pose.pose.getZ())
                    > 1 // Must have realistic Z coordinate

                // Must be within the field boundaries
                || pose.pose.getX() < 0.0
                || pose.pose.getX() > VisionConstants.APRIL_TAG_POSES.getFieldLength()
                || pose.pose.getY() < 0.0
                || pose.pose.getY() > VisionConstants.APRIL_TAG_POSES.getFieldWidth();

        /*
        if (rejectPose) {
            if (pose.tagCount == 0) {
                System.out.println("No tags");
            } else if (pose.tagCount == 1 && pose.getAvgTagAmbiguity() > 0.5) System.out.println("Too high ambiguity");
            else if (pose.pose.getX() < 0.0
                || pose.pose.getX() > VisionConstants.APRIL_TAG_POSES.getFieldLength()
                || pose.pose.getY() < 0.0
                || pose.pose.getY() > VisionConstants.APRIL_TAG_POSES.getFieldWidth()) {
                    System.out.println("Outside field");
                }
        }
                */
        return rejectPose;
    }

    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelight);

        Optional<PoseEstimate> frontPoseEstimate = BotPose.BLUE_MEGATAG2.get(limelight).filter(p -> !rejectPose(p));

        if (frontPoseEstimate.isPresent()) {
            SmartDashboard.putBoolean("Vision/Front Estimated", true);
            PoseEstimate poseEstimate = frontPoseEstimate.get();
            drivetrain.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);

        } else {
            SmartDashboard.putBoolean("Vision/Front Estimated", false);
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

        //Isaac: I saudi the problem, and Iran to fix it.

        //Class of 2020: We didnt even get a graduation.

        //Random kid: I thought you said pair of shoes!

        //Finn: What are you doing now did you fix the back limelight
        //Aria: I'm commenting
        //Ayaan: I feel like I'm stuck between Iraq and a hard place.

        //Gas Station: *explodes*
        //CITY BOYYYYYY!!!!!!!!!!!!!!!!


    }
}


