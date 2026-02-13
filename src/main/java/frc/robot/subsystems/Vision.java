package frc.robot.subsystems;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import limelight.networktables.LimelightSettings.LEDMode;

import edu.wpi.first.math.geometry.Pose3d;
import frc.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * CURRENTLY NO LOGIC FOR ACTUAL LIMELIGHT
 * Just simulation
 */

public class Vision extends SubsystemBase {
    
    Limelight limelightLeft = new Limelight(VisionConstants.LIMELIGHT_LEFT_ID);
    Limelight limelightRight = new Limelight(VisionConstants.LIMELIGHT_RIGHT_ID);

    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;

    // AdvantageScope stuff
    StructArrayPublisher<Pose3d> aprilTagPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ApriltagPoseArray", Pose3d.struct).publish();

    // configures the target limelight initially, only needed once in the initializer
    private void configureLimelight(Limelight limelight, Pose3d offset) {

      limelight.getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)  // the LEDs will be on or off depending on the pipeline
        .withCameraOffset(offset)  // where the limelight is relative to the robot center
        .save();

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

    public Vision(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = drivetrain;
        this.pigeon = drivetrain.getPigeon2();
        
        configureLimelightMegatag(limelightLeft);
        configureLimelightMegatag(limelightRight);

    }

    /*
     * ADVANTAGESCOPE SIM METHODS
     */
    
    @Override
    public void simulationPeriodic() {

        Pose3d drivetrainPose = new Pose3d(drivetrain.getState().Pose);
        AprilTagFieldLayout aprilTagPoses = VisionConstants.APRIL_TAG_POSES;



    }

  public boolean isAprilTagVisible(Pose3d limelightPose, Pose3d tagPose, int tagId) {

      return true;
  }


}
