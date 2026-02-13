package frc.robot.subsystems;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Pose3d;
import frc.Constants.OperatorConstants;
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


    // configures the target limelight for MegaTag2 detection
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
    
    // where the tag is relative to the limelight, where the limelight is (0, 0, 0) and the direction it points is the positive x-axis
    Pose3d tagToLimelight = tagPose.relativeTo(limelightPose);
    
    // calculates 2D distance and considers the apriltag visible only if it's within a certain range
    double distance = Math.sqrt(
        tagToLimelight.getX() * tagToLimelight.getX() +
            tagToLimelight.getY() * tagToLimelight.getY());

    if (distance >= OperatorConstants.LIMELIGHT_RANGE)
      return false;

    double relativeAngle = Math.toDegrees(tagToLimelight.getRotation().getAngle());

    if(relativeAngle >= 180)
      relativeAngle -= 180;
    else
      relativeAngle += 180;

    SmartDashboard.putNumber("Tag " + tagId + " Angle", relativeAngle);

    if (relativeAngle < 330 && relativeAngle > 30) {
      return false;
    }

    // calculates whether the apriltag is within the limelight's fov
    double yaw = Math.atan2(tagToLimelight.getY(), tagToLimelight.getX());
    double pitch = Math.atan2(tagToLimelight.getZ(), tagToLimelight.getX());

    double horizontal_fov_radians = Math.toRadians(OperatorConstants.LIMELIGHT_HORIZONTAL_FOV);
    double vertical_fov_radians = Math.toRadians(OperatorConstants.LIMELIGHT_VERTICAL_FOV);

    boolean inFov = Math.abs(yaw) < horizontal_fov_radians / 2 &&
        Math.abs(pitch) < vertical_fov_radians / 2;

    return inFov;
  }


}
