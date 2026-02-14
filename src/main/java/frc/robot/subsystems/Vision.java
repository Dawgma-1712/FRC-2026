// package frc.robot.subsystems;

// import java.util.Arrays;

// import limelight.Limelight;
// import limelight.networktables.AngularVelocity3d;
// import limelight.networktables.Orientation3d;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.networktables.StructPublisher;
// import limelight.networktables.LimelightSettings.LEDMode;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation3d;

// import frc.Constants.VisionConstants;
// import frc.Constants.VisionConstants.SimulationConstants;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /*
//  * CURRENTLY NO LOGIC FOR ACTUAL LIMELIGHT
//  * Just simulation
//  */

// public class Vision extends SubsystemBase {
    
//     Limelight limelightLeft = new Limelight(VisionConstants.LIMELIGHT_LEFT_ID);
//     Limelight limelightRight = new Limelight(VisionConstants.LIMELIGHT_RIGHT_ID);

//     CommandSwerveDrivetrain drivetrain;
//     Pigeon2 pigeon;

//     // AdvantageScope stuff
//     StructPublisher<Pose3d> limelightLeftPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightLeft", Pose3d.struct).publish();
//     StructPublisher<Pose3d> limelightRightPublisher = NetworkTableInstance.getDefault().getStructTopic("LimelightRight", Pose3d.struct).publish();
//     StructArrayPublisher<Pose3d> aprilTagLeftPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ApriltagPoseArrayLeft", Pose3d.struct).publish();
//     StructArrayPublisher<Pose3d> aprilTagRightPosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("ApriltagPoseArrayRight", Pose3d.struct).publish();


//     // configures the target limelight initially, only needed once in the initializer
//     private void configureLimelight(Limelight limelight, Pose3d offset) {

//       limelight.getSettings()
//         .withLimelightLEDMode(LEDMode.PipelineControl)  // the LEDs will be on or off depending on the pipeline
//         .withCameraOffset(offset)  // where the limelight is relative to the robot center
//         .save();

//     }

//     // configures the target limelight for MegaTag2 detection, needed before fetching pose in periodic
//     private void configureLimelightMegatag(Limelight limelight) {

//         limelight.getSettings()
//             .withRobotOrientation(
//                 new Orientation3d(
//                     drivetrain.getRotation3d(), 
//                     new AngularVelocity3d(
//                         pigeon.getAngularVelocityXWorld().getValue(), 
//                         pigeon.getAngularVelocityYWorld().getValue(), 
//                         pigeon.getAngularVelocityZWorld().getValue())
//                 )
//             )
//             .save();
//     }

//     public Vision(CommandSwerveDrivetrain drivetrain) {

//         this.drivetrain = drivetrain;
//         this.pigeon = drivetrain.getPigeon2();
        
//         configureLimelight(limelightLeft, new Pose3d().transformBy(SimulationConstants.LIMELIGHT_LEFT_TO_ROBOT));
//         configureLimelight(limelightRight, new Pose3d().transformBy(SimulationConstants.LIMELIGHT_RIGHT_TO_ROBOT));

//     }

//     /*
//      * ADVANTAGESCOPE SIM METHODS
//      */
    

//     // buffer arrays to avoid using arraylists
//     private final Pose3d[] visibleTagBufferLeft = new Pose3d[32];
//     private final Pose3d[] visibleTagBufferRight = new Pose3d[32];

//     @Override
//     public void simulationPeriodic() {

//       Pose3d drivetrainPose = new Pose3d(drivetrain.getState().Pose);

//       // field relative limelight positions based on the transforms in Constants
//       Pose3d limelightLeftPose = drivetrainPose.transformBy(SimulationConstants.LIMELIGHT_LEFT_TO_ROBOT);
//       Pose3d limelightRightPose = drivetrainPose.transformBy(SimulationConstants.LIMELIGHT_RIGHT_TO_ROBOT);
      
//       AprilTagFieldLayout aprilTagPoses = VisionConstants.APRIL_TAG_POSES;

//       limelightLeftPublisher.set(limelightLeftPose);
//       limelightRightPublisher.set(limelightRightPose);

//       int visibleCountLeft = 0;
//       int visibleCountRight = 0;
//       // check each aprilTag to see if it's visible
//       for (int i = 1; i <= 32; i++) {
//         Pose3d tagPose = aprilTagPoses.getTagPose(i).orElse(new Pose3d());  // the orElse is because getTagPoses returns an optional, and should never be called if the tagIds are correct

//         if (isAprilTagVisible(limelightLeftPose, tagPose, i)) {
//           visibleTagBufferLeft[visibleCountLeft] = tagPose;
//           visibleCountLeft++;
//         }
//         if (isAprilTagVisible(limelightRightPose, tagPose, i)) {
//           visibleTagBufferRight[visibleCountRight] = tagPose;
//           visibleCountRight++;
//         }
//       }
//       aprilTagLeftPosePublisher.set(Arrays.copyOfRange(visibleTagBufferLeft, 0, visibleCountLeft));
//       aprilTagRightPosePublisher.set(Arrays.copyOfRange(visibleTagBufferRight, 0, visibleCountRight));

//     }

//   public boolean isAprilTagVisible(Pose3d limelightPose, Pose3d tagPose, int tagId) {

//     Pose3d relativeTagPose = tagPose.relativeTo(limelightPose);
//     Translation3d relativeTagTranslation = relativeTagPose.getTranslation();

//     // distance check
//     double distance = relativeTagPose.getTranslation().getNorm();
//     if (distance > SimulationConstants.LIMELIGHT_VIEW_RANGE) return false;

//     // fov check
//     double horizontal = Math.atan2(relativeTagTranslation.getY(), relativeTagTranslation.getX());  // all values are in radians!
//     double vertical = Math.atan2(relativeTagTranslation.getZ(), relativeTagTranslation.getX());

//     if (Math.abs(horizontal) >= SimulationConstants.LIMELIGHT_HORIZONTAL_FOV / 2 ||
//         Math.abs(vertical) >= SimulationConstants.LIMELIGHT_VERTICAL_FOV / 2) {
//           return false;
//     }

//     // orientation check

//     Rotation3d tagRotation = relativeTagPose.getRotation();
//     Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagRotation);
    
//     // vector from tag to limelight
//     Translation3d tagToLimelight = relativeTagTranslation.unaryMinus().div(distance);  // divided by distance to get a length of 1
    
//     // calculate angle between tag normal and direction to limelight
//     double dotProduct = tagNormal.getX() * tagToLimelight.getX() + 
//                         tagNormal.getY() * tagToLimelight.getY() + 
//                         tagNormal.getZ() * tagToLimelight.getZ();
//     double angle = Math.acos(Math.max(-1.0, Math.min(1.0, dotProduct))); // sometimes acos returns values outside of -1, 1
    
//     // check if angle is within 60 degrees
//     if (angle > Math.PI / 3) return false;

//     return true;
//   }

// }
