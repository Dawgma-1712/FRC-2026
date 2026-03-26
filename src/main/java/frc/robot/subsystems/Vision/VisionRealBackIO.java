package frc.robot.subsystems.Vision;

import limelight.Limelight;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.results.RawFiducial;
import frc.Constants.IdConstants;
import frc.Constants.VisionConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class VisionRealBackIO implements VisionInterface {
    
   Limelight limelightBack = new Limelight(IdConstants.LIMELIGHT_BACK_ID);
private StructPublisher<Pose3d> limelightPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Limelight Pose", Pose3d.struct).publish();


    CommandSwerveDrivetrain drivetrain;
    Pigeon2 pigeon;
    boolean backPoseEstimated = true;

    public VisionRealBackIO(CommandSwerveDrivetrain drivetrain) {

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

    private boolean rejectPose(Pose2d pose) {
        boolean rejectPose =
                // Must be within the field boundaries
                pose.getX() < 0.0
                || pose.getX() > VisionConstants.APRIL_TAG_POSES.getFieldLength()
                || pose.getY() < 0.0
                || pose.getY() > VisionConstants.APRIL_TAG_POSES.getFieldWidth();
        return rejectPose;
    }

    public Pose2d getManualYALLMegaTag2(Limelight ll, Rotation2d gyroYaw, double yawRateDegPerSec) {
        
        this.configureLimelightMegatag(ll);

        // 2. Fetch the raw botpose array manually.
        double[] rawPose = ll.getNTTable().getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

        // 3. Validate data (Array must have at least 8 elements, and index 7 is tag count)
        if (rawPose.length < 8 || rawPose[7] == 0) {
            backPoseEstimated = false;
        }

        Pose2d estimatedPose = new Pose2d(
            new Translation2d(rawPose[0], rawPose[1]),
            Rotation2d.fromDegrees(rawPose[5])
        );

        if (rejectPose(estimatedPose)) {
            backPoseEstimated = false;
        }

        if (backPoseEstimated = false) {
            backPoseEstimated = true;
            SmartDashboard.putBoolean("Vision/Back Pose Estimated", backPoseEstimated);
            return null;
        }

        SmartDashboard.putBoolean("Vision/Back Pose Estimated", backPoseEstimated);

        // MegaTag2 Array Map:
        // [0]x, [1]y, [2]z, [3]roll, [4]pitch, [5]yaw, [6]latency, [7]tagCount...
        return estimatedPose;
    }

    /**
     * Calculates the timestamp for latency compensation manually.
     */
    public static double getMegaTag2Timestamp(Limelight ll) {
        double[] rawPose = ll.getNTTable().getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
        if (rawPose.length < 7) return Timer.getFPGATimestamp();

        // rawPose[6] is total latency in milliseconds
        return Timer.getFPGATimestamp()- (rawPose[6] / 1000.0);
    }

    @Override
    public void addVisionMeasurements() {

        configureLimelightMegatag(limelightBack);

        Pose2d manualPose = getManualYALLMegaTag2(limelightBack, drivetrain.getState().Pose.getRotation(), pigeon.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
        double timestamp = getMegaTag2Timestamp(limelightBack);
        double now = Timer.getFPGATimestamp();

        if (manualPose != null) {
            if (timestamp > 0 && timestamp <= now) {
                System.out.println("adding pose!");
                if (!rejectPose(manualPose)) {
                    drivetrain.addVisionMeasurement(manualPose, timestamp);
                }
            }
            limelightPosePublisher.set(new Pose3d(manualPose));
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

    }
}


