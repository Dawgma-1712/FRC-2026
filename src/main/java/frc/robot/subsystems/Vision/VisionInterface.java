package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionInterface {

    @AutoLog
    public static class VisionIOInputs {
        // Left limelight data
        public boolean leftHasTarget = false;
        public Pose2d leftEstimatedPose = new Pose2d();
        public double leftTimestamp = 0.0;
        public int leftTagCount = 0;

        // Right limelight data
        public boolean rightHasTarget = false;
        public Pose2d rightEstimatedPose = new Pose2d();
        public double rightTimestamp = 0.0;
        public int rightTagCount = 0;
    }

    /** Update the inputs with the latest vision data. */
    public default void updateInputs(VisionIOInputs inputs) {}

    /** Apply vision measurements to the drivetrain odometry. */
    public default void addVisionMeasurements() {}
}