package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionInterface visionIO;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, VisionInterface visionInterface) {
        this.drivetrain = drivetrain;
        this.visionIO = visionInterface;
    }

    @Override
    public void periodic() {
        /*
        // Step 1: Pull latest data from hardware into the inputs object
        visionIO.updateInputs(inputs);
    
        // Step 2: Send inputs to AdvantageKit for logging (and replay injection)
        Logger.processInputs("Vision", inputs);

        // Step 3: Use the (now-logged) inputs to update odometry
        if (inputs.leftHasTarget) {
            drivetrain.addVisionMeasurement(inputs.leftEstimatedPose, inputs.leftTimestamp);
        }
        if (inputs.rightHasTarget) {
            drivetrain.addVisionMeasurement(inputs.rightEstimatedPose, inputs.rightTimestamp);
        }

        // Step 4: Log any outputs you want to track
        Logger.recordOutput("Vision/LeftHasTarget", inputs.leftHasTarget);
        Logger.recordOutput("Vision/RightHasTarget", inputs.rightHasTarget);
        Logger.recordOutput("Vision/LeftPose", inputs.leftEstimatedPose);
        Logger.recordOutput("Vision/RightPose", inputs.rightEstimatedPose);
        */
        visionIO.addVisionMeasurements();
    }
}