package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class VisionSubsystemBack extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionInterface visionIO;

    public VisionSubsystemBack(CommandSwerveDrivetrain drivetrain, VisionInterface visionInterface) {
        this.drivetrain = drivetrain;
        this.visionIO = visionInterface;
    }

    @Override
    public void periodic() {
        visionIO.addVisionMeasurements();
    }
}