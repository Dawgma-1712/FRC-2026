package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.RobotBase;

public class VisionSubsystem extends SubsystemBase {

    CommandSwerveDrivetrain drivetrain;
    VisionInterface visionInterface;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, VisionInterface visionInterface) {
        this.drivetrain = drivetrain;
        this.visionInterface = visionInterface;
    }

    @Override
    public void periodic() {

        visionInterface.addVisionMeasurements();

    }

}
