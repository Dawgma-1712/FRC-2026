package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {

    CommandSwerveDrivetrain drivetrain;
    VisionInterface visionInterface;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = drivetrain;
        if (RobotBase.isReal()) this.visionInterface = new VisionReal(drivetrain);
        else this.visionInterface = new VisionSim(drivetrain);

    }

    @Override
    public void periodic() {

        visionInterface.addVisionMeasurements();

    }

}
