package frc.robot.subsystems.Revolver;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RevolverSubsystem extends SubsystemBase{

    RevolverIO io;

    public RevolverSubsystem(RevolverIO io) {
        this.io = io;
    }

    public void setRevolverPercentOutput(double velocity) {
        io.setRevolverPercentOutput(velocity);
    }

    public AngularVelocity getRevolverVelocity() {
        return io.getRevolverVelocity();
    }

    @Override
    public void periodic() {}

}
