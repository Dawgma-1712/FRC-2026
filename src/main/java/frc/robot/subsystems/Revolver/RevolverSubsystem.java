package frc.robot.subsystems.Revolver;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;



public class RevolverSubsystem extends SubsystemBase{

    private final RevolverIOInputsAutoLogged inputs = new RevolverIOInputsAutoLogged();

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

    public void stop() {
        setRevolverPercentOutput(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Revolver", inputs);


    }

}
