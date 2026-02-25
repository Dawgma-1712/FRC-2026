package frc.robot.subsystems.Revolver;

import edu.wpi.first.units.measure.AngularVelocity;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;

public interface RevolverIO {

    @AutoLog
    public static class RevolverIOInputs {
        public double revolverVelocity = 0;
    }

    public default void setRevolverPercentOutput(double percentOutput) {}

    public default AngularVelocity getRevolverVelocity() {
        return Units.RadiansPerSecond.of(0);
    }

    public default void updateInputs(RevolverIOInputs inputs) {}    

    
}
