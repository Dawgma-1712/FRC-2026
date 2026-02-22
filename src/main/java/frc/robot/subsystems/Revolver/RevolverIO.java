package frc.robot.subsystems.Revolver;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.Units;

public interface RevolverIO {

    public static class RevolverIOInputs{
        public double revolverVelocity = 0;
    }

    public default void setRevolverVelocity(AngularVelocity velocity){}

    public default AngularVelocity getRevolverVelocity(){
        return Units.RadiansPerSecond.of(0);
    }

    public default void updateInputs(RevolverIOInputs inputs) {}    

    
}
