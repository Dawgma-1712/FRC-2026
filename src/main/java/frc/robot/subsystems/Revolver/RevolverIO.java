package frc.robot.subsystems.Revolver;

public interface RevolverIO {

    public static class RevolverIOInputs{
        public double revolverVelocity = 0;
    }

    public default void setRevolverVelocity(double velocity){}

    public default double getRevolverVelocity(){
        return 0.0;
    }

    public default void updateInputs(RevolverIOInputs inputs) {}    

    
}
