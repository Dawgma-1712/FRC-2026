package frc.robot.subsystems.Launcher;

public interface LauncherIO {

    public static class LauncherIOInputs{
        public double kickMotorVelocity = 0;
        public double feedMotorVelocity = 0;
        public double hoodMotorPosition = 0;

        public boolean hasFuelIntaked = false;
        public boolean fuelShot = false;
    }

    public default void setKickerVelocity(double shooterRPS){}

    public default double getKickerVelocity(){
        return 0.0;
    }
    
    public default void setFeederVelocity(double feederRPS){}

    public default double getFeederVelocity(){
        return 0.0;
    }

    public default void setHoodPosition(double position){}

    public default double getHoodPosition(){
        return 0.0;
    }

    public default boolean hasFuelIntaked(){
        return false;
    }

    public default boolean hasShotFuel(){
        return false;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}
    
    public default void periodic() {}
}