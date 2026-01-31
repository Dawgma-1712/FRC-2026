package frc.robot.subsystems.Launcher;

public interface LauncherIO {


    public static class LauncherIOInputs{
        public double shootMotorVelocity = 0;
        public double feedMotorVelocity = 0;

        public boolean hasFuel = false;
    }

    public default void setShooterVelocity(double shooterRPS){}

    public default double getShooter1Velocity(){
        return 0.0;
    }
    
    public default double getShooter2Velocity(){
        return 0.0;
    }

    public default void setFeederVelocity(double feederRPS){}

    public default double getFeederVelocity(){
        return 0.0;
    }

    public default boolean hasFuel(){
        return false;
    }

    public default void updateInputs(LauncherIOInputs inputs) {}    
}
