package frc.robot.subsystems.Climber;

public interface ClimberIO {

    public static class ClimberIOInputs{
        double climberPos = 0.0;
    }

    public default void ZeroMotorEncoder(){}

    public default void setClimberPosition(double position){}

    public default double getClimberPosition(){
        return 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {} 

    public default void simPeriodic() {}

}

