package frc.robot.subsystems.Intake;

public interface IntakeIO {

    public static class IntakeIOInputs{
        public double angleMotorPosition = 0.0;
    }

    public default void ZeroMotorEncoder(){}

    public default void setIntakeMotorSpeed(double speed){}

    public default void setAngleMotorSpeed(double speed){}

    public default boolean setAngleMotorPosition(double target){
        return false;
    }

    public default double getAngleMotorPosition(){
        return 0.0;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {}    
}
