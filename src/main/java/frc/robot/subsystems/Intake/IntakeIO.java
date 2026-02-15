package frc.robot.subsystems.Intake;

public interface IntakeIO {

    public static class IntakeIOInputs{
        public double angleMotorPosition = 0.0;
    }

    public default void setIntakeMotorSpeed(double speed) {}

    public default void setAngleMotorSpeed(double speed) {}

    public default void setAngle(double target) {}

    public default double getAngle() {
        return 0.0;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {} 
    
    public default void simPeriodic() {}
}
