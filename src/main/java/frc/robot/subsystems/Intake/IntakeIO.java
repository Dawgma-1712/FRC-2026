package frc.robot.subsystems.Intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public interface IntakeIO {

    public default void setIntakeMotorSpeed(double percentOutput) {}

    public default void setAngleMotorPercentOutput(double percentOutput) {}

    public default void setAngle(Angle target) {}

    public default Angle getAngle() {
        return Units.Degrees.of(0);
    }

    public default void holdPosition() {}
    
    public default void controlLoop() {}
    
    public default void simPeriodic() {}
}
