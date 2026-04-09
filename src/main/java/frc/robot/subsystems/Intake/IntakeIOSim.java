package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {


    private final PIDController pidController = new PIDController(
        IntakeConstants.ANGLE_kP, 
        IntakeConstants.ANGLE_kI, 
        IntakeConstants.ANGLE_kD
    );

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(
            IntakeConstants.ANGLE_CRUISE_VELOCITY, 
            IntakeConstants.ANGLE_ACCELERATION)  
    );

    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        IntakeConstants.ANGLE_kS, 
        IntakeConstants.ANGLE_kG,
        IntakeConstants.ANGLE_kV
    );

    public IntakeIOSim() {

    }

    @Override
    public void simPeriodic() {
        
        currentState = trapezoidProfile.calculate(0.020, currentState, goalState);  // instead of passing the goal position to the pid controller, we can use the trapezoidal profile instead to avoid giant jumps in voltage.
        double pidOutput = pidController.calculate(getAngle().in(Units.Degrees), currentState.position);

        double positionRadians = Math.toRadians(currentState.position);
        double velocityRadiansPerSecond = Math.toRadians(currentState.velocity);
        double feedforwardOutput = armFeedforward.calculate(positionRadians, velocityRadiansPerSecond);

        double totalVoltage = pidOutput + feedforwardOutput;

        // voltage clamp
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        SmartDashboard.putNumber("Arm/Total Voltage", totalVoltage);

    }

}
