package frc.robot.subsystems.Revolver;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import frc.Constants.IdConstants;


public class RevolverIOReal implements RevolverIO {

    public final TalonFX revolverMotor = new TalonFX(IdConstants.REVOLVER_MOTOR_ID);

    private final StatusSignal<Current> statorCurrentSignal = revolverMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocitySignal = revolverMotor.getVelocity();

    private final Debouncer stallDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kRising);

    private static final double STALL_CURRENT_THRESHOLD = 40.0; 
    private static final double REVERSE_SPEED = -0.3; 
    private static final double REVERSE_DURATION = 1.0;   

    private boolean isReversing = false;
    private double reverseStartTime = 0.0;

    public RevolverIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        revolverMotor.getConfigurator().apply(config);

       
        statorCurrentSignal.setUpdateFrequency(100.0);
        velocitySignal.setUpdateFrequency(100.0);
        revolverMotor.optimizeBusUtilization();
    }

    @Override
    public void setRevolverPercentOutput(double percentOutput) {
        if (isReversing) return;
        revolverMotor.set(percentOutput);
    }

    @Override
    public AngularVelocity getRevolverVelocity() {
        return velocitySignal.getValue();
    }

    @Override
    public void periodic() {

        double statorCurrent = statorCurrentSignal.getValueAsDouble();

        boolean isOverCurrent = statorCurrent > STALL_CURRENT_THRESHOLD;
        boolean isStalled = stallDebouncer.calculate(isOverCurrent); 

        if (isReversing) {
            if (Timer.getFPGATimestamp() - reverseStartTime >= REVERSE_DURATION) {
                isReversing = false;  
                revolverMotor.set(0.0);
            } else {
                revolverMotor.set(REVERSE_SPEED); 
            }
        } else if (isStalled) {
            isReversing = true;
            reverseStartTime = Timer.getFPGATimestamp();
            revolverMotor.set(REVERSE_SPEED);
        }
    }
}