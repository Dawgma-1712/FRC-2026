package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIORealTrapezoidal implements IntakeIO {

    /*
     * MOTORS
     */

    private final SparkMax intakeMotor = new SparkMax(IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);

    private final TalonFX angleMotor = new TalonFX(IdConstants.ANGLE_MOTOR_ID);
    private final TalonFX angleFollowerMotor = new TalonFX(IdConstants.ANGLE_FOLLOWER_MOTOR_ID);

    /*
     * SENSORS
     */

    // REV through bore encoder
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(IdConstants.INTAKE_ANGLE_ENCODER_ID, 
                                                             360.0, 
                                                                       IntakeConstants.ENCODER_OFFSET);  // encoder offset is just the value that the encoder first reads at our desired zero position
    
    /*
     * OTHER
     */

    private double prevAngle = 0.0;

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

    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        IntakeConstants.ANGLE_kS, 
        IntakeConstants.ANGLE_kG,
        IntakeConstants.ANGLE_kV
    );

    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    public IntakeIORealTrapezoidal() {
        configureKrakens();
    }

    private void configureKrakens() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 70;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        angleMotor.getConfigurator().apply(config);
        angleFollowerMotor.getConfigurator().apply(config);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        angleFollowerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setIntakeMotorSpeed(double percentOutput) {
        intakeMotor.set(percentOutput);
    }

    @Override
    public void setAngle(Angle target) {
        double angleClamped = Math.max(IntakeConstants.MIN_ANGLE, Math.min(IntakeConstants.MAX_ANGLE, target.in(Units.Degrees)));
        currentState = new TrapezoidProfile.State(getAngle().in(Units.Degrees), getVelocityDegPerSec());
    }

    @Override
    public Angle getAngle() {
        return Units.Degrees.of(angleEncoder.get());
    }

    @Override //literally useless until we set up advantagekit
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angleMotorPosition = getAngle().in(Units.Degrees);
    }

    private double getVelocityDegPerSec() {
        double current = getAngle().in(Units.Degrees);
        double vel = (current - prevAngle) / 0.020;
        prevAngle = current;
        return vel;
    }
    
}
