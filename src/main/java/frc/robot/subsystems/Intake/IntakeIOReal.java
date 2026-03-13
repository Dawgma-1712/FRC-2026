package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOReal implements IntakeIO {

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
    private double currentVelocity = 0.0;
    private boolean isProfileRunning = false;

    // what actually controls how much voltage is going to the motors
    private final PIDController pidController = new PIDController(
        IntakeConstants.ANGLE_kP, 
        IntakeConstants.ANGLE_kI, 
        IntakeConstants.ANGLE_kD
    );

    // the thing that makes it move in a smooth motion
    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(
            IntakeConstants.ANGLE_CRUISE_VELOCITY, 
            IntakeConstants.ANGLE_ACCELERATION)  
    );

    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    // it's an arm, so we're going to have to fight gravity as well as a bunch of other factors that need feedforward
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        IntakeConstants.ANGLE_kS, 
        IntakeConstants.ANGLE_kG,
        IntakeConstants.ANGLE_kV
    );

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private double intakeOutput = 0;

    public IntakeIOReal() {
        configureKrakens();
    }

    private void configureKrakens() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        angleMotor.getConfigurator().apply(config);
        angleFollowerMotor.getConfigurator().apply(config);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        angleFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

        angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kCoast);
        hoodConfig.smartCurrentLimit(30); // adjust for bag motor
        hoodConfig.inverted(true);
        intakeMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void controlLoop() {

        updateVelocity();  // update the velocity state variable

        currentState = trapezoidProfile.calculate(0.020, currentState, goalState);  // instead of passing the goal position to the pid controller, we can use the trapezoidal profile instead to avoid giant jumps in voltage.
        double pidOutput = pidController.calculate(getAngle().in(Units.Degrees), currentState.position);

        double positionRadians = Math.toRadians(currentState.position);
        double velocityRadiansPerSecond = Math.toRadians(currentState.velocity);
        double feedforwardOutput = armFeedforward.calculate(positionRadians, velocityRadiansPerSecond);

        double totalVoltage = pidOutput + feedforwardOutput;

        // voltage clamp
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        angleMotor.setControl(voltageRequest.withOutput(totalVoltage));
        angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        intakeMotor.set(intakeOutput);

        SmartDashboard.putNumber("Arm/AngleDeg", getAngle().in(Units.Degrees));
        SmartDashboard.putNumber("Arm/TargetDeg", currentState.position);
        SmartDashboard.putNumber("Arm/GoalDeg", goalState.position);
        SmartDashboard.putNumber("Arm/VelocityDeg_s", currentVelocity);
        SmartDashboard.putNumber("Arm/VoltageOut", totalVoltage);
        SmartDashboard.putNumber("Arm/FF", feedforwardOutput);
        SmartDashboard.putNumber("Arm/PID", pidOutput);
        SmartDashboard.putBoolean("Arm/AtGoal", atGoal());
    }

    public boolean atGoal() {
        return isProfileRunning &&
               Math.abs(getAngle().in(Units.Degrees) - goalState.position) < 1.0 &&   // rn it's 1 degree of tolerance, might need to change that
               Math.abs(currentVelocity) < 2.0;
    }

    @Override
    public void setIntakeMotorSpeed(double percentOutput) {
        intakeOutput = percentOutput;
    }

    @Override
    public void setAngleMotorPercentOutput(double percentOutput) {
        if (angleEncoder.get() >= IntakeConstants.STOWED_INTAKE_ANGLE && percentOutput > 0) return;
        else if (angleEncoder.get() <= IntakeConstants.EXTENDED_INTAKE_ANGLE && percentOutput < 0) return;
        
        angleMotor.set(percentOutput);
        angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setAngle(Angle target) {
        double angleClamped = Math.max(IntakeConstants.MIN_ANGLE, Math.min(IntakeConstants.MAX_ANGLE, target.in(Units.Degrees)));
        currentState = new TrapezoidProfile.State(getAngle().in(Units.Degrees), currentVelocity);
        goalState = new TrapezoidProfile.State(angleClamped, 0.0);

        pidController.reset();
        isProfileRunning = true;
    }

    @Override
    public void holdPosition() {
        setAngle(getAngle());
    }

    @Override
    public Angle getAngle() {
        return Units.Degrees.of(angleEncoder.get());
    }

    private void updateVelocity() {
        double currentAngle = getAngle().in(Units.Degrees);
        currentVelocity = (currentAngle - prevAngle) / 0.020;  // the last measurement was 20ms ago
        prevAngle = currentAngle;
    }
    
}
