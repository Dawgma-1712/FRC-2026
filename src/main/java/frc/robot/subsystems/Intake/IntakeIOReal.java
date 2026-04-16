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

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

     private final SparkMax intakeMotor2 = new SparkMax(IdConstants.INTAKE_MOTOR_2_ID, MotorType.kBrushed);
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
    private boolean runProfile = false;
    private Debouncer armDebouncer = new Debouncer(0.5);

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
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(90, 0);

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
        angleEncoder.setInverted(false);
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

        angleMotor.setControl(new Follower(angleFollowerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.smartCurrentLimit(45);
        intakeConfig.inverted(true);

        SparkMaxConfig intakeConfig2 = new SparkMaxConfig();
        intakeConfig2.idleMode(IdleMode.kCoast);
        intakeConfig2.smartCurrentLimit(45);
        intakeConfig2.inverted(true);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void controlLoop() {

        updateVelocity();  // update the velocity state variable

        currentState = trapezoidProfile.calculate(0.020, currentState, goalState);  // instead of passing the goal position to the pid controller, we can use the trapezoidal profile instead to avoid giant jumps in voltage.
        double pidOutput = -pidController.calculate(getAngle().in(Units.Degrees), currentState.position);

        double positionRadians = Math.toRadians(currentState.position);
        double velocityRadiansPerSecond = Math.toRadians(currentState.velocity);
        double feedforwardOutput = -armFeedforward.calculate(positionRadians, velocityRadiansPerSecond);

        double totalVoltage = pidOutput + feedforwardOutput;

        // voltage clamp
        totalVoltage = Math.max(-12.0, Math.min(12.0, totalVoltage));

        if (runProfile) {
            angleMotor.setControl(voltageRequest.withOutput(totalVoltage));
            angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            if (atAngle()) {
                runProfile = false;
            }
        }

        intakeMotor.set(intakeOutput);
        intakeMotor2.set(intakeOutput);

        SmartDashboard.putNumber("Arm/AngleDeg", getAngle().in(Units.Degrees));
        SmartDashboard.putNumber("Arm/TargetDeg", currentState.position);
        SmartDashboard.putNumber("Arm/GoalDeg", goalState.position);
        SmartDashboard.putNumber("Arm/VelocityDeg_s", currentVelocity);
        SmartDashboard.putNumber("Arm/VoltageOut", totalVoltage);
        SmartDashboard.putNumber("Arm/FF", feedforwardOutput);
        SmartDashboard.putNumber("Arm/PID", pidOutput);
        SmartDashboard.putBoolean("Arm/RunPID", runProfile);
        SmartDashboard.putBoolean("Arm/AtAngle", atAngle());
    }

    @Override
    public void setIntakeMotorSpeed(double percentOutput) {
        intakeOutput = percentOutput;
    }

    @Override
    public void setAngleMotorPercentOutput(double percentOutput) {  //down is positive up is negative
        // if (angleEncoder.get() <= IntakeConstants.STOWED_INTAKE_ANGLE && percentOutput < 0) return;
        // else if (angleEncoder.get() >= IntakeConstants.EXTENDED_INTAKE_ANGLE && percentOutput > 0) return;

        //extended is 90 stowed is 0
        
        angleMotor.set(percentOutput);
        angleFollowerMotor.set(-percentOutput);
    }

    @Override
    public void setAngle(Angle target) {
        double angleClamped = Math.max(IntakeConstants.MIN_ANGLE, Math.min(IntakeConstants.MAX_ANGLE, target.in(Units.Degrees)));
        currentState = new TrapezoidProfile.State(getAngle().in(Units.Degrees), currentVelocity);
        goalState = new TrapezoidProfile.State(angleClamped, 0.0);
        if (atAngle()) { 
            goalState = new TrapezoidProfile.State(getAngle().in(Units.Degrees), 0);
            return;
        }
        runProfile = true;
        pidController.reset();
    }

    @Override
    public void setAngleMotorSupplier(Supplier<Double> stickSupplier) {
        
        double output = stickSupplier.get();
        if (Math.abs(output) < 0.05) {
            return;
        }

        runProfile = false;
        if (output > 0) {
            output *= 0.1;
        } else {
            output *= 0.2;
        }

        setAngleMotorPercentOutput(output);
        holdPosition();
    }

    @Override
    public void holdPosition() {
        setAngle(getAngle());
    }

    public boolean atAngle() {
        return armDebouncer.calculate(Math.abs(getAngle().in(Units.Degrees) - getGoalAngle().in(Units.Degrees)) < 3);
    }

    @Override
    public Angle getAngle() {
        Angle angle = Units.Degrees.of(angleEncoder.get());
        double angleDegs = angle.in(Units.Degrees);
        if (angle.in(Units.Degrees) >= 350) {
            return Units.Degrees.of(-(360-angleDegs));
        }
        return angle;
    }

    @Override
    public Angle getGoalAngle() {
        return Units.Degrees.of(goalState.position);
    }

    private void updateVelocity() {
        double currentAngle = getAngle().in(Units.Degrees);
        currentVelocity = (currentAngle - prevAngle) / 0.020;  // the last measurement was 20ms ago
        prevAngle = currentAngle;
    }
    
}
