package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    private static final double LOOP_PERIOD = 0.020;

    private final SparkMax intakeMotor = new SparkMax(IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    private final SparkMax intakeMotor2 = new SparkMax(IdConstants.INTAKE_MOTOR_2_ID, MotorType.kBrushed);

    private final TalonFX angleMotor = new TalonFX(IdConstants.ANGLE_MOTOR_ID);
    private final TalonFX angleFollowerMotor = new TalonFX(IdConstants.ANGLE_FOLLOWER_MOTOR_ID);

    // REV through bore encoder; offset is the raw reading at our desired zero position
    private final DutyCycleEncoder angleEncoder =
        new DutyCycleEncoder(IdConstants.INTAKE_ANGLE_ENCODER_ID, 360.0, IntakeConstants.ENCODER_OFFSET);

    private final PIDController pidController =
        new PIDController(IntakeConstants.ANGLE_kP, IntakeConstants.ANGLE_kI, IntakeConstants.ANGLE_kD);

    private final ArmFeedforward armFeedforward =
        new ArmFeedforward(IntakeConstants.ANGLE_kS, IntakeConstants.ANGLE_kG, IntakeConstants.ANGLE_kV);

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(IntakeConstants.ANGLE_CRUISE_VELOCITY, IntakeConstants.ANGLE_ACCELERATION));

    private TrapezoidProfile.State currentState = new TrapezoidProfile.State(90, 0);
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(90, 0);

    private final Debouncer armDebouncer = new Debouncer(0.5);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double prevAngle = 0.0;
    private double currentVelocity = 0.0;
    private boolean runProfile = true;
    private double intakeOutput = 0;

    public IntakeIOReal() {
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        angleConfig.CurrentLimits.SupplyCurrentLimit = 40;
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        angleMotor.getConfigurator().apply(angleConfig);
        angleFollowerMotor.getConfigurator().apply(angleConfig);
        angleMotor.setControl(new Follower(angleFollowerMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(45).inverted(true);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void controlLoop() {
        updateVelocity();
        double angle = getAngle().in(Degrees);

        currentState = trapezoidProfile.calculate(LOOP_PERIOD, currentState, goalState);
        double pidOutput = -pidController.calculate(angle, currentState.position);
        double feedforwardOutput = -armFeedforward.calculate(
            Math.toRadians(currentState.position), Math.toRadians(currentState.velocity));
        double totalVoltage = MathUtil.clamp(pidOutput + feedforwardOutput, -12.0, 12.0);

        /*
        if (runProfile) {
            angleMotor.setControl(voltageRequest.withOutput(totalVoltage));
            angleFollowerMotor.setControl(new Follower(angleMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            if (atAngle()) {
                runProfile = false;
            }
        }
        */
        intakeMotor.set(intakeOutput);
        intakeMotor2.set(intakeOutput);

        SmartDashboard.putNumber("Arm/AngleDeg", angle);
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
    public void setAngleMotorPercentOutput(double percentOutput) {  // down is positive, up is negative
        // if (angleEncoder.get() <= IntakeConstants.STOWED_INTAKE_ANGLE && percentOutput < 0) return;
        // else if (angleEncoder.get() >= IntakeConstants.EXTENDED_INTAKE_ANGLE && percentOutput > 0) return;

        // extended is 90, stowed is 0
        double output = Math.abs(percentOutput) < 0.15 ? 0 : percentOutput;
        angleMotor.set(output);
        angleFollowerMotor.set(-output);

        SmartDashboard.putNumber("Arm/GoalPercent", percentOutput);
    }

    @Override
    public void setAngle(Angle target) {
        double angle = getAngle().in(Degrees);
        currentState = new TrapezoidProfile.State(angle, currentVelocity);
        goalState = new TrapezoidProfile.State(
            MathUtil.clamp(target.in(Degrees), IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE), 0);

        if (atAngle()) {
            goalState = new TrapezoidProfile.State(angle, 0);
            return;
        }
        runProfile = true;
        pidController.reset();
    }

    @Override
    public void setAngleMotorSupplier(Supplier<Double> stickSupplier) {
        runProfile = false;
        double output = -stickSupplier.get();
        double angle = getAngle().in(Degrees);

        if (output < 0) {
            output = (angle > 80 && angle < 180) ? 0 : output * 0.75;
        } else {
            output = (angle > 180 && angle < 345) ? 0 : output * 0.65;
        }

        SmartDashboard.putNumber("Intake/Intake goal dutycycle", output);
        setAngleMotorPercentOutput(output);
        holdPosition();
    }

    @Override
    public void holdPosition() {
        setAngle(getAngle());
    }

    public boolean atAngle() {
        return armDebouncer.calculate(Math.abs(getAngle().in(Degrees) - goalState.position) < 3);
    }

    @Override
    public Angle getAngle() {
        double degrees = angleEncoder.get();
        return Degrees.of(degrees >= 350 ? degrees - 360 : degrees);
    }

    @Override
    public Angle getGoalAngle() {
        return Degrees.of(goalState.position);
    }

    private void updateVelocity() {
        double angle = getAngle().in(Degrees);
        currentVelocity = (angle - prevAngle) / LOOP_PERIOD;
        prevAngle = angle;
    }
}
