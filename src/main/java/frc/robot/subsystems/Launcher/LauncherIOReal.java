package frc.robot.subsystems.Launcher;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants.IdConstants;
import frc.Constants.LauncherConstants;

public class LauncherIOReal implements LauncherIO {

    private static final double HOOD_GEAR_RATIO = 17.0;  // encoder shaft rotations per hood rotation
    private static final double MAX_HOOD_ANGLE = 30.0;

    private final TalonFX kickerKraken = new TalonFX(IdConstants.KICK_MOTOR_ID);
    private final TalonFX launchLeaderKraken = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX launchFollowerKraken = new TalonFX(IdConstants.LAUNCH_FOLLOWER_ID);
    private final SparkMax hoodBagMotor = new SparkMax(IdConstants.HOOD_MOTOR_ID, MotorType.kBrushed);

    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID, 360, 0.0);

    private final PIDController hoodPidController =
        new PIDController(LauncherConstants.HOOD_kP, LauncherConstants.HOOD_kI, LauncherConstants.HOOD_kD);

    private final VelocityVoltage launcherControlMode = new VelocityVoltage(0);

    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();

    private double lastRawAngle = 0.0;
    private double cumulativeEncoderDegrees = 0.0;
    private boolean hoodZeroed = false;
    private Angle hoodGoalAngle = Degrees.of(0);

    public LauncherIOReal() {
        TalonFXConfiguration launchConfig = new TalonFXConfiguration();
        launchConfig.CurrentLimits.SupplyCurrentLimit = 50;
        launchConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        launchConfig.Slot0.kP = LauncherConstants.LAUNCH_kP;
        launchConfig.Slot0.kI = LauncherConstants.LAUNCH_kI;
        launchConfig.Slot0.kD = LauncherConstants.LAUNCH_kD;
        launchConfig.Slot0.kV = LauncherConstants.LAUNCH_kV;
        launchConfig.Slot0.kS = LauncherConstants.LAUNCH_kS;
        launchConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        launchLeaderKraken.getConfigurator().apply(launchConfig);
        launchFollowerKraken.getConfigurator().apply(launchConfig);
        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        hoodBagMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerKraken.optimizeBusUtilization();
    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        launchLeaderKraken.setControl(launcherControlMode.withVelocity(shooterRps));
    }

    @Override
    public void setLauncherPercentOutput(double goalOutput) {
        launchLeaderKraken.set(goalOutput);
    }

    @Override
    public AngularVelocity getLauncherVelocity() {
        return launcherVelocitySignal.refresh().getValue();
    }

    @Override
    public void setKickerPercentOutput(double percentOutput) {
        kickerKraken.set(percentOutput);
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return kickerVelocitySignal.refresh().getValue();
    }

    @Override
    public void hoodControlLoop() {
        if (!hoodZeroed) {
            // No valid encoder reading yet, so getHoodPosition() would report a
            // constant 0 and the PID would drive the hood open-loop into a hard stop.
            hoodBagMotor.setVoltage(0);
            return;
        }

        double pidOutput = hoodPidController.calculate(getHoodPosition().in(Degrees), hoodGoalAngle.in(Degrees));
        double clamped = MathUtil.clamp(pidOutput, -12.0, 12.0);

        SmartDashboard.putNumber("Launcher/Hood PID Output", pidOutput);
        SmartDashboard.putNumber("Launcher/Clamped Voltage", clamped);
        hoodBagMotor.setVoltage(clamped);
    }

    @Override
    public void setHoodAngle(Angle goalAngle) {
        hoodGoalAngle = goalAngle.in(Degrees) > MAX_HOOD_ANGLE ? Degrees.of(MAX_HOOD_ANGLE) : goalAngle;
    }

    @Override
    public Angle getHoodPosition() {
        return Degrees.of(-cumulativeEncoderDegrees / HOOD_GEAR_RATIO);
    }

    /**
     * Accumulates hood encoder motion. Must be called exactly once per robot loop,
     * before hoodControlLoop().
     */
    @Override
    public void periodic() {
        boolean connected = hoodEncoder.isConnected();
        SmartDashboard.putBoolean("Launcher/Hood Encoder Connected", connected);
        if (!connected) {
            return;
        }

        double currentRaw = hoodEncoder.get();
        if (hoodZeroed) {
            cumulativeEncoderDegrees += MathUtil.inputModulus(currentRaw - lastRawAngle, -180, 180);
        } else {
            hoodZeroed = true;
        }
        lastRawAngle = currentRaw;
    }
}
