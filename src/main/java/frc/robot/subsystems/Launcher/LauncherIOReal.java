package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.LauncherConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;

public class LauncherIOReal implements LauncherIO {

    // KICKER
    private final TalonFX kickerKraken = new TalonFX(IdConstants.KICK_MOTOR_ID);

    // LAUNCHER
    private final TalonFX launchLeaderKraken = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX launchFollowerKraken = new TalonFX(IdConstants.LAUNCH_FOLLOWER_ID);

    // HOOD
    private final SparkMax hoodBagMotor = new SparkMax(IdConstants.HOOD_MOTOR_ID, MotorType.kBrushed);
    private final PIDController hoodPidController = new PIDController(LauncherConstants.HOOD_kP, LauncherConstants.HOOD_kI, LauncherConstants.HOOD_kD);
    private final ArmFeedforward hoodFeedforward = new ArmFeedforward(LauncherConstants.HOOD_kS, LauncherConstants.HOOD_kG, LauncherConstants.HOOD_kV);
    private Angle hoodGoalAngle = Units.Degrees.of(0);

    // SENSORS
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID, 360, LauncherConstants.HOOD_ENCODER_OFFSET);

    // CONTROL MODES
    private final VelocityVoltage launcherControlMode = new VelocityVoltage(0);

    // STATUS SIGNALS
    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();
    
    public LauncherIOReal() {

        var config = new TalonFXConfiguration();

        config.Slot0.kP = LauncherConstants.LAUNCH_kP;
        config.Slot0.kI = LauncherConstants.LAUNCH_kI;
        config.Slot0.kD = LauncherConstants.LAUNCH_kD;
        config.Slot0.kV = Constants.LauncherConstants.LAUNCH_kV;
        config.Slot0.kS = Constants.LauncherConstants.LAUNCH_kS;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        launchLeaderKraken.getConfigurator().apply(config);
        launchFollowerKraken.getConfigurator().apply(config);
    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        
        launchLeaderKraken.setControl(launcherControlMode.withVelocity(shooterRps));
        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    @Override
    public void hoodControlLoop() {

        Angle currentAngle = getHoodPosition();

        // if necessary, we can add a trapezoidal profile here too
        double pidOutput = hoodPidController.calculate(currentAngle.in(Units.Degrees), hoodGoalAngle.in(Units.Degrees));

        double hoodAngleRads = Math.toRadians(hoodGoalAngle.in(Units.Degrees));
        double feedforwardOutput = hoodFeedforward.calculate(hoodAngleRads, 0);

        double totalVoltage = pidOutput + feedforwardOutput;
        double clamped = Math.max(-12.0, Math.min(totalVoltage, 12.0));

        hoodBagMotor.setVoltage(clamped);

    }

    @Override
    public void setHoodAngle(Angle goalAngle) {
        hoodGoalAngle = goalAngle;
    }

    @Override
    public Angle getHoodPosition() {
        return Units.Degrees.of(hoodEncoder.get());
    }
    
    @Override
    public AngularVelocity getLauncherVelocity() {
        AngularVelocity vel = launcherVelocitySignal.refresh().getValue();
        return vel;
    }

    @Override
    public void setLauncherPercentOutput(double goalOutput) {
        launchLeaderKraken.set(goalOutput);
    }

    @Override
    public void setKickerPercentOutput(double percentOutput) {
        kickerKraken.set(percentOutput);
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return kickerVelocitySignal.refresh().getValue();
    }

}
