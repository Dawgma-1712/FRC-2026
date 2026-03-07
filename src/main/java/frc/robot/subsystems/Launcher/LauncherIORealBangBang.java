package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.Constants.IdConstants;
import frc.Constants.LauncherConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;

public class LauncherIORealBangBang implements LauncherIO {

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
    private final CANrange feedDetector = new CANrange(IdConstants.FUEL_FEED_SENSOR_ID);
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID, 360, LauncherConstants.HOOD_ENCODER_OFFSET);

    // CONTROL MODES
    private final VelocityDutyCycle dutyCycleBangBang = new VelocityDutyCycle(0).withSlot(0);
    private final VelocityTorqueCurrentFOC torqueCurrentBangBang = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // STATUS SIGNALS
    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();

    private final Debouncer launcherDebouncer = new Debouncer(LauncherConstants.DEBOUNCE_LENGTH, Debouncer.DebounceType.kRising);
    
    public LauncherIORealBangBang() {

        var config = new TalonFXConfiguration();

        config.Slot0.kP = 999999.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;  // it should never go backwards anyways
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        launchLeaderKraken.getConfigurator().apply(config);
        launchFollowerKraken.getConfigurator().apply(config);

        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        
        // if the velocity is under tolerance, use a duty cycle bang bang and try to get up to speed
        // this will be the same regardless of whether it's trying to get up to speed from rest or recovering from a shot
        if (fuelShot(shooterRps)) {
            launchLeaderKraken.setControl(dutyCycleBangBang.withVelocity(shooterRps));
        // if it's up to speed, then use torque current control in preparation for a shot
        // the fuel's only in contact for like 4ms, so by the time we detect that the velocity dropped, the fuel will have already left
        } else {
            launchLeaderKraken.setControl(torqueCurrentBangBang.withVelocity(shooterRps));
        }

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
    public boolean getFeedSensor() {
        return feedDetector.getIsDetected().getValue();
    }

    private boolean fuelShot(AngularVelocity goalVelocity) {
        // Because the command scheduler is so long (20 ms) when compared to the time the fuel is in contact with the flywheel (like 4 ms?),
        // I think it's reasonable to assume that any drop in velocity will be detected after the shot has already.
        return launcherDebouncer.calculate(
            goalVelocity.in(Units.RotationsPerSecond) - getLauncherVelocity().in(Units.RotationsPerSecond) > LauncherConstants.AT_SPEED_TOLERANCE_RPS
        );
    }


}
