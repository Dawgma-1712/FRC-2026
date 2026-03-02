package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.ShooterConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.filter.Debouncer;

public class LauncherIORealBangBang implements LauncherIO {

    // KICKER
    private final TalonFX kickerKraken = new TalonFX(IdConstants.KICK_MOTOR_ID);

    // LAUNCHER
    private final TalonFX launchLeaderKraken = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX launchFollowerKraken = new TalonFX(IdConstants.LAUNCH_FOLLOWER_ID);

    // SENSORS
    private final CANrange feedDetector = new CANrange(IdConstants.FUEL_FEED_SENSOR_ID);

    // CONTROL MODES
    private final VelocityDutyCycle dutyCycleBangBang = new VelocityDutyCycle(0).withSlot(0);
    private final VelocityTorqueCurrentFOC torqueCurrentBangBang = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // STATUS SIGNALS
    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();

    private final Debouncer launcherDebouncer = new Debouncer(ShooterConstants.DEBOUNCE_LENGTH, Debouncer.DebounceType.kRising);
    
    public LauncherIORealBangBang() {

        var config = new TalonFXConfiguration();

        config.Slot0.kP = 999999.0;
        config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;  // it should never go backwards anyways

        launchLeaderKraken.getConfigurator().apply(config);
        launchFollowerKraken.getConfigurator().apply(config);
        kickerKraken.getConfigurator().apply(config);

    }

    private boolean fuelShot(AngularVelocity goalVelocity) {
        // if the velocity drops below the goal velocity outside of the tolerance for a certain amount of time, return true.
        return launcherDebouncer.calculate(
            goalVelocity.in(Units.RotationsPerSecond) - getLauncherVelocity().in(Units.RotationsPerSecond) > ShooterConstants.AT_SPEED_TOLERANCE_RPS
        );
    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        
        // if the velocity is under tolerance, use a duty cycle bang bang and try to get up to speed
        // this will be the same regardless of whether it's trying to get up to speed from rest or recovering from a shot
        if (fuelShot(shooterRps)) {
            launchLeaderKraken.setControl(dutyCycleBangBang.withVelocity(shooterRps));
            launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));
        // if it's up to speed, then use torque current control in preparation for a shot
        // the fuel's only in contact for like 4ms, so by the time we detect that the velocity dropped, the fuel will have already left
        } else {
            launchLeaderKraken.setControl(torqueCurrentBangBang.withVelocity(shooterRps));
            launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));
        }

    }

    
    @Override
    public AngularVelocity getLauncherVelocity() {
        return launcherVelocitySignal.refresh().getValue();
    }

    @Override
    public void setKickerVelocity(AngularVelocity kickerRps) {
        kickerKraken.setControl(torqueCurrentBangBang.withVelocity(kickerRps));
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return kickerVelocitySignal.refresh().getValue();
    }

    @Override
    public boolean getFeedSensor() {
        return feedDetector.getIsDetected().getValue();
    }

}
