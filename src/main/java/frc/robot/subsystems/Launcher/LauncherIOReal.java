package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.LauncherConstants;
import frc.Constants.LauncherConstants;

public class LauncherIOReal implements LauncherIO {

    // KICKER
    private final TalonFX kickerKraken = new TalonFX(IdConstants.KICK_MOTOR_ID);

    // LAUNCHER
    private final TalonFX launchLeaderKraken = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX launchFollowerKraken = new TalonFX(IdConstants.LAUNCH_FOLLOWER_ID);

    // SENSORS
    private final CANrange feedDetector = new CANrange(IdConstants.FUEL_FEED_SENSOR_ID);

    // CONTROL MODES
    private final VelocityTorqueCurrentFOC launcherControlMode = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final VelocityTorqueCurrentFOC kickerControlMode = new VelocityTorqueCurrentFOC(0).withSlot(0);

    // STATUS SIGNALS
    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();
    
    public LauncherIOReal() {

        TalonFXConfiguration launcherConfigs = new TalonFXConfiguration();
        launcherConfigs.Slot0.kP = LauncherConstants.LAUNCH_kP;
        launcherConfigs.Slot0.kI = LauncherConstants.LAUNCH_kI;
        launcherConfigs.Slot0.kD = LauncherConstants.LAUNCH_kD;
        launcherConfigs.Slot0.kV = Constants.LauncherConstants.LAUNCH_kV;
        launcherConfigs.Slot0.kS = Constants.LauncherConstants.LAUNCH_kS;

        launcherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        launcherConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        launchLeaderKraken.getConfigurator().apply(launcherConfigs);
        launchFollowerKraken.getConfigurator().apply(launcherConfigs);

        TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
        kickerConfigs.Slot0.kP = LauncherConstants.KICKER_kP;
        kickerConfigs.Slot0.kI = LauncherConstants.KICKER_kI;
        kickerConfigs.Slot0.kD = LauncherConstants.KICKER_kD;
        kickerConfigs.Slot0.kV = LauncherConstants.KICKER_kV;
        kickerConfigs.Slot0.kS = LauncherConstants.KICKER_kS;

        kickerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kickerKraken.getConfigurator().apply(kickerConfigs);

    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        
        launchLeaderKraken.setControl(launcherControlMode.withVelocity(shooterRps));
        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), LauncherConstants.isFollowerAligned));

    }

    
    @Override
    public AngularVelocity getLauncherVelocity() {
        return launcherVelocitySignal.refresh().getValue();
    }

    @Override
    public void setKickerVelocity(AngularVelocity kickerRps) {
        kickerKraken.setControl(kickerControlMode.withVelocity(kickerRps));
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
