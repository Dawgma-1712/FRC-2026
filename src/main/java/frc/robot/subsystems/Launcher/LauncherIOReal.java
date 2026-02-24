package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.ShooterConstants;

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
    
    public LauncherIOReal() {

        TalonFXConfiguration launcherConfigs = new TalonFXConfiguration();
        launcherConfigs.Slot0.kP = ShooterConstants.LAUNCH_kP;
        launcherConfigs.Slot0.kI = ShooterConstants.LAUNCH_kI;
        launcherConfigs.Slot0.kD = ShooterConstants.LAUNCH_kD;
        launcherConfigs.Slot0.kV = Constants.ShooterConstants.LAUNCH_kV;
        launcherConfigs.Slot0.kS = Constants.ShooterConstants.LAUNCH_kS;

        launcherConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        launcherConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        launchLeaderKraken.getConfigurator().apply(launcherConfigs);
        launchFollowerKraken.getConfigurator().apply(launcherConfigs);

        TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
        kickerConfigs.Slot0.kP = ShooterConstants.KICKER_kP;
        kickerConfigs.Slot0.kI = ShooterConstants.KICKER_kI;
        kickerConfigs.Slot0.kD = ShooterConstants.KICKER_kD;
        kickerConfigs.Slot0.kV = Constants.ShooterConstants.KICKER_kV;
        kickerConfigs.Slot0.kS = Constants.ShooterConstants.KICKER_kS;

        kickerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kickerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kickerKraken.getConfigurator().apply(kickerConfigs);

    }

    @Override
    public void setLauncherVelocity(AngularVelocity shooterRps) {
        
        launchLeaderKraken.setControl(launcherControlMode.withVelocity(shooterRps));
        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), ShooterConstants.isFollowerAligned));

    }

    @Override
    public AngularVelocity getLauncherVelocity() {
        return launchLeaderKraken.getVelocity().getValue();
    }

    @Override
    public void setKickerVelocity(AngularVelocity kickerRps) {
        kickerKraken.setControl(kickerControlMode.withVelocity(kickerRps));
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return kickerKraken.getVelocity().getValue();
    }

    @Override
    public boolean getFeedSensor() {
        return feedDetector.getIsDetected().getValue();
    }

}
