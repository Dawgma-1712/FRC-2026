package frc.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.Constants;
import frc.Constants.IdConstants;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
//if add hooded motor then import these

public class LauncherIOReal implements LauncherIO{
    private final TalonFX shootMotor1 = new TalonFX(IdConstants.SHOOT_MOTOR_ID1);
    private final TalonFX shootMotor2 = new TalonFX(IdConstants.SHOOT_MOTOR_ID2);

    private final TalonFX feedMotor = new TalonFX(IdConstants.FEED_MOTOR_ID);
    
    private final VelocityVoltage shooterControl = new VelocityVoltage(0);
    private final VelocityVoltage feederControl = new VelocityVoltage(0);

    private final DigitalInput fuelBeamBreak = new DigitalInput(Constants.IdConstants.LAUNCHER_FUEL_DETECTION_BEAMBREAK_ID);

    public LauncherIOReal() {
        TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        shooterConfigs.Slot0.kP = Constants.ShooterConstants.SHOOTER_KP1;
        shooterConfigs.Slot0.kI = Constants.ShooterConstants.SHOOTER_KI1;
        shooterConfigs.Slot0.kD = Constants.ShooterConstants.SHOOTER_KD1;
        shooterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        shootMotor1.getConfigurator().apply(shooterConfigs);
        shootMotor2.getConfigurator().apply(shooterConfigs);

        TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
        feederConfigs.Slot0.kP = Constants.ShooterConstants.FEEDER_KP2;
        feederConfigs.Slot0.kI = Constants.ShooterConstants.FEEDER_KI2;
        feederConfigs.Slot0.kD = Constants.ShooterConstants.FEEDER_KD2;
        
        feedMotor.getConfigurator().apply(feederConfigs);
    }

    @Override
    public void setShooterVelocity(double rps) {
        shootMotor1.setControl(shooterControl.withVelocity(rps));
        shootMotor2.setControl(shooterControl.withVelocity(rps));
    }

    @Override
    public double getShooter1Velocity(){
        return shootMotor1.getVelocity().getValueAsDouble();
    }

    @Override
    public double getShooter2Velocity(){
        return shootMotor2.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFeederVelocity(){
        return feedMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setFeederVelocity(double rps) {
        feedMotor.setControl(feederControl.withVelocity(rps));
    }

    @Override
    public boolean hasFuel(){
        return !fuelBeamBreak.get(); 
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.shootMotorVelocity = shootMotor1.getVelocity().getValueAsDouble();
        inputs.feedMotorVelocity = feedMotor.getVelocity().getValueAsDouble();
        inputs.hasFuel = hasFuel();
    }

}
