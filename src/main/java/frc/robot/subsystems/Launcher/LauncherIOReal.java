package frc.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.Constants;
import frc.Constants.IdConstants;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// if add hooded motor then import these

public class LauncherIOReal implements LauncherIO{

    private final TalonFX kickMotor = new TalonFX(IdConstants.KICK_MOTOR_ID);

    private final TalonFX feedMotor = new TalonFX(IdConstants.FEED_MOTOR_ID);
    
    private final VelocityVoltage kickerControl = new VelocityVoltage(0);
    private final VelocityVoltage feederControl = new VelocityVoltage(0);

    private final DigitalInput fuelBeamBreak = new DigitalInput(IdConstants.LAUNCHER_FUEL_DETECTION_BEAMBREAK_ID);

    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID,40,0);
    //we are saying that the range for this encoder has a maximum of 40 and the zero is at 0
    //We say 40 because that is the max angle of the hood and 0 is the start
    //FOR WIRING USE THE BLACK RED WHITE TRIPLET OF WIRES AND PLUG INTO DIO

    public LauncherIOReal() {

        TalonFXConfiguration kickConfigs = new TalonFXConfiguration();
        kickConfigs.Slot0.kP = Constants.ShooterConstants.KICKER_kP;
        kickConfigs.Slot0.kI = Constants.ShooterConstants.KICKER_kI;
        kickConfigs.Slot0.kD = Constants.ShooterConstants.KICKER_kD;

        // may have to change whether both motors are spinning clockwise
        kickConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kickMotor.getConfigurator().apply(kickConfigs);

        TalonFXConfiguration feederConfigs = new TalonFXConfiguration();
        feederConfigs.Slot0.kP = Constants.ShooterConstants.FEEDER_kP;
        feederConfigs.Slot0.kI = Constants.ShooterConstants.FEEDER_kI;
        feederConfigs.Slot0.kD = Constants.ShooterConstants.FEEDER_kD;
        
        feedMotor.getConfigurator().apply(feederConfigs);
    }

    @Override
    public void setKickerVelocity(double rps) {
        kickMotor.setControl(kickerControl.withVelocity(rps));
    }

    @Override
    public double getKickerVelocity(){
        return kickMotor.getVelocity().getValueAsDouble();
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
    public double getHoodPosition(){
        return hoodEncoder.get();
    }

    @Override
    public void setHoodPosition(double angle){
        //write logic here depending on if you use a servo or something else
        //1 rotation of the motor equals 17/360 degree change physically
    }

    @Override
    public boolean hasFuel(){
        return !fuelBeamBreak.get(); 
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.kickMotorVelocity = kickMotor.getVelocity().getValueAsDouble();
        inputs.feedMotorVelocity = feedMotor.getVelocity().getValueAsDouble();
        inputs.hasFuel = hasFuel();
    }

}
