package frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ClimberIOReal implements ClimberIO {
    
    private final TalonFX climberMotor = new TalonFX(IdConstants.CLIMBER_MOTOR);

    private final VelocityVoltage climberControl = new VelocityVoltage(0);

    public ClimberIOReal(){

        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();

        climberConfigs.Slot0.kP = Constants.ClimberConstants.CLIMBER_kP;
        climberConfigs.Slot0.kI = Constants.ClimberConstants.CLIMBER_kI;
        climberConfigs.Slot0.kD = Constants.ClimberConstants.CLIMBER_kD;

        climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberMotor.getConfigurator().apply(climberConfigs);

        climberConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.ANGLE_CRUISE_VELOCITY; // Rotations per second
        climberConfigs.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.ANGLE_ACCELERATION;   // Rotations per second^2
        climberConfigs.MotionMagic.MotionMagicJerk = Constants.ClimberConstants.ANGLE_JERK;         // Smoothing

        climberMotor.setNeutralMode(NeutralModeValue.Brake);
         ZeroMotorEncoder();

    }

    public void ZeroMotorEncoder(){
        climberMotor.setPosition(0);
    }

    public double getClimberVelocity(){
        return climberMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setClimberPose() {
        
    }





}
