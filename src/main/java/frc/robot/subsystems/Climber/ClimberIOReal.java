package frc.robot.subsystems.Climber;
import frc.Constants;
import frc.Constants.IdConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class ClimberIOReal implements ClimberIO {
    
    private final TalonFX climberMotor = new TalonFX(IdConstants.CLIMBER_MOTOR);

    private double climberPos = 0.0;

    private final MotionMagicVoltage requestControl = new MotionMagicVoltage(0);


    public ClimberIOReal(){

        TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

        climberConfigs.Slot0.kP = Constants.ClimberConstants.CLIMBER_kP;
        climberConfigs.Slot0.kI = Constants.ClimberConstants.CLIMBER_kI;
        climberConfigs.Slot0.kD = Constants.ClimberConstants.CLIMBER_kD;

        climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberConfigs.Slot0.kS = Constants.ClimberConstants.CLIMBER_kS;
        climberConfigs.Slot0.kV = Constants.ClimberConstants.CLIMBER_kV;
        climberConfigs.Slot0.kA = Constants.ClimberConstants.CLIMBER_kA;


        climberConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimberConstants.ANGLE_CRUISE_VELOCITY; // Rotations per second
        climberConfigs.MotionMagic.MotionMagicAcceleration = Constants.ClimberConstants.ANGLE_ACCELERATION;   // Rotations per second^2
        climberConfigs.MotionMagic.MotionMagicJerk = Constants.ClimberConstants.ANGLE_JERK;         // Smoothing

        climberMotor.getConfigurator().apply(climberConfigs);

        climberMotor.setNeutralMode(NeutralModeValue.Brake);

        ZeroMotorEncoder();
    }

    @Override
    public void ZeroMotorEncoder(){
        climberMotor.setPosition(0);
    }

    @Override
    public double getClimberPosition(){
        climberPos = climberMotor.getPosition().getValueAsDouble();
        return climberPos;
    }

    @Override
    public void setClimberPosition(double position) {
        climberMotor.setControl(requestControl.withPosition(position));
    }


    @Override //literally useless until we set up advantagekit
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPos = getClimberPosition();
    }

}
