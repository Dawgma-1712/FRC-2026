package frc.robot.subsystems.Revolver;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.Constants.IdConstants;
import frc.Constants.RevolverConstants;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;


public class RevolverIOReal implements RevolverIO{
    public final TalonFX revolverMotor = new TalonFX(IdConstants.REVOLVER_MOTOR_ID);   

    public RevolverIOReal(){

    }

    @Override
    public void setRevolverVelocity(double velocity){
        revolverMotor.set(velocity);
    }

    @Override
    public double getRevolverVelocity(){
        return revolverMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void updateInputs(RevolverIOInputs inputs) {
    }


}


