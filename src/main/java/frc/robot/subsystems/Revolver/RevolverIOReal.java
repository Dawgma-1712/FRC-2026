package frc.robot.subsystems.Revolver;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.Constants.IdConstants;


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


