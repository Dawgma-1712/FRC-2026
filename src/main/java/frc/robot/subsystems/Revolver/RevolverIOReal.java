package frc.robot.subsystems.Revolver;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.Constants.IdConstants;


public class RevolverIOReal implements RevolverIO {

    public final TalonFX revolverMotor = new TalonFX(IdConstants.REVOLVER_MOTOR_ID);   

    public RevolverIOReal() {}

    @Override
    public void setRevolverPercentOutput(double percentOutput) {
        revolverMotor.set(percentOutput);
    }

    @Override
    public AngularVelocity getRevolverVelocity() {
        return revolverMotor.getVelocity().getValue();
    }

    @Override
    public void updateInputs(RevolverIOInputs inputs) {}


}


