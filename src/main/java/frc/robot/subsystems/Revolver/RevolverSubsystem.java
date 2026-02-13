package frc.robot.subsystems.Revolver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.RevolverConstants;

public class RevolverSubsystem extends SubsystemBase{

    RevolverIO io;

    public RevolverSubsystem(RevolverIO io){
        this.io=io;
    }

    public void setRevolverVelocity(double velocity){
        io.setRevolverVelocity(velocity);
    }

    public double getRevolverVelocity(){
        return io.getRevolverVelocity();
    }

    @Override
    public void periodic(){
        
    }

}
