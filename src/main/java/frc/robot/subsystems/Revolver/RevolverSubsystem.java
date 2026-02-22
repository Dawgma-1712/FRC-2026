package frc.robot.subsystems.Revolver;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class RevolverSubsystem extends SubsystemBase{

    RevolverIO io;

    public RevolverSubsystem(RevolverIO io){
        this.io = io;
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
