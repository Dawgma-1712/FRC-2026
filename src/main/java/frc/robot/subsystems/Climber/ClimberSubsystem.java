package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;



public class ClimberSubsystem extends SubsystemBase{

    ClimberIO io;
    
    public ClimberSubsystem(ClimberIO io){
        this.io = io;
    }

    public void ZeroMotorEncoder(){
        io.ZeroMotorEncoder();
    }

    public void setClimberPosition(double position){
        io.setClimberPosition(position);
    }

    public double getClimberPosition(){
        return io.getClimberPosition();
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
    
}
