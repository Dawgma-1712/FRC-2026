package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsystem extends SubsystemBase{
    
    IntakeIO io;
    
    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

     public void setIntakeMotorSpeed(double speed) {
        io.setIntakeMotorSpeed(speed);
    }

    public void setAngleMotorSpeed(double speed) {
        io.setAngleMotorSpeed(speed);
    }

    public Command setAngle(double target) {
        return Commands.runOnce(
            () -> {
                io.setAngle(target);
            }
        );
    }

    public double getAngle() {
        return io.getAngle();
    }

    @Override
    public void periodic() {
        // io.updateInputs(IntakeIOInputs);
        SmartDashboard.putNumber("Intake Angle", io.getAngle());
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
    
}
