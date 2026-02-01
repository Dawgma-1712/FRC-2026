package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

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

    public boolean setAngleMotorPosition(double target) {
        return io.setAngleMotorPosition(target);
    }

    public double getAngleMotorPosition() {
        return io.getAngleMotorPosition();
    }

    public void ZeroMotorEncoder() {
        io.ZeroMotorEncoder();
    }

    @Override
    public void periodic() {
        // io.updateInputs(IntakeIOInputs);
        SmartDashboard.putNumber("Intake Angle", io.getAngleMotorPosition());
    }

    
}
