package frc.robot.subsystems.Intake;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class IntakeSubsystem extends SubsystemBase {

    IntakeIO io;
    
    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

    public void setIntakeMotorSpeed(double speed) {
        io.setIntakeMotorSpeed(speed);
    }

    public void setAngleMotorSpeed(double percentOutput) {
        io.setAngleMotorPercentOutput(percentOutput);
    }

    public void holdPosition() {
        io.holdPosition();
    }

    public void manualTriggerIntakeSpeed(Supplier<Double> triggerOutput) {
        double output = triggerOutput.get();
        setIntakeMotorSpeed(output);
    }

    public Command setAngle(Angle target) {
        return Commands.runOnce(
            () -> {
                io.setAngle(target);
            }
        );
    }

    public void setAngleDirect(Angle target) {
        io.setAngle(target);
    }


    public Angle getAngle() {
        return io.getAngle();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Intake angle", getAngle().in(Units.Degrees));
        io.controlLoop();
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
    
}
