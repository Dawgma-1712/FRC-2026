package frc.robot.subsystems.Intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;


public class IntakeSubsystem extends SubsystemBase{

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    IntakeIO io;
    
    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

     public void setIntakeMotorSpeed(double speed) {
        io.setIntakeMotorSpeed(speed);
    }

    public Command setAngle(Angle target) {
        return Commands.runOnce(
            () -> {
                io.setAngle(target);
            }
        );
    }

    public Angle getAngle() {
        return io.getAngle();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake Angle", io.getAngle().in(Units.Degrees));
    }

    @Override
    public void simulationPeriodic() {
        io.simPeriodic();
    }
    
}
