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
    private boolean intakeDown = false;
    
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

    public void toggleIntake() {
        if (intakeDown) {
            setAngleDirect(Units.Degrees.of(90));
        } else {
            setAngleDirect(Units.Degrees.of(0));
        }
        intakeDown = !intakeDown;
    }

    public void manualTriggerIntakeSpeed(Supplier<Double> triggerOutput, Supplier<Double> triggerOutput2) {
        double output = triggerOutput.get();
        double rightOutput = triggerOutput2.get();
        if (output > rightOutput) {
        setIntakeMotorSpeed(output);
        } else {
            setIntakeMotorSpeed(-rightOutput);
        }
    }

    public void dumbDumbPID(double goalAngle) {
        // double currentAngle = getAngle().in(Units.Degrees);

        // if(Math.abs(currentAngle - goalAngle) <= 5) {
        //     io.setAngleMotorPercentOutput(0);
        // }
        // else if (currentAngle > goalAngle) {
        //     io.setAngleMotorPercentOutput(0.1);
        // }
        // else if (currentAngle < goalAngle) {
        //     io.setAngleMotorPercentOutput(-0.2);
        // }

        io.setAngleMotorPercentOutput(0.1);
    }

    public void nodumbDumbPID(double goalAngle) {
        // double currentAngle = getAngle().in(Units.Degrees);

        // if(Math.abs(currentAngle - goalAngle) <= 5) {
        //     io.setAngleMotorPercentOutput(0);
        // }
        // else if (currentAngle > goalAngle) {
        //     io.setAngleMotorPercentOutput(0.1);
        // }
        // else if (currentAngle < goalAngle) {
        //     io.setAngleMotorPercentOutput(-0.2);
        // }

        io.setAngleMotorPercentOutput(0);
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
