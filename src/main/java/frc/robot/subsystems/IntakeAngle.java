package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;

public class IntakeAngle extends SubsystemBase {

    public final TalonFX angleMotor = new TalonFX(Constants.IdConstants.ANGLE_MOTOR_ID);
    public double desiredPosition = 0;
    
   
    public IntakeAngle() {
        zeroPosition();
    }

    public void stop() {
        angleMotor.setControl(new MotionMagicVoltage(0));
    }

    public void setPosition(double position) {
        angleMotor.setPosition(position / 3.6);
    }

    public void zeroPosition() {
        angleMotor.setPosition(0);
    }

    public double getPosition() {
        return angleMotor.getPosition().getValueAsDouble() * 3.6;
    }

    public void PIDPosition(double position) {
        //0 is straight up
        PositionVoltage pVoltage = new PositionVoltage(0).withSlot(0);
        angleMotor.setControl(pVoltage.withPosition(position / 3.6).withFeedForward(0));
    }

    public void moveToPosition(double position) {
        desiredPosition = position;
    }

    public double getSpeed() {
        return angleMotor.get();
    }

    public double getSetpoint() {
        return desiredPosition;
    }

    public void setSetpoint(double setpoint) {
        desiredPosition = setpoint;
    }

    @Override
    public void periodic() {
        //   if(!topSwitch.get()) {
        //       setPosition(OperatorConstants.topSwitchPosition);
        //       if(desiredPosition <= OperatorConstants.topSwitchPosition) {
        //         desiredPosition = OperatorConstants.topSwitchPosition;
        //       }
        //  } else if(!bottomSwitch.get()) {
        //     //setPosition(OperatorConstants.bottomSwitchPosition);
        //  }

        //  if(!bottomSwitch.get() && desiredPosition >= OperatorConstants.bottomSwitchPosition) {
        //      desiredPosition = OperatorConstants.bottomSwitchPosition;
        //  }

         SmartDashboard.putNumber("Intake Angle", getPosition());

        PIDPosition(desiredPosition);
    }
}
