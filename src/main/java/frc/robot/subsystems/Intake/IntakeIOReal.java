package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO {

    private final SparkMax intakeMotor = new SparkMax(IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    public final TalonFX angleMotor = new TalonFX(IdConstants.ANGLE_MOTOR_ID);
    
    public double angleMotorPosition = 0;

    private final MotionMagicVoltage requestControl = new MotionMagicVoltage(0);

    private final DigitalInput limitSwitch = new DigitalInput(IdConstants.INTAKE_LIMIT_SWITCH_ID);

    public IntakeIOReal(){
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();

        // pid constants - NOT TUNED
        angleConfig.Slot0.kP = IntakeConstants.ANGLE_kP; 
        angleConfig.Slot0.kI = IntakeConstants.ANGLE_kI;
        angleConfig.Slot0.kD = IntakeConstants.ANGLE_kD;

        // physics constants-idk exactly how this works we should change them at some point
        angleConfig.Slot0.kS = IntakeConstants.ANGLE_kS; // Static friction (voltage to get it moving)
        angleConfig.Slot0.kV = IntakeConstants.ANGLE_kV; // Velocity feedforward
        angleConfig.Slot0.kA = IntakeConstants.ANGLE_kA; // Acceleration feedforward

        // Setup Motion Magic limits
        angleConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.ANGLE_CRUISE_VELOCITY; // Rotations per second
        angleConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.ANGLE_ACCELERATION;   // Rotations per second^2
        angleConfig.MotionMagic.MotionMagicJerk = IntakeConstants.ANGLE_JERK;         // Smoothing

        angleMotor.getConfigurator().apply(angleConfig);

        angleMotor.setNeutralMode(NeutralModeValue.Brake);

        ZeroMotorEncoder();
    }

    @Override
    public void setIntakeMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(speed);
    }

    @Override
    public boolean setAngleMotorPosition(double target) {
        double currentPos = getAngleMotorPosition();

        boolean closeEnough = Math.abs(target - currentPos) <= IntakeConstants.POSITION_TOLERANCE;
        if (!closeEnough) {
            angleMotor.setControl(requestControl.withPosition(target));
            return false;
        } else {
            angleMotor.stopMotor(); 
            return true;
        }
    }

    @Override
    public double getAngleMotorPosition() {
        if(limitSwitch.get()){
            ZeroMotorEncoder();
        }
        angleMotorPosition=angleMotor.getPosition().getValueAsDouble();
        return angleMotorPosition;
    }

    @Override
    public void ZeroMotorEncoder() {
        angleMotor.setPosition(0);
    }

    @Override //literally useless until we set up advantagekit
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angleMotorPosition = getAngleMotorPosition();
        // inputs.limitSwitchTriggered = limitSwitch.get(); // Ensure this is in your IntakeIOInputs class
    }
    
}
