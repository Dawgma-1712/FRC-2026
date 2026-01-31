package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOReal implements IntakeIO{

    private final SparkMax intakeMotor = new SparkMax(Constants.IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    public final TalonFX angleMotor = new TalonFX(Constants.IdConstants.ANGLE_MOTOR_ID);
    public double angleMotorPosition = 0;

    private final MotionMagicVoltage requestControl = new MotionMagicVoltage(0);

    private final DigitalInput limitSwitch = new DigitalInput(Constants.IdConstants.INTAKE_LIMIT_SWITCH_ID);

    private final double POSITION_TOLERANCE = 2.0; // Tune this (e.g., 2 degrees)
    //put that into constants

    public IntakeIOReal(){
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();

        //pid constants-NOT TUNED
        angleConfig.Slot0.kP = 2.4; 
        angleConfig.Slot0.kI = 0.0;
        angleConfig.Slot0.kD = 0.1;

        //physics constants-idk exactly how this works we should change them at some point
        angleConfig.Slot0.kS = 0.25; // Static friction (voltage to get it moving)
        angleConfig.Slot0.kV = 0.12; // Velocity feedforward
        angleConfig.Slot0.kA = 0.01; // Acceleration feedforward

        // 2. Setup Motion Magic limits
        angleConfig.MotionMagic.MotionMagicCruiseVelocity = 40; // Rotations per second
        angleConfig.MotionMagic.MotionMagicAcceleration = 80;   // Rotations per second^2
        angleConfig.MotionMagic.MotionMagicJerk = 1600;         // Smoothing

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

        boolean closeEnough = Math.abs(target-currentPos)<=POSITION_TOLERANCE;
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
