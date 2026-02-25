package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.Constants.IdConstants;
import frc.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class IntakeIOReal implements IntakeIO {

    /*
     * MOTORS
     */

    private final SparkMax intakeMotor = new SparkMax(IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    public final TalonFX angleMotor = new TalonFX(IdConstants.ANGLE_MOTOR_ID);

    /*
     * SENSORS
     */

    // REV through bore encoder
    private final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(IdConstants.INTAKE_ANGLE_ENCODER_ID, 
                                                             360.0, 
                                                          0.0);
    
    /*
     * OTHER
     */

    public double angleMotorPosition = 0;
    private final MotionMagicVoltage requestControl = new MotionMagicVoltage(0);

    public IntakeIOReal() {
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
        // we probably don't need jerk
        angleMotor.getConfigurator().apply(angleConfig);

        angleMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setIntakeMotorSpeed(double percentOutput) {
        intakeMotor.set(percentOutput);
    }

    @Override
    public void setAngle(Angle target) {

        angleMotor.setControl(
            requestControl
            .withPosition(
                target.in(Units.Rotations)
            )
        );

    }

    @Override
    public Angle getAngle() {
        return Units.Degrees.of(angleEncoder.get());
    }

    @Override //literally useless until we set up advantagekit
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angleMotorPosition = getAngle().in(Units.Degrees);
    }
    
}
