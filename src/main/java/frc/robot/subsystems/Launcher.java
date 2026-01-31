package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.ShooterConstants;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

public class Launcher extends SubsystemBase {
    
    private final TalonFX shooterMotor1 = new TalonFX(IdConstants.SHOOTER_MOTOR_ID1);
    private final TalonFX shooterMotor2 = new TalonFX(IdConstants.SHOOTER_MOTOR_ID2);
    private final TalonFX shooterMotor3 = new TalonFX(IdConstants.SHOOTER_MOTOR_ID3);
    private final SparkMax hoodMotor = new SparkMax(Constants.IdConstants.HOOD_MOTOR_ID, MotorType.kBrushed);

    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

    public Launcher() {

        // configure the Kraken
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor1.getConfigurator().apply(configs);
        shooterMotor2.getConfigurator().apply(configs);
        shooterMotor3.getConfigurator().apply(configs);

        // PID
        Slot0Configs slot0configs = new Slot0Configs();
        slot0configs.kP = ShooterConstants.SHOOTER_KP1;
        slot0configs.kI = ShooterConstants.SHOOTER_KI1;
        slot0configs.kD = ShooterConstants.SHOOTER_KD1;

        Slot0Configs slot1configs = new Slot0Configs();
        slot0configs.kP = ShooterConstants.SHOOTER_KP2;
        slot0configs.kI = ShooterConstants.SHOOTER_KI2;
        slot0configs.kD = ShooterConstants.SHOOTER_KD2;

        shooterMotor1.getConfigurator().apply(slot0configs);
        shooterMotor2.getConfigurator().apply(slot0configs);
        shooterMotor3.getConfigurator().apply(slot1configs);
        hoodMotor.getEncoder().setPosition(0); //arbitrary


    }

    public void stopShooter() {
        velocityDutyCycle.Velocity = 0;
        shooterMotor1.setControl(velocityDutyCycle);
        shooterMotor2.setControl(velocityDutyCycle);

    }

    public void stopFeeder() {
        velocityDutyCycle.Velocity = 0;
        shooterMotor3.setControl(velocityDutyCycle);

    }

    // A 2D kinematics equation, neglects air resistance
    // Maybe check my work?
    private double calculateInitialVelocity(double deltaX) {
        
        double g = ShooterConstants.GRAVITY;
        double theta = ShooterConstants.SHOOTER_ANGLE;
        double deltaY = ShooterConstants.HUB_HEIGHT_METERS;

        return Math.sqrt(
          (g * Math.pow(deltaX, 2))  / (2 * Math.pow( Math.cos(theta), 2) * (deltaX * Math.tan(theta) - deltaY))
        );

    }

    // calculates rpm of a wheel given circumference and surface speed
    // ASSUMES CIRCUMFERENCE IS IN INCHES AND SURFACE SPEED IS IN FEET PER MINUTE
    private double calculateRpm(double circumference, double surfaceSpeed) {

        double circumferenceFeet = circumference / 12;

        return surfaceSpeed / circumferenceFeet;

    }

    public void shootRpm(double desiredRpm) {

        velocityDutyCycle.Velocity = desiredRpm / 60;  // in rotations per second
        shooterMotor1.setControl(velocityDutyCycle);
    
    }

    // Command factory to calculate how fast the shooter should spin and then shoot
    public Command adaptiveShoot(double distance) {
        
        double initialVelocity = calculateInitialVelocity(distance);

        // if we use two wheels, the fuel will have an exit velocity of about the surface speed of the wheels, so we just need to get the rpm from the surface speed and circumference
        double circumference = ShooterConstants.WHEEL_DIAMETER * Math.PI;
        double surfaceSpeedFeetMinute = initialVelocity * (1 / 60) * (39.37) * 12;
        double desiredRpm = calculateRpm(circumference, surfaceSpeedFeetMinute);


        return this.run(
            () -> {
                shootRpm(desiredRpm);
            }
        );

    }

}
