package frc.robot.subsystems.Launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.ShooterConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class LauncherIORealBangBang implements LauncherIO{


    private enum FlywheelPhase {
        STARTUP, IDLE, BALL, RECOVERY
    }

    private enum FeederPhase {
        STARTUP, IDLE
    }

    private FeederPhase feederPhase = FeederPhase.STARTUP;
    private double feederTargetRPS = 0.0;

    private AngularVelocity prevLauncherVelocity;
    private boolean isBeginning = true;

    private FlywheelPhase kickerPhase = FlywheelPhase.STARTUP;
    private double kickerTargetRPS = 0.0;

    private final TalonFX kickMotor = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX feedMotor = new TalonFX(IdConstants.KICK_MOTOR_ID);

    private final DigitalInput intakeSwitch = new DigitalInput(IdConstants.FUEL_FEED_SENSOR_ID);
    private final DigitalInput shootingSwitch = new DigitalInput(IdConstants.LAUNCHER_FUEL_SHOOTING_BEAMBREAK_ID);  // holy variable name

    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID,40,0);
    //we are saying that the range for this encoder has a maximum of 40 and the zero is at 0
    //We say 40 because that is the max angle of the hood and 0 is the start
    //FOR WIRING USE THE BLACK RED WHITE TRIPLET OF WIRES AND PLUG INTO DIO

    private final VelocityDutyCycle kickerDutyCycle = new VelocityDutyCycle(0).withSlot(0);
    private final VelocityTorqueCurrentFOC kickerTorqueBangBang = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC kickerConstantTorque = new TorqueCurrentFOC(0);

    private final VelocityDutyCycle feederDutyCycle = new VelocityDutyCycle(0).withSlot(0);
    private final VelocityTorqueCurrentFOC feederTorqueBangBang = new VelocityTorqueCurrentFOC(0).withSlot(0);

    Debouncer launcherDebouncer = new Debouncer(ShooterConstants.DEBOUNCE_LENGTH, Debouncer.DebounceType.kBoth);

    private boolean previousFuelBeamBreak = false;
    private boolean previousShotBeamBreak = false;

    public LauncherIORealBangBang() {

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = Constants.ShooterConstants.BANG_BANG_LAUNCHER_kP;
        configs.Slot0.kI = Constants.ShooterConstants.BANG_BANG_LAUNCHER_kI;
        configs.Slot0.kD = Constants.ShooterConstants.BANG_BANG_LAUNCHER_kD;
        configs.Slot0.kV = Constants.ShooterConstants.BANG_BANG_LAUNCHER_kV;
        configs.Slot0.kS = Constants.ShooterConstants.BANG_BANG_LAUNCHER_kS;

        configs.MotorOutput.PeakForwardDutyCycle = ShooterConstants.LAUNCHER_PEAK_FORWARD_DUTY_CYCLE;
        configs.MotorOutput.PeakReverseDutyCycle = ShooterConstants.LAUNCHER_PEAK_REVERSE_DUTY_CYCLE;

        configs.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.LAUNCHER_PEAK_FORWARD_TORQUE_CURRENT;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = ShooterConstants.LAUNCHER_PEAK_REVERSE_TORQUE_CURRENT;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kickMotor.getConfigurator().apply(configs);
        feedMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setLauncherVelocity(AngularVelocity wheelVelocity) {

        double rps = wheelVelocity.in(Units.RotationsPerSecond);

        if(Math.abs(rps-kickerTargetRPS)>5.0){
            kickerPhase = FlywheelPhase.STARTUP;
        }
        kickerTargetRPS = rps;
    }

    @Override
    public AngularVelocity getLauncherVelocity() {
        return kickMotor.getVelocity().getValue();
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return feedMotor.getVelocity().getValue();
    }

    @Override
    public void setKickerVelocity(AngularVelocity kickerVelocity) {

        double rps = kickerVelocity.in(Units.RotationsPerSecond);

        if(Math.abs(rps-feederTargetRPS)>5.0){
            feederPhase = FeederPhase.STARTUP;
        }
        feederTargetRPS = rps;
    }

    @Override
    public Angle getHoodPosition() {
        return Units.Rotations.of(hoodEncoder.get());
    }

    @Override
    public void setHoodPosition(Angle angle) {
        //write logic here depending on if you use a servo or something else
        //1 rotation of the motor equals 17/360 degree change physically
    }

    @Override
    public boolean hasFuelIntaked() {
        return !intakeSwitch.get(); 
    }

    @Override
    public boolean hasShotFuel() {
        return !shootingSwitch.get();
    }

    @Override
    public void updateInputs(LauncherIOInputs inputs) {
        inputs.kickMotorVelocity = kickMotor.getVelocity().getValueAsDouble();
        inputs.feedMotorVelocity = feedMotor.getVelocity().getValueAsDouble();
        inputs.hoodMotorPosition = hoodEncoder.get();
        inputs.hasFuelIntaked = !intakeSwitch.get();    // beam breaks are typically inverted
        inputs.fuelShot = !shootingSwitch.get();
    }  


    public Command actuallySetLauncherVelocity(AngularVelocity launcherTargetVelocity) {

        return Commands.runOnce(() -> {

            AngularVelocity launcherVelocity = getKickerVelocity();

            if(isBeginning) {
                prevLauncherVelocity = getLauncherVelocity();
                isBeginning = false;
            }      

            boolean velocityDecreasing = launcherVelocity.minus(prevLauncherVelocity).magnitude() < 0;
            boolean decreasedThreshold = launcherDebouncer.calculate(velocityDecreasing);

            //if decreased we are in the constant mode
            if(decreasedThreshold){
                kickMotor.setControl(kickerConstantTorque.withOutput(ShooterConstants.LAUNCHER_PEAK_FORWARD_TORQUE_CURRENT));
            }
            else{
                kickMotor.setControl(kickerTorqueBangBang.withVelocity(launcherTargetVelocity));
            }

            prevLauncherVelocity = launcherVelocity;
            
        });
        

    }




    public void periodic() {

        AngularVelocity kickerVelocity = getKickerVelocity();
        double kickerRotationsPerSecond = kickerVelocity.in(Units.RotationsPerSecond);
        double kickerError = kickerTargetRPS - kickerRotationsPerSecond;

        boolean fuelIntakedNow = hasFuelIntaked();
        boolean fuelShotNow = hasShotFuel();

        if(!previousFuelBeamBreak && fuelIntakedNow && kickerPhase == FlywheelPhase.IDLE) {
            kickerPhase = FlywheelPhase.BALL;
        }

        if(!previousShotBeamBreak && fuelShotNow && kickerPhase == FlywheelPhase.BALL) {
            kickerPhase = FlywheelPhase.RECOVERY;
        }

        previousFuelBeamBreak = fuelIntakedNow;
        previousShotBeamBreak = fuelShotNow;

        switch (kickerPhase) {
            case STARTUP:
                kickMotor.setControl(kickerDutyCycle.withVelocity(kickerTargetRPS));
                if(Math.abs(kickerError) < ShooterConstants.AT_SPEED_TOLERANCE_RPS){
                    kickerPhase = FlywheelPhase.IDLE;
                }
                break;
            
            case IDLE:
                kickMotor.setControl(kickerTorqueBangBang.withVelocity(kickerTargetRPS));
                break;
            
            case BALL:
                kickMotor.setControl(kickerConstantTorque.withOutput(ShooterConstants.LAUNCHER_PEAK_FORWARD_TORQUE_CURRENT));
                break;
            
            case RECOVERY:
                kickMotor.setControl(kickerDutyCycle.withVelocity(kickerTargetRPS));
                if(Math.abs(kickerError) < ShooterConstants.AT_SPEED_TOLERANCE_RPS){
                    kickerPhase = FlywheelPhase.IDLE;
                }
                break;
        }

        AngularVelocity feederVelocity = getKickerVelocity();
        double feederRotationsPerSecond = feederVelocity.in(Units.RotationsPerSecond);
        double feederError = feederTargetRPS - feederRotationsPerSecond;

        switch (feederPhase) {
            case STARTUP:
                feedMotor.setControl(feederDutyCycle.withVelocity(feederTargetRPS));
                if(Math.abs(feederError) < ShooterConstants.AT_SPEED_TOLERANCE_RPS){
                    feederPhase = FeederPhase.IDLE;
                }
                break;
            
            case IDLE:
                feedMotor.setControl(feederTorqueBangBang.withVelocity(feederTargetRPS));
                break;
        }
    }
}