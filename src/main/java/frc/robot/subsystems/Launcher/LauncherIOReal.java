package frc.robot.subsystems.Launcher;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.Constants;
import frc.Constants.IdConstants;
import frc.Constants.LauncherConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;

public class LauncherIOReal implements LauncherIO {


    // KICKER
    private final TalonFX kickerKraken = new TalonFX(IdConstants.KICK_MOTOR_ID);

    // LAUNCHER
    private final TalonFX launchLeaderKraken = new TalonFX(IdConstants.LAUNCH_MOTOR_ID);
    private final TalonFX launchFollowerKraken = new TalonFX(IdConstants.LAUNCH_FOLLOWER_ID);

    // HOOD
    private final SparkMax hoodBagMotor = new SparkMax(IdConstants.HOOD_MOTOR_ID, MotorType.kBrushed);
    private final PIDController hoodPidController = new PIDController(LauncherConstants.HOOD_kP, LauncherConstants.HOOD_kI, LauncherConstants.HOOD_kD);
    private final ArmFeedforward hoodFeedforward = new ArmFeedforward(LauncherConstants.HOOD_kS, LauncherConstants.HOOD_kG, LauncherConstants.HOOD_kV);

    // SENSORS
    // Relative-mode hood encoder: we only ever consume deltas, so expectedZero is 0.
    // Any offset here cancels out of (currentRaw - lastRawAngle) and does nothing.
    private final DutyCycleEncoder hoodEncoder = new DutyCycleEncoder(IdConstants.HOOD_ENCODER_ID, 360, 0.0);

    // CONTROL MODES
    private final VelocityVoltage launcherControlMode = new VelocityVoltage(0);

    // STATUS SIGNALS
    private final StatusSignal<AngularVelocity> launcherVelocitySignal = launchLeaderKraken.getVelocity();
    private final StatusSignal<AngularVelocity> kickerVelocitySignal = kickerKraken.getVelocity();

    private final Debouncer launcherDebouncer = new Debouncer(LauncherConstants.DEBOUNCE_LENGTH, Debouncer.DebounceType.kRising);


    // Encoder shaft rotations per hood rotation.
    private static final double HOOD_GEAR_RATIO = 17.0;

    // raw angle is zero here, because the encoder has not initialized yet. 
    private double lastRawAngle = 0.0;
    private double cumulativeEncoderDegrees = 0.0;
    private boolean hoodZeroed = false;
    private Angle hoodGoalAngle = Units.Degrees.of(0);

        
        public LauncherIOReal() {
    
            //var config = new TalonFXConfiguration();
            var config = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Units.Amps.of(50))
            .withSupplyCurrentLimitEnable(true));
    
            config.Slot0.kP = LauncherConstants.LAUNCH_kP;
            config.Slot0.kI = LauncherConstants.LAUNCH_kI;
            config.Slot0.kD = LauncherConstants.LAUNCH_kD;
            config.Slot0.kV = Constants.LauncherConstants.LAUNCH_kV;
            config.Slot0.kS = Constants.LauncherConstants.LAUNCH_kS;
    
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
            launchLeaderKraken.getConfigurator().apply(config);
            launchFollowerKraken.getConfigurator().apply(config);

            SparkMaxConfig hoodConfig = new SparkMaxConfig();
            hoodConfig.idleMode(IdleMode.kBrake);
            hoodConfig.smartCurrentLimit(30); // adjust for bag motor
            hoodBagMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            kickerKraken.optimizeBusUtilization();

        }
    
        @Override
        public void setLauncherVelocity(AngularVelocity shooterRps) {
            
            // if the velocity is under tolerance, use a duty cycle bang bang and try to get up to speed
            // this will be the same regardless of whether it's trying to get up to speed from rest or recovering from a shot
            launchLeaderKraken.setControl(launcherControlMode.withVelocity(shooterRps));
            // if it's up to speed, then use torque current control in preparation for a shot
            // the fuel's only in contact for like 4ms, so by the time we detect that the velocity dropped, the fuel will have already left
            launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    
        @Override
        public void hoodControlLoop() {

            if (!hoodZeroed) {
                // No valid encoder reading yet, so getHoodPosition() would report a
                // constant 0 and the PID would drive the hood open-loop into a hard stop.
                hoodBagMotor.setVoltage(0);
                return;
            }

            Angle currentAngle = getHoodPosition();
    
            // if necessary, we can add a trapezoidal profile here too
            double pidOutput = hoodPidController.calculate(currentAngle.in(Units.Degrees), hoodGoalAngle.in(Units.Degrees));
    
            double hoodAngleRads = Math.toRadians(hoodGoalAngle.in(Units.Degrees));
    
            double totalVoltage = pidOutput;
            SmartDashboard.putNumber("Launcher/Hood PID Output", pidOutput);
            double clamped = Math.max(-12.0, Math.min(totalVoltage, 12.0));
            SmartDashboard.putNumber("Launcher/Clamped Voltage", clamped);
    
            hoodBagMotor.setVoltage(clamped);
        }

        @Override
        public void inGameHoodOffset(double offset){
            
        }
    
        @Override
        public void setHoodAngle(Angle goalAngle) {
            if (goalAngle.in(Units.Degrees) > 30) {
                hoodGoalAngle = Units.Degrees.of(30);
            } else {
                hoodGoalAngle = goalAngle;
            }  
        }
    
    /**
     * Accumulates hood encoder motion. Must be called exactly once per robot loop,
     * before hoodControlLoop().
     */
    @Override
    public void periodic() {
        boolean connected = hoodEncoder.isConnected();
        SmartDashboard.putBoolean("Launcher/Hood Encoder Connected", connected);

        if (!connected) {
            return;
        }

        double currentRaw = hoodEncoder.get();

        if (!hoodZeroed) {
            // First valid reading since boot: this is our zero.
            lastRawAngle = currentRaw;
            cumulativeEncoderDegrees = 0.0;
            hoodZeroed = true;
            return;
        }

        // Handle 0/360 wraparound
        cumulativeEncoderDegrees += MathUtil.inputModulus(currentRaw - lastRawAngle, -180, 180);
        lastRawAngle = currentRaw;
    }

    @Override
    public Angle getHoodPosition() {
        return Units.Degrees.of(-cumulativeEncoderDegrees / HOOD_GEAR_RATIO);
    }
    
    @Override
    public AngularVelocity getLauncherVelocity() {
        AngularVelocity vel = launcherVelocitySignal.refresh().getValue();
        return vel;
    }

    @Override
    public void setLauncherPercentOutput(double goalOutput) {
        launchLeaderKraken.set(goalOutput);
        launchFollowerKraken.setControl(new Follower(launchLeaderKraken.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setKickerPercentOutput(double percentOutput) {
        kickerKraken.set(percentOutput);
    }

    @Override
    public AngularVelocity getKickerVelocity() {
        return kickerVelocitySignal.refresh().getValue();
    }
}
