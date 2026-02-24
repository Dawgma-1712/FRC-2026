// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveSlowMode;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Climber.*;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Revolver.*;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.commandFactories.AutoLockAndShoot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Revolver.RevolverIOReal;
import frc.robot.subsystems.Revolver.RevolverSubsystem;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.Vision.*;

public class RobotContainer {
  public static double speed = 1;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final ModularAutoHandler autoHandler;

  // subsystems
  private final LauncherSubsystem launcher;
  private final LauncherIO launcherIO;

  private final IntakeSubsystem intake;
  private final IntakeIO intakeIO;

  private final ClimberSubsystem climber;
  private final ClimberIO climberIO;

  private final RevolverSubsystem revolver;
  private final RevolverIO revolverIO;

  private final VisionInterface visionInterface;
  private final VisionSubsystem vision;


  private final Joystick driver;
  private final Joystick operator;

  public RobotContainer() {

    if (RobotBase.isReal()) {

      this.launcherIO = new LauncherIOReal();
      this.intakeIO = new IntakeIOReal();
      this.climberIO = new ClimberIOReal();
      this.revolverIO = new RevolverIOReal();
      this.visionInterface = new VisionReal(this.drivetrain);

    } else {

      this.launcherIO = new LauncherIOSim(this.drivetrain);
      this.intakeIO = new IntakeIOSim();
      this.climberIO = new ClimberIOSim();
      this.revolverIO = new RevolverIOSim();
      this.visionInterface = new VisionSim(this.drivetrain);

    }
    

    this.launcher = new LauncherSubsystem(this.launcherIO, this.drivetrain);
    this.intake = new IntakeSubsystem(this.intakeIO);
    this.climber = new ClimberSubsystem(this.climberIO);
    this.revolver = new RevolverSubsystem(this.revolverIO);
    this.vision = new VisionSubsystem(this.drivetrain, this.visionInterface);

    autoHandler = new ModularAutoHandler();


    driver = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
    operator = new Joystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0) // Drive forward with negative Y (forward)
            .withVelocityY(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0) // Drive left with negative X (left)
            // Field perspective is 90 degrees from driver perspective
            .withRotationalRate(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_RX) * MaxAngularRate) > 0.05 ? -driver.getRawAxis(OperatorConstants.DRIVER_RX) * MaxAngularRate * speed : 0)
            )
    );

    configureBindings();
  }

  private void configureBindings() {

      new JoystickButton(driver, OperatorConstants.DRIVER_X).whileTrue(drivetrain.applyRequest(() -> brake));

      new JoystickButton(driver, OperatorConstants.DRIVER_RT).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));

      new JoystickButton(driver, OperatorConstants.DRIVER_LB).whileTrue(new RepeatCommand(AutoLockAndShoot.autoLockAndShoot(
                                                                     this.drivetrain, 
                                                                     this.launcher,
                                                                     this.revolver,
                                                                    () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0), //x supplier
                                                                    () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0)) //y supplier
                                                              ));


      // new JoystickButton(driver, OperatorConstants.DRIVER_LT).whileTrue(launcher.adaptiveShoot(() -> launcher.calculateDistance()));
  }

  public Command getAutonomousCommand() {
    return autoHandler.getSelectedModularCommand();
  }
}
