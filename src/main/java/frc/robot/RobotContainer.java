// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Constants;
import frc.Constants.DriveConstants;
import frc.Constants.FieldConstants;
import frc.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveSlowMode;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.*;

import java.nio.ReadOnlyBufferException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Revolver.RevolverIOReal;
import frc.robot.subsystems.Revolver.RevolverSubsystem;
import frc.robot.subsystems.Swerve.AutoLock;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionReal;
import frc.robot.subsystems.Vision.VisionSim;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Intake.*;

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
  private final LauncherIOReal launcherIOReal;

  private final IntakeSubsystem intake;
  private final IntakeIOReal intakeIOReal;

  private final ClimberSubsystem climber;
  private final ClimberIOReal climberIOReal;

  private final RevolverSubsystem revolver;
  private final RevolverIOReal revolverIOReal;

  private final VisionSubsystem vision;


  private final Joystick driver;
  private final Joystick operator;

  public RobotContainer() {
    launcherIOReal = new LauncherIOReal();
    launcher = new LauncherSubsystem(launcherIOReal, drivetrain);
    
    intakeIOReal = new IntakeIOReal();
    intake = new IntakeSubsystem(intakeIOReal);

    climberIOReal = new ClimberIOReal();
    climber = new ClimberSubsystem(climberIOReal);

    revolverIOReal = new RevolverIOReal();
    revolver = new RevolverSubsystem(revolverIOReal);
    

    autoHandler = new ModularAutoHandler();


    driver = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
    operator = new Joystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0) // Drive forward with negative Y (forward)
            .withVelocityY(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0) // Drive left with negative X (left)
            .withRotationalRate(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_RX) * MaxAngularRate) > 0.05 ? -driver.getRawAxis(OperatorConstants.DRIVER_RX) * MaxAngularRate * speed : 0)
            )
    );

    if (RobotBase.isReal()) {

      this.vision = new VisionSubsystem(drivetrain, new VisionReal(drivetrain));

    } else {

      this.vision = new VisionSubsystem(drivetrain, new VisionSim(drivetrain));
    }

    configureBindings();
  }

  private void configureBindings() {

      new JoystickButton(driver, OperatorConstants.DRIVER_X).whileTrue(drivetrain.applyRequest(() -> brake));

      new JoystickButton(driver, OperatorConstants.DRIVER_RT).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));

      new JoystickButton(driver, OperatorConstants.DRIVER_RT).whileTrue(
            new AutoLock(
                drivetrain,
                FieldConstants.BLUE_HUB_POSE2D, // auto lock target set to hub pose2d in actual 2026 robot code
                () -> Math.abs(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0), //x supplier
                () -> Math.abs(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0) //y supplier
            )
        );

      // new JoystickButton(driver, OperatorConstants.DRIVER_LT).whileTrue(launcher.adaptiveShoot(() -> launcher.calculateDistance()));
  }

  public Command getAutonomousCommand() {
    return autoHandler.getSelectedModularCommand();
  }
}
