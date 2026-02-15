// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Constants.DriveConstants;
import frc.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveSlowMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.Vision.VisionReal;
import frc.robot.subsystems.Vision.VisionSim;
import edu.wpi.first.wpilibj.RobotBase;

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
  // private final LauncherSubsystem launcher;
  // private final LauncherIOReal launcherIOReal;

  // private final IntakeSubsystem intake;
  // private final IntakeIOReal intakeIOReal;
  private final VisionSubsystem vision;

  private final Joystick driver;
  private final Joystick operator;

  public RobotContainer() {
    

    // launcherIOReal = new LauncherIOReal();
    // launcher = new LauncherSubsystem(launcherIOReal);
    
    // intakeIOReal = new IntakeIOReal();
    // intake = new IntakeSubsystem(intakeIOReal);

    autoHandler = new ModularAutoHandler();


    driver = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
    operator = new Joystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Math.abs(-driver.getRawAxis(1)) > 0.2 ? -driver.getRawAxis(1) * MaxSpeed * speed : 0) // Drive forward with negative Y (forward)
            .withVelocityY(Math.abs(-driver.getRawAxis(0)) > 0.2 ? -driver.getRawAxis(0) * MaxSpeed * speed : 0) // Drive left with negative X (left)
            .withRotationalRate(Math.abs(-driver.getRawAxis(2) * MaxAngularRate) > 0.05 ? -driver.getRawAxis(2) * MaxAngularRate * speed : 0)
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
      new JoystickButton(driver, 1).whileTrue(drivetrain.applyRequest(() -> brake));

      new JoystickButton(driver, 8).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));
  }

  public Command getAutonomousCommand() {
    return autoHandler.getSelectedModularCommand();
  }
}
