// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Constants.IntakeConstants;
import frc.Constants.LauncherConstants;
import frc.Constants.OperatorConstants;
import frc.Constants.RevolverConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.utils.FuelSim;

import java.util.Set;
import java.util.function.Supplier;

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

  // private final ModularAutoHandler autoHandler;

  public final FuelSim fuelSim;


  // subsystems
  private final LauncherSubsystem launcher;
  private final LauncherIO launcherIO;

  private final IntakeSubsystem intake;
  private final IntakeIO intakeIO;

  private final ClimberSubsystem climber;
  private final ClimberIO climberIO;

  private final RevolverSubsystem revolver;
  private final RevolverIO revolverIO;

 // private final VisionInterface visionInterface;
  //private final VisionSubsystem vision;


  private final Joystick driver;
  private final Joystick operator;

  // robot state
  private boolean intakeDeployed = false;

  public RobotContainer() {

    fuelSim = new FuelSim();

    if (RobotBase.isReal()) {

      this.launcherIO = new LauncherIORealBangBang();
      this.intakeIO = new IntakeIOReal();
      this.climberIO = new ClimberIOReal();
      this.revolverIO = new RevolverIOReal();
      //this.visionInterface = new VisionReal(this.drivetrain);

    } else {

      this.launcherIO = new LauncherIOSim(this.drivetrain, this.fuelSim);
      this.intakeIO = new IntakeIOSim();
      this.climberIO = new ClimberIOSim();
      this.revolverIO = new RevolverIOSim();
      //this.visionInterface = new VisionSim(this.drivetrain);

      // FuelSim stuff

      fuelSim.spawnStartingFuel();
      Distance robotLength = Units.Inches.of(27);
      Distance bumperHeight = Units.Inches.of(5);
      Supplier<Pose2d> poseSupplier = () -> drivetrain.getState().Pose;
      Supplier<ChassisSpeeds> speedSupplier = () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                                                                    drivetrain.getState().Speeds, 
                                                                    drivetrain.getState().Pose.getRotation()
                                                                  );
      fuelSim.registerRobot(
        robotLength.in(Units.Meters), 
        robotLength.in(Units.Meters), 
        bumperHeight.in(Units.Meters), 
        poseSupplier, 
        speedSupplier
      );

      Distance intakeXMin = Units.Inches.of(16.724);
      Distance intakeYMin = Units.Inches.of(-17.024);
      Distance intakeXMax = Units.Inches.of(25.14);
      Distance intakeYMax = Units.Inches.of(17.024);

      fuelSim.registerIntake(intakeXMin, intakeXMax, intakeYMin, intakeYMax, () -> true, () -> { launcherIO.intakeFuel(); });

      fuelSim.start();
      fuelSim.enableAirResistance();

      SmartDashboard.putData(Commands.runOnce(() -> {
            fuelSim.clearFuel();
            fuelSim.spawnStartingFuel();
        })
        .withName("Reset Fuel")
        .ignoringDisable(true));
    }


    this.launcher = new LauncherSubsystem(this.launcherIO, this.drivetrain);
    this.intake = new IntakeSubsystem(this.intakeIO);
    this.climber = new ClimberSubsystem(this.climberIO);
    this.revolver = new RevolverSubsystem(this.revolverIO);
    //this.vision = new VisionSubsystem(this.drivetrain, this.visionInterface);

    // autoHandler = new ModularAutoHandler();

    driver = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
    operator = new Joystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0) // Drive forward with negative Y (forward)
            .withVelocityY(Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2 ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0) // Drive left with negative X (left)
            // Field perspective is 90 degrees from driver perspective
            .withRotationalRate(Math.abs(driver.getRawAxis(OperatorConstants.DRIVER_RX)) > 0.1 ? -driver.getRawAxis(OperatorConstants.DRIVER_RX) * MaxAngularRate * speed : 0)
          )
    );
    
/* 
    intake.setDefaultCommand(intake.run(() -> {
      double joystickY = operator.getRawAxis(OperatorConstants.OPERATOR_RY);
      if (Math.abs(joystickY) > 0.1) { // deadband
          double targetAngle = MathUtil.interpolate(
              IntakeConstants.MIN_ANGLE,
              IntakeConstants.MAX_ANGLE,
              (joystickY + 1.0) / 2.0 // map -1 to 1 → 0 to 1
          );
          intake.setAngleDirect(Units.Degrees.of(targetAngle));
      } else {
          intake.holdPosition();
      }
    }));
*/
    configureBindings();
  }

  private void configureBindings() {

      new JoystickButton(driver, OperatorConstants.DRIVER_X).whileTrue(drivetrain.applyRequest(() -> brake));

      new JoystickButton(driver, OperatorConstants.DRIVER_RT).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));

      new JoystickButton(driver, OperatorConstants.DRIVER_LB).whileTrue(AutoLockAndShoot.autoLockAndShoot(
                                                                                          this.drivetrain,
                                                                                          this.launcher,
                                                                                          this.revolver,
                                                                                          () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2
                                                                                              ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0),
                                                                                          () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2
                                                                                              ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0)
                                                                                      ).repeatedly());


      new JoystickButton(driver, OperatorConstants.DRIVER_A).onTrue(
          Commands.runOnce(() -> {
              intakeDeployed = !intakeDeployed;
              if (intakeDeployed) {
                  intake.setAngleDirect(Units.Degrees.of(IntakeConstants.EXTENDED_INTAKE_ANGLE));
                  intake.setIntakeMotorSpeed(0.8);  // spin intake roller when deployed
              } else {
                  intake.setAngleDirect(Units.Degrees.of(0));  // stow
                  intake.setIntakeMotorSpeed(0);  // stop roller when stowed
              }
          }, intake)
      );

      new JoystickButton(operator, OperatorConstants.OPERATOR_A).whileTrue(
        Commands.run(() -> {
          revolver.setRevolverPercentOutput(RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
        }, revolver)
      );

      new JoystickButton(operator, OperatorConstants.OPERATOR_LB).whileTrue(
        Commands.run(() -> {
          launcher.setLauncherVelocity(Units.RotationsPerSecond.of(-50));
        })).whileFalse(
        Commands.runOnce(() -> {
          launcher.setLauncherPercentOutput(0);;
        })
      );

      new JoystickButton(operator, OperatorConstants.OPERATOR_RB).whileTrue(
        Commands.run(() -> {
          launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
        })
      ).onFalse(
        Commands.runOnce(() -> {
          launcher.setKickerPercentOutput(0);
        })
      );


      // new JoystickButton(driver, OperatorConstants.DRIVER_LT).whileTrue(launcher.adaptiveShoot(() -> launcher.calculateDistance()));
  }


  public Command getAutonomousCommand() {
    return Commands.run(() -> {

    });
  }
}
