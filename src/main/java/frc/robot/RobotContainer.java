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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.SwerveSlowMode;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Climber.*;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Revolver.*;

import frc.robot.commandFactories.ShootOnTheMove;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final VisionInterface visionInterface;
  private final VisionSubsystem vision;

  private final VisionInterface visionInterfaceBack;
  private final VisionSubsystemBack visionSubsystemBack;


  private final Joystick driver;
  private final Joystick operator;

  // robot state
  private boolean intakeDeployed = false;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    fuelSim = new FuelSim();

    if (RobotBase.isReal()) {

      this.launcherIO = new LauncherIOReal();
      this.intakeIO = new IntakeIOReal();
      this.climberIO = new ClimberIOReal();
      this.revolverIO = new RevolverIOReal();
      this.visionInterface = new VisionReal(this.drivetrain);
      this.visionInterfaceBack = new VisionRealBackIO(this.drivetrain);

    } else {

      this.launcherIO = new LauncherIOSim(this.drivetrain, this.fuelSim);
      this.intakeIO = new IntakeIOSim();
      this.climberIO = new ClimberIOSim();
      this.revolverIO = new RevolverIOSim();
      this.visionInterface = new VisionSim(this.drivetrain);
      this.visionInterfaceBack = new VisionSim(this.drivetrain);

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
    }


    this.launcher = new LauncherSubsystem(this.launcherIO, this.drivetrain);
    this.intake = new IntakeSubsystem(this.intakeIO);
    this.climber = new ClimberSubsystem(this.climberIO);
    this.revolver = new RevolverSubsystem(this.revolverIO);
    this.vision = new VisionSubsystem(this.drivetrain, this.visionInterface);
    this.visionSubsystemBack = new VisionSubsystemBack(this.drivetrain, this.visionInterfaceBack);
    
    NamedCommands.registerCommand("Shoot", getAutoPreloads());
    NamedCommands.registerCommand("Intake", Commands.sequence(
                                                                          Commands.runOnce(() -> {
                                                                            intake.setAngleDirect(Units.Degrees.of(IntakeConstants.EXTENDED_INTAKE_ANGLE));
                                                                          }),
                                                                          Commands.runOnce(() -> {
                                                                            intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_DUTY_CYCLE);
                                                                          })                                                                         
      ));
      NamedCommands.registerCommand("Stow Intake", Commands.sequence(
        
                                                                          Commands.runOnce(() -> {
                                                                            intake.setIntakeMotorSpeed(0);
                                                                          }),
                                                                          Commands.runOnce(() -> {
                                                                            intake.setAngleDirect(Units.Degrees.of(IntakeConstants.STOWED_INTAKE_ANGLE));
                                                                          })

      ));
    

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

    // intake manual trigger
    Supplier<Double> ltSupplier = () -> operator.getRawAxis(OperatorConstants.OPERATOR_LT);
    intake.setDefaultCommand(
      Commands.run(() -> {
        intake.manualTriggerIntakeSpeed(ltSupplier);
      }, intake)
    );
      autoChooser = AutoBuilder.buildAutoChooser();
      autoChooser.addOption("OuterBumpRight", new PathPlannerAuto("OuterBumpRight"));
      autoChooser.addOption("InnerBumpRight", new PathPlannerAuto("InnerBumpRight"));
      autoChooser.addOption("InnerBumpLeftShoot", new PathPlannerAuto("InnerBumpLeftShoot"));
      autoChooser.setDefaultOption("OuterBumpRight", new PathPlannerAuto("OuterBumpRight"));
      SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {

    // shoot sequence
    Timer shootTimer = new Timer();
    new JoystickButton(driver, OperatorConstants.DRIVER_RT).whileTrue(
        Commands.run(() -> {
          /*
            Pose2d robotPose = drivetrain.getState().Pose;
            launcher.launcherLookupTable(robotPose);
          */

            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
            ShootOnTheMove.launcherShootOnTheMove(drivetrain, fieldSpeeds, launcher.target.toTranslation2d(), launcher, 1);

            if (shootTimer.hasElapsed(0.2)) {
                revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
                launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
            }
        }, launcher, revolver)
        .alongWith(new AutoLock(
                                                                                        this.drivetrain,
                                                                                        () -> launcher.predictedTarget.toTranslation2d(),
                                                                                        () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2
                                                                                            ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0),
                                                                                        () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2
                                                                                            ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0)
                                                                                    ).repeatedly()) 
        .beforeStarting(() -> shootTimer.restart())
        .finallyDo(() -> {
            revolver.setRevolverPercentOutput(0);
            launcher.setKickerPercentOutput(0);
            launcher.setLauncherVelocity(Units.RadiansPerSecond.of(0)); 
            launcher.setHoodPosition(Units.Degrees.of(0));
        })
    );
    
    // slow mode
    new JoystickButton(driver, OperatorConstants.DRIVER_RB).onTrue(new SwerveSlowMode(0.15)).onFalse(new SwerveSlowMode(1));
    
    // x mode
    new JoystickButton(driver, OperatorConstants.DRIVER_X).whileTrue(drivetrain.applyRequest(() -> brake));
    
    // auto align
    new JoystickButton(driver, OperatorConstants.DRIVER_LT).whileTrue(new AutoLock(
                                                                                        this.drivetrain,
                                                                                        () -> launcher.target.toTranslation2d(),
                                                                                        () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LY)) > 0.2
                                                                                            ? -driver.getRawAxis(OperatorConstants.DRIVER_LY) * MaxSpeed * speed : 0),
                                                                                        () -> (Math.abs(-driver.getRawAxis(OperatorConstants.DRIVER_LX)) > 0.2
                                                                                            ? -driver.getRawAxis(OperatorConstants.DRIVER_LX) * MaxSpeed * speed : 0)
                                                                                    ).repeatedly());

      // spin the kicker and revolver wheels, and set the hood angle to the calculated angle                                                                               
      new JoystickButton(driver, OperatorConstants.DRIVER_Y).whileTrue(Commands.defer(() -> {

          return Commands.run(() -> {
            launcher.setLauncherVelocity(Units.RotationsPerSecond.of(100));
            launcher.setHoodPosition(Units.Degrees.of(launcher.hoodAngleMap.get(Units.Inches.of(89.0).in(Units.Meters))));
          });

      }, Set.of(launcher))
      .finallyDo(() -> {
        launcher.setLauncherVelocity(Units.RadiansPerSecond.of(0)); 
        launcher.setHoodPosition(Units.Degrees.of(0));
      }));

      new JoystickButton(driver, OperatorConstants.DRIVER_B).whileTrue(Commands.run(() -> {
        revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
        launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
      }).finallyDo(
        () -> {
        revolver.setRevolverPercentOutput(-0);
        launcher.setKickerPercentOutput(0);
        }
      ));


      new JoystickButton(driver, OperatorConstants.DRIVER_A).onTrue(
          Commands.runOnce(() -> {
              intakeDeployed = !intakeDeployed;
              if (intakeDeployed) {
                  intake.setAngleDirect(Units.Degrees.of(IntakeConstants.EXTENDED_INTAKE_ANGLE));
                  intake.setIntakeMotorSpeed(0.8);  // spin intake roller when deployed
              } else {
                  intake.setAngleDirect(Units.Degrees.of(IntakeConstants.STOWED_INTAKE_ANGLE));  // stow
                  intake.setIntakeMotorSpeed(0);  // stop roller when stowed
              }
          }, intake)
      );

      new JoystickButton(driver, OperatorConstants.DRIVER_START).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // launcher unloading
    new JoystickButton(operator, OperatorConstants.OPERATOR_X).whileTrue(
        Commands.run(() -> {
          launcher.setLauncherPercentOutput(0.6);
          launcher.setKickerPercentOutput(0.6);
        }).finallyDo(() -> {
          launcher.setLauncherPercentOutput(0);
          launcher.setKickerPercentOutput(0);
        })
    );
    
    // launch at a lower speed, without adjusting the hood angle
    Timer operatorShootTimer = new Timer();
    new JoystickButton(operator, OperatorConstants.OPERATOR_B).whileTrue(
        Commands.run(() -> {
            launcher.setLauncherVelocity(Units.RotationsPerSecond.of(70));
            if (operatorShootTimer.hasElapsed(0.2)) {
                revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
                launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
            }
        }, launcher, revolver) 
        .beforeStarting(() -> operatorShootTimer.restart())
        .finallyDo(() -> {
            revolver.setRevolverPercentOutput(0);
            launcher.setKickerPercentOutput(0);
            launcher.setLauncherVelocity(Units.RadiansPerSecond.of(0)); 
        })
    );
    
    // angle manual up?
    new JoystickButton(operator, OperatorConstants.OPERATOR_Y).whileTrue(
      Commands.run(() -> {
        intake.setAngleMotorSpeed(-0.1);
      }, intake)
    );

    // angle manual down?
    new JoystickButton(operator, OperatorConstants.OPERATOR_A).whileTrue(
      Commands.run(() -> {
        intake.setAngleMotorSpeed(0.1);
      }, intake)
    );

    // spindexer manual CCW
    new JoystickButton(operator, OperatorConstants.OPERATOR_RB).whileTrue(
      Commands.run(() -> {
        revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
      }, revolver).finallyDo(() -> {
        revolver.setRevolverPercentOutput(0);
      })
    );

    // spindexer manual CW
    new JoystickButton(operator, OperatorConstants.OPERATOR_LB).whileTrue(
      Commands.run(() -> {
        revolver.setRevolverPercentOutput(RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
      }, revolver).finallyDo(() -> {
        revolver.setRevolverPercentOutput(0);
      })
    );

    // hood angle increase
    new POVButton(operator, 0).whileTrue(
      Commands.run(() -> {
        launcher.setHoodPosition(launcher.getHoodPosition().plus(Units.Degrees.of(0.1)));
      })
    );
    
    // hood angle decrease
    new POVButton(operator, 180).whileTrue(
      Commands.run(() -> {
        launcher.setHoodPosition(launcher.getHoodPosition().minus(Units.Degrees.of(0.1)));
      })
    );

      // new JoystickButton(driver, OperatorConstants.DRIVER_LT).whileTrue(launcher.adaptiveShoot(() -> launcher.calculateDistance()));
  }


  public Command getAutoPreloads() {
    Command autoPreloads;
    Command prepAutoPreloads;
    Command launchAutoPreloads;
    
      prepAutoPreloads = Commands.defer(() -> {

          return Commands.run(() -> {
            launcher.launcherLookupTable(drivetrain.getState().Pose);
          });

      }, Set.of(launcher))
      
      .finallyDo(() -> {
        launcher.setLauncherVelocity(Units.RadiansPerSecond.of(0)); 
        launcher.setHoodPosition(Units.Degrees.of(0));
      });

      launchAutoPreloads = new SequentialCommandGroup(new WaitCommand(1), Commands.run(() -> {
        revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
        launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
      }).finallyDo(
        () -> {
        revolver.setRevolverPercentOutput(-0);
        launcher.setKickerPercentOutput(0);
        }
      ));

      autoPreloads = Commands.parallel(prepAutoPreloads.raceWith(new WaitCommand(LauncherConstants.SHOOT_WAIT_LENGTH)), launchAutoPreloads.raceWith(new WaitCommand(LauncherConstants.SHOOT_WAIT_LENGTH)));
    return autoPreloads;
  }

  public Command getAutonomousCommand() {
    drivetrain.configureAutoBuilder();
    return autoChooser.getSelected();
  }
}
