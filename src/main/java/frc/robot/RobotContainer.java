// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Intake.*;

public class RobotContainer {

  private final ModularAutoHandler autoHandler;

  // subsystems
  private final LauncherSubsystem launcher;
  private final LauncherIOReal launcherIOReal;

  private final Joystick driver;
  private final Joystick operator;

  public RobotContainer() {

    launcherIOReal = new LauncherIOReal();
    launcher = new LauncherSubsystem(launcherIOReal);

    autoHandler = new ModularAutoHandler();

    driver = new Joystick(OperatorConstants.DRIVER_JOYSTICK_PORT);
    operator = new Joystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return autoHandler.getSelectedModularCommand();
  }
}
