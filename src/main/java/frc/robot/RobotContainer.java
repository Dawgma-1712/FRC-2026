// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  private final ModularAutoHandler autoHandler;

  // subsystems
  private final Shooter shooter;

  private final Joystick driver;
  private final Joystick operator;

  public RobotContainer() {

    shooter = new Shooter();
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
