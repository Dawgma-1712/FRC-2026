// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IdConstants {
    public static final int SHOOTER_MOTOR_ID = 0;
  }

  public static class ShooterConstants {

    public static final double GRAVITY = 9.8;
    public static final double SHOOTER_ANGLE = 30.0;
    public static final double HUB_HEIGHT_METERS = 1.8288;
    public static final double WHEEL_DIAMETER = 4; // in inches

    public static final double SHOOTER_KP = 0.1;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;

  }

}
