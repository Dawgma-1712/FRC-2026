package frc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

// Remember to use the correct subclass so our code stays organized please
public final class Constants {

  public static class OperatorConstants {
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;
  }

  public static class IdConstants {
    public static final int SHOOTER_MOTOR_ID = 0;
    public static final int INTAKE_MOTOR_ID = 2;// arbitrary value
    public static final int ANGLE_MOTOR_ID = 1; //arbitrary value

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

  public static class VisionConstants {

    public static final AprilTagFieldLayout APRIL_TAG_POSES = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  }

}
