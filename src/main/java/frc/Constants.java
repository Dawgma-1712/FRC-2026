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
    public static final int SHOOT_MOTOR_ID1 = 3;//arbitrary all
    public static final int SHOOT_MOTOR_ID2 = 4;
    public static final int FEED_MOTOR_ID = 5;
    public static final int HOOD_MOTOR_ID = 6;

    public static final int ANGLE_MOTOR_ID = 1; //arbitrary value
    public static final int INTAKE_MOTOR_ID = 2;// arbitrary value

    public static final int INTAKE_LIMIT_SWITCH_ID = 0;
    public static final int LAUNCHER_FUEL_DETECTION_BEAMBREAK_ID = 1;

  }

  public static class ShooterConstants {

    public static final double GRAVITY = 9.8;
    public static final double SHOOTER_ANGLE = 30.0;
    public static final double HUB_HEIGHT_METERS = 1.8288;
    public static final double WHEEL_DIAMETER = 4; // in inches

    public static final double SHOOTER_KP1 = 0.1;
    public static final double SHOOTER_KI1 = 0.0;
    public static final double SHOOTER_KD1 = 0.0;

    public static final double FEEDER_KP2 = 0.1; //These PID values are arbitrary
    public static final double FEEDER_KI2 = 0.0;
    public static final double FEEDER_KD2 = 0.0;

  }

  public static class VisionConstants {

    public static final AprilTagFieldLayout APRIL_TAG_POSES = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  }

}
