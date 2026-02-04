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

    public static final int INTAKE_STOWED_SWITCH_ID = 0;
    public static final int INTAKE_EXTENDED_SWITCH_ID = 1;
    public static final int LAUNCHER_FUEL_DETECTION_BEAMBREAK_ID = 2;

    public static final int CLIMBER_MOTOR=1;

  }

  public static class IntakeConstants {

    public static final double POSITION_TOLERANCE = 2.0;  // in degrees

    public static final double ANGLE_kP = 2.4;
    public static final double ANGLE_kI = 0.0;
    public static final double ANGLE_kD = 0.0;

    public static final double ANGLE_kS = 0.25;
    public static final double ANGLE_kV = 0.12;
    public static final double ANGLE_kA = 0.01;

    public static final double ANGLE_CRUISE_VELOCITY = 40;
    public static final double ANGLE_ACCELERATION = 80;
    public static final double ANGLE_JERK = 1600;

    public static final double EXTENDED_INTAKE_ANGLE = 90;

  }

  public static class ShooterConstants {

    public static final double GRAVITY = 9.8;
    public static final double SHOOTER_ANGLE = 30.0;
    public static final double HUB_HEIGHT_METERS = 1.8288;
    public static final double WHEEL_DIAMETER = 4; // in inches

    public static final double SHOOTER_kP = 0.1;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0;

    public static final double FEEDER_kP = 0.1; //These PID values are arbitrary
    public static final double FEEDER_kI = 0.0;
    public static final double FEEDER_kD = 0.0;

    public static final double FEEDER_SPEED_MPS = 15; //idk change

    public static final double TARGET_VELOCITY_TOLERANCE = 0.95;

  }

  public static class VisionConstants {

    public static final AprilTagFieldLayout APRIL_TAG_POSES = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  }

  public static class ClimberConstants{
    public static final double CLIMBER_kP = 0;
    public static final double CLIMBER_kI = 0;
    public static final double CLIMBER_kD = 0;

    public static final double ANGLE_CRUISE_VELOCITY = 40;
    public static final double ANGLE_ACCELERATION = 80;
    public static final double ANGLE_JERK = 1600;

  }



}