package frc;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// Remember to use the correct subclass so our code stays organized please
public final class Constants {

  public static class DriveConstants {
        // Auto constants
        public static final int drive_kP = 20;
        public static final int drive_kI = 0;
        public static final int drive_kD = 0;

        public static final int rotation_kP = 7;
        public static final int rotation_kI = 0;
        public static final int rotation_kD = 0;
        public static final PathConstraints GAME_CONSTRAINTS = new PathConstraints(0.3, 0.4, Units.degreesToRadians(54), Units.degreesToRadians(72));
    }

  public static class OperatorConstants {
    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;

    public static final int DRIVER_LX = 0;
    public static final int DRIVER_LY = 1;
    public static final int DRIVER_RX = 2;
    public static final int DRIVER_RY = 3;

    public static final int DRIVER_X = 1;
    public static final int DRIVER_Y = 4;
    public static final int DRIVER_A = 2;
    public static final int DRIVER_B = 3;

    public static final int DRIVER_LB = 5;
    public static final int DRIVER_RB = 6;
    public static final int DRIVER_LT = 7;
    public static final int DRIVER_RT = 8;

    public static final int DRIVER_BACK = 9;
    public static final int DRIVER_START = 10;

    public static final int DRIVER_LS = 11;
    public static final int DRIVER_RS = 12;

    public static final int OPERATOR_LX = 0;
    public static final int OPERATOR_LY = 1;
    public static final int OPERATOR_RX = 4;
    public static final int OPERATOR_RY = 5;
    public static final int OPERATOR_LT = 2;
    public static final int OPERATOR_RT = 3;

    public static final int OPERATOR_A = 1;
    public static final int OPERATOR_B = 2;
    public static final int OPERATOR_X = 3;
    public static final int OPERATOR_Y = 4;

    public static final int OPERATOR_LB = 5;
    public static final int OPERATOR_RB = 6;

    public static final int OPERATOR_BACK = 7;
    public static final int OPERATOR_START = 8;

    public static final int OPERATOR_LS = 9;
    public static final int OPERATOR_RS = 10;
  }

  public static class IdConstants {

    public static final int KICK_MOTOR_ID = 3;//arbitrary all
    public static final int FEED_MOTOR_ID = 5;
    public static final int HOOD_MOTOR_ID = 6;

    public static final int REVOLVER_MOTOR_ID = 7;

    public static final int ANGLE_MOTOR_ID = 1; //arbitrary value
    public static final int INTAKE_MOTOR_ID = 2;// arbitrary value

    public static final int INTAKE_STOWED_SWITCH_ID = 0;
    public static final int INTAKE_EXTENDED_SWITCH_ID = 1;
    public static final int LAUNCHER_FUEL_DETECTION_BEAMBREAK_ID = 2;

    public static final int INTAKE_ANGLE_ENCODER_ID = 3;
    
    public static final int HOOD_ENCODER_ID = 3;

    public static final int CLIMBER_MOTOR = 1;

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

    public static final double ANGLE_REDUCTION = 200;  // arbitrary value, PLEASE CHANGE

    public static class SimulationConstants {

      public static final double ARM_LENGTH = Units.inchesToMeters(21);
      public static final double ARM_MASS = 8;  // in kilograms
      public static final double MIN_ANGLE = Units.degreesToRadians(0);
      public static final double MAX_ANGLE = Units.degreesToRadians(90);
      public static final double ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / 2048;  // in radians, our encoder has 2048 pulses per rotation

    }

  }

  public static class ShooterConstants {

    public static final double GRAVITY = 9.8;    
    public static final double HUB_HEIGHT_METERS = 1.8288;
    public static final double LAUNCHER_HEIGHT_METERS = 1; //completely arbitrary
    public static final double WHEEL_DIAMETER = 4; // in inches

    public static final double KICKER_kP = 999999;
    public static final double KICKER_PEAK_FORWARD_TORQUE_CURRENT = 40.0;
    public static final double KICKER_PEAK_REVERSE_TORQUE_CURRENT = 0.0;
    public static final double KICKER_PEAK_FORWARD_DUTY_CYCLE = 1.0;
    public static final double KICKER_PEAK_REVERSE_DUTY_CYCLE = 0.0;

    public static final double FEEDER_kP = 0.1; //These PID values are arbitrary
    public static final double FEEDER_kI = 0.0;
    public static final double FEEDER_kD = 0.0;

    public static final double FEEDER_SPEED_MPS = 15; //idk change

    public static final double TARGET_VELOCITY_TOLERANCE = 0.95;
    public static final double TARGET_HOOD_TOLERANCE_DEGREES = 1.0;

    public static final double BASE_ANGLE = 9.0;
    public static final double MAX_HOOD_OFFSET = 40.0;

  }

  public static class VisionConstants {

    public static final AprilTagFieldLayout APRIL_TAG_POSES = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final Pose3d BLUE_HUB_POSE = new Pose3d(
        new Translation3d(Units.feetToMeters(15.4825), Units.feetToMeters(13.48), Units.feetToMeters(6)),
        new Rotation3d(0, 0, 0)
    );
    
    public static final String LIMELIGHT_LEFT_ID = "limelight-left";
    public static final String LIMELIGHT_RIGHT_ID = "limelight-right";

    public static final Transform3d LIMELIGHT_LEFT_TO_ROBOT = new Transform3d(
                                                                  new Translation3d(-0, 0.276, 0.1524),
                                                                  new Rotation3d(0, Math.toRadians(-30), Math.toRadians(135)
                                                              ));
    public static final Transform3d LIMELIGHT_RIGHT_TO_ROBOT = new Transform3d(
                                                                  new Translation3d(-0, -0.276, 0.1524), 
                                                                  new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-135)
                                                              ));

    public static class SimulationConstants {

      public static final double LIMELIGHT_VIEW_RANGE = 5.0; // distance in meters that the limelight can see
      public static final double LIMELIGHT_HORIZONTAL_FOV = Math.toRadians(82); // in radians
      public static final double LIMELIGHT_VERTICAL_FOV = Math.toRadians(56.2); // in radians
      public static final double INCHES_PER_METER = 39.37;

    }

  }

  public static class RevolverConstants {
    
  }


  public static class ClimberConstants {
    public static final double CLIMBER_kP = 0;
    public static final double CLIMBER_kI = 0;
    public static final double CLIMBER_kD = 0;

    public static final double CLIMBER_kS = 0.25;
    public static final double CLIMBER_kV = 0.12;
    public static final double CLIMBER_kA = 0.01;

    public static final double ANGLE_CRUISE_VELOCITY = 40;
    public static final double ANGLE_ACCELERATION = 80;
    public static final double ANGLE_JERK = 1600;

  }



}