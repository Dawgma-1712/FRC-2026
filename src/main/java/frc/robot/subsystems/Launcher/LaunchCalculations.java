package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.Constants.ShooterConstants;
import frc.Constants.FieldConstants;

public class LaunchCalculations {

    public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        return Units.Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
    }

    public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {

        double velocity = exitVelocity.in(Units.MetersPerSecond);
        double angle = Math.PI / 2 - hoodAngle.in(Units.Radians);
        double distanceMeters = distance.in(Units.Meters);

        return Units.Seconds.of(distanceMeters / (velocity * Math.cos(angle)));

    }

    public static AngularVelocity linearToAngularVelocity(LinearVelocity velocity, Distance radius) { 
        return Units.RadiansPerSecond.of(velocity.in(Units.MetersPerSecond) / radius.in(Units.Meters));
    }

    public static LinearVelocity angularToLinearVelocity(AngularVelocity velocity, Distance radius) {
        return Units.MetersPerSecond.of(velocity.in(Units.RadiansPerSecond) * radius.in(Units.Meters));
    }

    public static Translation3d predictTargetPosition(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Units.Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Units.Seconds);

        return new Translation3d(predictedX, predictedY, target.getZ());

    }

    public static ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget, Translation3d predictedTarget) {

        Distance DISTANCE_ABOVE_FUNNEL = Units.Inches.of(20);

        double x_dist = getDistanceToTarget(robot, predictedTarget).in(Units.Inches);
        double y_dist = predictedTarget
                .getMeasureZ()
                .minus(ShooterConstants.ROBOT_TO_LAUNCHER_TRANSFORM.getMeasureZ())
                .in(Units.Inches);
        double g = 386;
        double r = FieldConstants.FUNNEL_RADIUS.in(Units.Inches)
                * x_dist
                / getDistanceToTarget(robot, actualTarget).in(Units.Inches);
        double h = FieldConstants.FUNNEL_HEIGHT.plus(DISTANCE_ABOVE_FUNNEL).in(Units.Inches);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));

        if (Double.isNaN(v0) || Double.isNaN(theta)) {
            v0 = 0;
            theta = 0;
        }

        return new ShotData(
            linearToAngularVelocity(Units.InchesPerSecond.of(v0), Units.Inches.of(ShooterConstants.FLYWHEEL_WHEEL_DIAMETER / 2)),
            Units.Radians.of(theta), 
            predictedTarget
        );
    }
    
    public static ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot, ChassisSpeeds fieldSpeeds, Translation3d target, int iterations) {
        ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
        Distance distance = getDistanceToTarget(robot, target);
        Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
        Translation3d predictedTarget = target;

    
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPosition(target, fieldSpeeds, timeOfFlight);
            shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
            timeOfFlight = calculateTimeOfFlight(
                    shot.getExitVelocity(), shot.getHoodAngle(), getDistanceToTarget(robot, predictedTarget));
        }

        return shot;
    }
    
    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {

        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(Units.RadiansPerSecond), hoodAngle.in(Units.Radians), target);
        }
        
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.BLUE_HUB_POSE.getTranslation());
        }

        public ShotData(double exitVelocity, double hoodAngle) {
            this(exitVelocity, hoodAngle, FieldConstants.BLUE_HUB_POSE.getTranslation());
        }

        public LinearVelocity getExitVelocity() {
            return angularToLinearVelocity(Units.RadiansPerSecond.of(this.exitVelocity), Units.Inches.of(ShooterConstants.FLYWHEEL_WHEEL_DIAMETER / 2));
        }

        public Angle getHoodAngle() {
            return Units.Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }
    }

}
