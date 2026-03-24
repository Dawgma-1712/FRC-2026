package frc.robot.commandFactories;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class ShootOnTheMove {
    
    private static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
        
        double vel = exitVelocity.in(Units.MetersPerSecond);
        double angle = hoodAngle.in(Units.Radians);
        double dist = distance.in(Units.Meters);
        return Units.Seconds.of(dist / (vel * Math.cos(angle)));

    }

    private static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return Units.MetersPerSecond.of(vel.in(Units.RadiansPerSecond) * radius.in(Units.Meters));
    }

    private static Translation2d predictTargetPos(Translation2d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {

        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Units.Seconds);
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Units.Seconds);

        return new Translation2d(predictedX, predictedY);
    }

    public static void launcherShootOnTheMove(
        CommandSwerveDrivetrain drivetrain, 
        ChassisSpeeds fieldSpeeds, 
        Translation2d target, 
        LauncherSubsystem launcher, 
        int iterations
    ) {

        // laziest code I've ever written
        Translation2d robotTranslation = new Pose3d(drivetrain.getState().Pose).transformBy(LauncherConstants.ROBOT_TO_LAUNCHER_TRANSFORM).toPose2d().getTranslation();
        
        Distance distance = Units.Meters.of(robotTranslation.getDistance(target));
        double hoodAngleDegrees = launcher.hoodAngleMap.get(distance.in(Units.Meters));
        double rps = launcher.rpsMap.get(distance.in(Units.Meters));

        Translation2d predictedTarget = target;

        AngularVelocity angularVelocity = Units.RotationsPerSecond.of(rps);
        LinearVelocity linearVelocity = angularToLinearVelocity(angularVelocity, Units.Inches.of(LauncherConstants.FLYWHEEL_WHEEL_DIAMETER / 2));
        Angle hoodAngle = Units.Degrees.of(hoodAngleDegrees);
        
        Time timeOfFlight = calculateTimeOfFlight(linearVelocity, hoodAngle, distance);

        for(int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);

            distance = Units.Meters.of(robotTranslation.getDistance(predictedTarget));
            hoodAngleDegrees = launcher.hoodAngleMap.get(distance.in(Units.Meters));
            rps = launcher.rpsMap.get(distance.in(Units.Meters));
            angularVelocity = Units.RotationsPerSecond.of(rps);
            linearVelocity = angularToLinearVelocity(angularVelocity, Units.Inches.of(LauncherConstants.FLYWHEEL_WHEEL_DIAMETER / 2));
            hoodAngle = Units.Degrees.of(hoodAngleDegrees);

            timeOfFlight = calculateTimeOfFlight(linearVelocity, hoodAngle, distance);
        }

        launcher.setLauncherVelocity(angularVelocity);
        launcher.setHoodPosition(hoodAngle);
        launcher.predictedTarget = new Translation3d(predictedTarget);
    }

    public static Command shootOnTheMoveCommand(
        CommandSwerveDrivetrain drivetrain, 
        Translation2d target, 
        LauncherSubsystem launcher, 
        int iterations) {

            return Commands.run(() -> {
                ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
                launcherShootOnTheMove(drivetrain, fieldSpeeds, target, launcher, iterations);
            }, launcher);
    }

}