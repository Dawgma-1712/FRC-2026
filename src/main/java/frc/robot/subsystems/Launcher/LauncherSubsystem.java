package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Translation2d;

import frc.Constants.FieldConstants;
import frc.Constants.LauncherConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;



public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;
    public InterpolatingDoubleTreeMap hoodAngleMap;
    public InterpolatingDoubleTreeMap rpsMap;

    private Angle hoodTarget = Units.Degrees.of(LauncherConstants.BASE_ANGLE);
    AngularVelocity targetVelocity;
    public Translation3d target;

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain) {

        this.io = io;
        this.drivetrain = drivetrain;
        this.target = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        this.hoodAngleMap = new InterpolatingDoubleTreeMap();
        this.rpsMap = new InterpolatingDoubleTreeMap();

        hoodAngleMap.put(Units.Inches.of(44).in(Units.Meters), 7.0);
        rpsMap.put(Units.Inches.of(44).in(Units.Meters), 50.0);

        hoodAngleMap.put(Units.Inches.of(68).in(Units.Meters), 10.0);
        rpsMap.put(Units.Inches.of(68).in(Units.Meters), 50.0);

        hoodAngleMap.put(Units.Inches.of(97).in(Units.Meters), 16.0);
        rpsMap.put(Units.Inches.of(97).in(Units.Meters), 50.0);

        hoodAngleMap.put(Units.Inches.of(120).in(Units.Meters), 21.0);
        rpsMap.put(Units.Inches.of(120).in(Units.Meters), 55.0);

        hoodAngleMap.put(Units.Inches.of(140).in(Units.Meters), 25.0);
        rpsMap.put(Units.Inches.of(140).in(Units.Meters), 55.0);

        hoodAngleMap.put(Units.Inches.of(176.5).in(Units.Meters), 25.0);
        rpsMap.put(Units.Inches.of(176.5).in(Units.Meters), 60.0);

        hoodAngleMap.put(Units.Inches.of(211).in(Units.Meters), 35.0);
        rpsMap.put(Units.Inches.of(211).in(Units.Meters), 65.0);

    }

    public void setLauncherVelocity(AngularVelocity velocity) {
        io.setLauncherVelocity(velocity);
    }

    public AngularVelocity getLauncherVelocity() {
        return io.getLauncherVelocity();
    }

    public void setKickerPercentOutput(double percentOutput) {
        io.setKickerPercentOutput(percentOutput);
    }

    public AngularVelocity getKickerVelocity() {
        return io.getKickerVelocity();
    }

    public boolean hasFuelIntaked() {
        return io.hasFuelIntaked();
    }

    public void setHoodPosition(Angle angle) {
        hoodTarget = angle;
        io.setHoodAngle(angle);
    }

    public Angle getHoodPosition() {
        return io.getHoodPosition();
    }

    public void setLauncherPercentOutput(double goalOutput) {
        io.setLauncherPercentOutput(goalOutput);
    }

    public void stop() {
        setLauncherVelocity(Units.RotationsPerSecond.of(0));
        setKickerPercentOutput(0);
    }

    public boolean hoodAtPosition() {
        double targetDegrees = hoodTarget.in(Units.Degrees);
        double currentDegrees = getHoodPosition().in(Units.Degrees);
        return Math.abs(targetDegrees - currentDegrees) < LauncherConstants.TARGET_HOOD_TOLERANCE_DEGREES;
    }

    public ChassisSpeeds fieldSpeedsFromRelativeSpeeds(ChassisSpeeds relativeSpeeds) {
        
        Rotation2d rotation = drivetrain.getState().Pose.getRotation();
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, rotation);

    }

    public void launcherLookupTable(Pose2d robotPose) {

        Translation3d target = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        Translation2d robotTranslation = robotPose.getTranslation();
        double distance = robotTranslation.getDistance(target.toTranslation2d());

        double hoodAngle = hoodAngleMap.get(distance);
        AngularVelocity velocity = Units.RotationsPerSecond.of(rpsMap.get(distance));
        setHoodPosition(Units.Degrees.of(hoodAngle));
        setLauncherVelocity(velocity);

    }

    @Override
    public void simulationPeriodic() {
        io.periodic();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Hood angle", getHoodPosition().in(Units.Degrees));
        SmartDashboard.putNumber("Launcher/Launcher Velocity", getLauncherVelocity().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Hood Goal", hoodTarget.in(Units.Degrees));
        io.hoodControlLoop();
    }
}