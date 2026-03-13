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
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;



public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;
    InterpolatingDoubleTreeMap radiansPerSecondMap;
    public InterpolatingDoubleTreeMap hoodAngleMap;

    private Angle hoodTarget = Units.Degrees.of(LauncherConstants.BASE_ANGLE);
    AngularVelocity targetVelocity;
    public Translation3d target;

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain) {

        this.io = io;
        this.drivetrain = drivetrain;
        this.target = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        this.radiansPerSecondMap = new InterpolatingDoubleTreeMap();
        this.hoodAngleMap = new InterpolatingDoubleTreeMap();

        hoodAngleMap.put(Units.Inches.of(101).in(Units.Meters), 17.0);
        hoodAngleMap.put(Units.Inches.of(75).in(Units.Meters), 25.0);
        hoodAngleMap.put(Units.Inches.of(45).in(Units.Meters), 16.0);

        hoodAngleMap.put(Units.Inches.of(100).in(Units.Meters), 17.0);
        radiansPerSecondMap.put(Units.Inches.of(100).in(Units.Meters), 100 * 2 * Math.PI);

        hoodAngleMap.put(Units.Inches.of(45).in(Units.Meters), 16.0);
        radiansPerSecondMap.put(Units.Inches.of(45).in(Units.Meters), 100 * 2 * Math.PI);

        hoodAngleMap.put(Units.Inches.of(75).in(Units.Meters), 25.0);

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

    public ShotData getShotData() {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d target = (alliance == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        return LaunchCalculations.iterativeMovingShotFromFunnelClearance(
            drivetrain.getState().Pose,
            fieldSpeedsFromRelativeSpeeds(drivetrain.getState().Speeds),
            target,
            3
        );
    }

    public ShotData getShotDataLookupTable() {

        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d target = (alliance == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        return LaunchCalculations.iterativeMovingShotLookupTable(
            drivetrain.getState().Pose, 
            fieldSpeedsFromRelativeSpeeds(drivetrain.getState().Speeds), 
            target, 
            3, 
            radiansPerSecondMap, 
            hoodAngleMap
        );

    }

    public void launcherLookupTable(Pose2d robotPose) {

        Translation3d target = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();

        Translation2d robotTranslation = robotPose.getTranslation();
        double distance = robotTranslation.getDistance(target.toTranslation2d());

        double hoodAngle = hoodAngleMap.get(distance);
        setHoodPosition(Units.Degrees.of(hoodAngle));
        setLauncherVelocity(Units.RotationsPerSecond.of(100));

    }

    @Override
    public void simulationPeriodic() {
        io.periodic();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Hood angle", getHoodPosition().in(Units.Degrees));
        SmartDashboard.putNumber("Launcher/Launcher Velocity", getLauncherVelocity().in(Units.RotationsPerSecond));
        SmartDashboard.putNumber("Target Velocity",getShotData().exitVelocity() / (2 * Math.PI));
        SmartDashboard.putNumber("Hood Goal", hoodTarget.in(Units.Degrees));
        io.hoodControlLoop();
    }
}