package frc.robot.subsystems.Launcher;

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

import frc.Constants.FieldConstants;
import frc.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;



public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;
    InterpolatingDoubleTreeMap radiansPerSecondMap;
    InterpolatingDoubleTreeMap hoodAngleMap;

    private Angle hoodTarget = Units.Degrees.of(LauncherConstants.BASE_ANGLE);

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain) {

        this.io = io;
        this.drivetrain = drivetrain;

        this.radiansPerSecondMap = new InterpolatingDoubleTreeMap();
        this.hoodAngleMap = new InterpolatingDoubleTreeMap();

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

    public void followLaunchCalculations() {

        ShotData currentShot = getShotData();
        setLauncherVelocity(Units.RadiansPerSecond.of(currentShot.exitVelocity()));

    }

    @Override
    public void simulationPeriodic() {
        io.periodic();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Hood angle", getHoodPosition().in(Units.Degrees));
        SmartDashboard.putNumber("Launcher/Launcher Velocity", getLauncherVelocity().in(Units.RotationsPerSecond));
        io.hoodControlLoop();
        // followLaunchCalculations(); uncomment if launch calculations are accurate enough
    }

}