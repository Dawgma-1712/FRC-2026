package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.FieldConstants;
import frc.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;



public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
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
        io.setHoodAngle(angle);
    }

    public Angle getHoodPosition() {
        return io.getHoodPosition();
    }
    public void stop() {
        setLauncherVelocity(Units.RotationsPerSecond.of(0));
        setKickerPercentOutput(0);
    }

    public ChassisSpeeds fieldSpeedsFromRelativeSpeeds(ChassisSpeeds relativeSpeeds) {
        
        Rotation2d rotation = drivetrain.getState().Pose.getRotation();
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, rotation);

    }

    public ShotData getShotData() {
        
        return LaunchCalculations.iterativeMovingShotFromFunnelClearance(
            drivetrain.getState().Pose, 
            fieldSpeedsFromRelativeSpeeds(drivetrain.getState().Speeds),  // make sure this is field relative
            FieldConstants.BLUE_HUB_POSE.getTranslation(),
             1
        );

    }

    public boolean readyToShoot(AngularVelocity desiredLauncherVelocity) {

        AngularVelocity launcherVelocity = getLauncherVelocity();

        AngularVelocity tolerance = LauncherConstants.TARGET_VELOCITY_TOLERANCE;

        boolean isLauncherReady = Math.abs(launcherVelocity.minus(desiredLauncherVelocity).magnitude()) < tolerance.magnitude();

        return isLauncherReady;

    }

    @Override
    public void simulationPeriodic() {
        io.periodic();
    }

    @Override
    public void periodic() {
        io.hoodControlLoop();
    }

}