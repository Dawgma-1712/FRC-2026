package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.FieldConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;


public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;
    private double launcherTargetRps = 0.0;
    private double kickerTargetRps = 0.0;


    //private final LauncherIOInputsAutoLogged LauncherInputs = new LauncherIOInputsAutoLogged();

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

    public void setKickerVelocity(AngularVelocity velocity) {
        io.setKickerVelocity(velocity);
    }

    public AngularVelocity getKickerVelocity() {
        return io.getKickerVelocity();
    }

    public boolean hasFuelIntaked() {
        return io.hasFuelIntaked();
    }

    public void setHoodPosition(Angle angle) {
        io.setHoodPosition(angle);
    }

    public Angle getHoodPosition() {
        return io.getHoodPosition();
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

    public void shoot(ShotData shot) {

    }

    @Override
    public void periodic() {
        io.periodic();
    }

    
}