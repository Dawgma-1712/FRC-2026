package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.FieldConstants;
import frc.Constants.ShooterConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;


public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;

    //private final LauncherIOInputsAutoLogged LauncherInputs = new LauncherIOInputsAutoLogged();

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain){
        this.io = io;
        this.drivetrain = drivetrain;
    }

    public void setKickerVelocity(double velocity){
        io.setKickerVelocity(velocity);
    }

    public double getKickerVelocity(){
        return io.getKickerVelocity();
    }

    public void setFeederVelocity(double velocity){
        io.setFeederVelocity(velocity);
    }

    public double getFeederVelocity(){
        return io.getFeederVelocity();
    }

    public boolean hasFuelIntaked(){
        return io.hasFuelIntaked();
    }

    public void setHoodPosition(double angle){
        io.setHoodPosition(angle);
    }

    public double getHoodPosition(){
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
             3
        );

    }

    @Override
    public void periodic() {
        io.periodic();
    }

    
}