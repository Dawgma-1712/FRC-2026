package frc.robot.subsystems.Launcher;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Constants.FieldConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class LauncherIOSim implements LauncherIO {

    private final StructArrayPublisher<Translation3d> trajectoryPublisher = NetworkTableInstance.getDefault()
                                                                            .getStructArrayTopic("SmartDashboard/ShotTrajectory", Translation3d.struct)
                                                                            .publish();
    private final CommandSwerveDrivetrain drivetrain;

    public LauncherIOSim(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public ChassisSpeeds fieldSpeedsFromRelativeSpeeds(ChassisSpeeds relativeSpeeds, Rotation2d rotation) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, rotation);
    }

    @Override
    public void periodic() {

        SwerveDriveState drivetrainState = drivetrain.getState();
        Pose2d robotPose = drivetrainState.Pose;
        ChassisSpeeds relativeSpeeds = drivetrainState.Speeds;
        Rotation2d rotation = robotPose.getRotation();
        
        ChassisSpeeds fieldSpeeds = fieldSpeedsFromRelativeSpeeds(relativeSpeeds, rotation);
        Translation3d target = FieldConstants.BLUE_HUB_POSE.getTranslation();

        ShotData shot = LaunchCalculations.iterativeMovingShotFromFunnelClearance(robotPose, fieldSpeeds, target, 3);
        List<Translation3d> points = LaunchCalculations.generateTrajectoryPoints(robotPose, shot, 50, fieldSpeeds);

        trajectoryPublisher.set(points.toArray(new Translation3d[0]));

    }

}
