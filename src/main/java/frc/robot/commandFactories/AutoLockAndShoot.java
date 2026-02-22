package frc.robot.commandFactories;

import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.AutoLock;

import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher.LaunchCalculations;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;

import java.util.function.Supplier;

public class AutoLockAndShoot {
    
    private static StructPublisher<Pose3d> targetPosePublisher = NetworkTableInstance.getDefault().getStructTopic("TargetPose", Pose3d.struct).publish();

    public static Command autoLockAndShoot(CommandSwerveDrivetrain drivetrain, 
                                    LauncherSubsystem launcher, 
                                    Supplier<Double> xSupplier, 
                                    Supplier<Double> ySupplier) {

                                        
        Supplier<ShotData> shotSupplier = () -> launcher.getShotData();
        SequentialCommandGroup CMDGroup = new SequentialCommandGroup(
            new AutoLock(drivetrain, () -> shotSupplier.get().getTarget().toTranslation2d(), xSupplier, ySupplier)
        );

        return CMDGroup;
    }

}
