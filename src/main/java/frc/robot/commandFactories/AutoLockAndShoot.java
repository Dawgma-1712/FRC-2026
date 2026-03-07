package frc.robot.commandFactories;

import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Revolver.RevolverSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.AutoLock;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.Constants.RevolverConstants;

import java.util.function.Supplier;

public class AutoLockAndShoot {

    private static StructPublisher<Pose3d> targetPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("TargetPose", Pose3d.struct).publish();

    public static Command autoLockAndShoot(CommandSwerveDrivetrain drivetrain,
                                    LauncherSubsystem launcher,
                                    RevolverSubsystem revolver,
                                    Supplier<Double> xSupplier,
                                    Supplier<Double> ySupplier) {

        Supplier<ShotData> shotSupplier = () -> launcher.getShotData();

        Command autoAim = new AutoLock(
            drivetrain,
            () -> shotSupplier.get().getTarget().toTranslation2d(),
            xSupplier,
            ySupplier
        );

        Command spinUpAndShoot = Commands.sequence(
            Commands.waitUntil(() -> { 
                ShotData shot = launcher.getShotData();
                return launcher.readyToShoot(Units.RotationsPerSecond.of(shot.exitVelocity()))
                    && launcher.hoodAtPosition();
            }),
            Commands.runOnce(() -> {
                revolver.setRevolverPercentOutput(
                    RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT
                );
            }, revolver),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> revolver.stop(), revolver)
        ).deadlineFor(
            Commands.run(() -> {
                ShotData shot = shotSupplier.get();
                AngularVelocity desiredVelocity = Units.RotationsPerSecond.of(shot.exitVelocity());
                launcher.setLauncherVelocity(desiredVelocity);
                launcher.setHoodPosition(shot.getHoodAngle());
                targetPosePublisher.set(new Pose3d(shot.getTarget(), new Rotation3d()));
            }, launcher)
        ).finallyDo(() -> {
            launcher.stop();
            revolver.stop();
        });


        return spinUpAndShoot.deadlineFor(autoAim);
    }
}