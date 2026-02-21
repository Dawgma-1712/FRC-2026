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

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;

import java.util.function.Supplier;

public class AutoLockAndShoot {
    
    public Command autoLockAndShoot(CommandSwerveDrivetrain drivetrain, 
                                    LauncherSubsystem launcher, 
                                    Supplier<Double> xSupplier, 
                                    Supplier<Double> ySupplier) {

        ShotData shotData = launcher.getShotData();
        Translation3d goalTarget = shotData.getTarget();

        SequentialCommandGroup CMDGroup = new SequentialCommandGroup(

            new AutoLock(drivetrain, goalTarget.toTranslation2d(), xSupplier, ySupplier),

            Commands.runOnce(() -> {
                launcher.setKickerVelocity(shotData.exitVelocity() / (Math.PI * 2));  // exit velocity is in radians per second, and setKickerVelocity takes rotations per second
            })
        );
                                        
        return CMDGroup;
    }
    

}
