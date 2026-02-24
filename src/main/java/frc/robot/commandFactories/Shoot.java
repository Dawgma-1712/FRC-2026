package frc.robot.commandFactories;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.Constants.ShooterConstants;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Revolver.*;

public class Shoot {

    public static Command shoot(LauncherSubsystem launcher, RevolverSubsystem revolver, ShotData shot) {

        AngularVelocity desiredLauncherVelocity = Units.RotationsPerSecond.of(shot.exitVelocity());
        AngularVelocity desiredKickerVelocity = Units.RotationsPerSecond.of(shot.exitVelocity() * ShooterConstants.KICKER_SPEED_PROPORTION);

        return new RepeatCommand(Commands.runOnce(() -> {
            launcher.setLauncherVelocity(desiredLauncherVelocity);
            launcher.setKickerVelocity(desiredKickerVelocity);
        })).raceWith(
            new SequentialCommandGroup(
                Commands.waitUntil(() -> launcher.readyToShoot(desiredLauncherVelocity, desiredKickerVelocity)),
                Commands.runOnce(() -> {
                    revolver.setRevolverPercentOutput(0.6);
                })
            )
        );

    }

}
