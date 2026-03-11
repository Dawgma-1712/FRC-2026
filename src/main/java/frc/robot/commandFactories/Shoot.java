package frc.robot.commandFactories;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.Constants.LauncherConstants;
import frc.Constants.RevolverConstants;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Revolver.*;

public class Shoot {

    public static Command shoot(LauncherSubsystem launcher, RevolverSubsystem revolver, Supplier<ShotData> shotSupplier) {

        return Commands.defer(() -> {
        ShotData shot = shotSupplier.get();
        AngularVelocity desiredLauncherVelocity = Units.RotationsPerSecond.of(shot.exitVelocity());

        return Commands.sequence(
            Commands.waitUntil(() -> launcher.readyToShoot(desiredLauncherVelocity)),
            Commands.runOnce(() -> revolver.setRevolverPercentOutput(
                RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT)),
            Commands.waitSeconds(0.5)
        ).deadlineFor(
            Commands.run(() -> {
                launcher.setLauncherVelocity(desiredLauncherVelocity);
                launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
                launcher.setHoodPosition(shot.getHoodAngle());
            }, launcher)
        ).finallyDo(() -> {
            revolver.stop();
            launcher.stop();
        });
    }, Set.of(launcher, revolver));


    }

}
