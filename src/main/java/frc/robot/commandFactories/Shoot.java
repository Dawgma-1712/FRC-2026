package frc.robot.commandFactories;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.Constants.ShooterConstants;
import frc.Constants.RevolverConstants;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Revolver.*;

public class Shoot {

    public static Command shoot(LauncherSubsystem launcher, RevolverSubsystem revolver, Supplier<ShotData> shotSupplier) {

        return Commands.defer(() -> {
            ShotData shot = shotSupplier.get();

            AngularVelocity desiredLauncherVelocity = Units.RotationsPerSecond.of(shot.exitVelocity());
            AngularVelocity desiredKickerVelocity = Units.RotationsPerSecond.of(shot.exitVelocity() * ShooterConstants.KICKER_SPEED_PROPORTION);

            return Commands.sequence(
                Commands.waitUntil(() -> launcher.readyToShoot(desiredLauncherVelocity, desiredKickerVelocity)),
                Commands.runOnce(() -> revolver.setRevolverPercentOutput(RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT)),
                Commands.waitSeconds(0.5)
            ).deadlineFor(
                Commands.run(() -> {
                    launcher.setLauncherVelocity(desiredLauncherVelocity);
                    launcher.setKickerVelocity(desiredKickerVelocity);
                }, launcher)
            ).finallyDo(() -> {
                launcher.stop();
                revolver.stop();
            });
        }, Set.of(launcher, revolver));


    }

}
