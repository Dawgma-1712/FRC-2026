package frc.robot.commandFactories;

import java.util.Set;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.Constants.LauncherConstants;
import frc.Constants.RevolverConstants;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Revolver.RevolverSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class AutoPreloads {

    public static Command getAutoPreloads(LauncherSubsystem launcher, CommandSwerveDrivetrain drivetrain, RevolverSubsystem revolver) {

        Command autoPreloads;
        Command prepAutoPreloads;
        Command launchAutoPreloads;

        prepAutoPreloads = Commands.defer(() -> {

        return Commands.run(() -> {
            launcher.launcherLookupTable(drivetrain.getState().Pose);
        });

        }, Set.of(launcher))

            .finallyDo(() -> {
            launcher.setLauncherVelocity(Units.RadiansPerSecond.of(0));
            launcher.setHoodPosition(Units.Degrees.of(0));
            });

        launchAutoPreloads = new SequentialCommandGroup(new WaitCommand(1), Commands.run(() -> {
        revolver.setRevolverPercentOutput(-RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
        launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
        }).finallyDo(
            () -> {
            revolver.setRevolverPercentOutput(-0);
            launcher.setKickerPercentOutput(0);
            }));

        autoPreloads = Commands.parallel(prepAutoPreloads.raceWith(new WaitCommand(LauncherConstants.SHOOT_WAIT_LENGTH)),
            launchAutoPreloads.raceWith(new WaitCommand(LauncherConstants.SHOOT_WAIT_LENGTH)));
        return autoPreloads;
    }


}
