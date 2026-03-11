package frc.robot.commandFactories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.Constants.LauncherConstants;
import frc.Constants.RevolverConstants;
import frc.robot.subsystems.Launcher.*;
import frc.robot.subsystems.Revolver.*;

public class Shoot {

    public static Command shoot(LauncherSubsystem launcher, RevolverSubsystem revolver, Supplier<ShotData> shotSupplier) {

        return Commands.run(() -> {
            
            ShotData shot = shotSupplier.get();
            launcher.setKickerPercentOutput(LauncherConstants.KICKER_PERCENT_OUTPUT);
            launcher.setHoodPosition(shot.getHoodAngle());

            Commands.waitSeconds(0.1);

            revolver.setRevolverPercentOutput(RevolverConstants.SHOOTING_PERCENTAGE_OUTPUT);
            
        }, launcher, revolver).finallyDo(() -> {
            
            launcher.setKickerPercentOutput(0);
            revolver.setRevolverPercentOutput(0);

        });
        
    };

}
