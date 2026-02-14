package frc.robot.commandFactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.Constants.DriveConstants;

public class PathToPose {
    
    public static Command pathToPose(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, DriveConstants.GAME_CONSTRAINTS, 0);

    }

}
