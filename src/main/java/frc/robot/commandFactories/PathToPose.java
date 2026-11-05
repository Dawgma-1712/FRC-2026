package frc.robot.commandFactories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.Constants.DriveConstants;

public class PathToPose {

    // targetPose is relative to the blue alliance origin (like PathPlannerAuto.getStartingPose()),
    // it gets flipped to the red side when we're on red
    public static Command pathToPose(Pose2d targetPose) {
        // getStartingPose() returns null for autos that have no paths
        if (targetPose == null) {
            DriverStation.reportWarning("pathToPose got a null target pose, skipping pathfind", false);
            return Commands.none();
        }

        return AutoBuilder.pathfindToPoseFlipped(targetPose, DriveConstants.GAME_CONSTRAINTS, 0);
    }

}
