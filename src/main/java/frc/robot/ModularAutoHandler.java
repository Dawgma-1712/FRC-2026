package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandFactories.PathToPose;

public class ModularAutoHandler {

    private static final String NONE = "None";

    // allow driver to choose which actions the auto will take
    // these hold auto names, not PathPlannerAutos, so a fresh command gets built every time auto runs
    public SendableChooser<String> startingSide;
    public SendableChooser<String> action1;
    public SendableChooser<String> action2;
    public SendableChooser<String> action3;


    public ModularAutoHandler() {

        // this all just sends dropdown menus to select autos from to our driver dashboard

        startingSide = new SendableChooser<String>();
        startingSide.setDefaultOption("Left", "Left");
        // startingSide.addOption("Mid", "Mid");
        // startingSide.addOption("Right", "Right");

        SmartDashboard.putData("Starting Side", startingSide);

        action1 = createActionChooser();
        SmartDashboard.putData("Action 1", action1);

        action2 = createActionChooser();
        SmartDashboard.putData("Action 2", action2);

        action3 = createActionChooser();
        SmartDashboard.putData("Action 3", action3);

    }

    private SendableChooser<String> createActionChooser() {
        SendableChooser<String> chooser = new SendableChooser<String>();
        // default to doing nothing so getSelected() never returns null
        chooser.setDefaultOption("None", NONE);
        chooser.addOption("Gather Depot", "Gather Depot");
        chooser.addOption("Gather Human", "Gather Human");
        // chooser.addOption("Gather Neutral", "Gather Neutral");
        chooser.addOption("Shoot Q1", "S1");
        chooser.addOption("Shoot Q2", "S2");
        chooser.addOption("Shoot Q3", "S3");
        chooser.addOption("Shoot Q4", "S4");
        return chooser;
    }

    // called when auto starts (autonomousInit)
    public Command getSelectedModularCommand() {

        System.out.println(String.join("-", startingSide.getSelected(), action1.getSelected(), action2.getSelected(), action3.getSelected()));

        // runs the selected autos one after another
        // everything is built new on each call because a command can only be in one group,
        // reusing the same PathPlannerAuto would throw the second time auto is enabled
        return new SequentialCommandGroup(

            new PathPlannerAuto(startingSide.getSelected()),
            pathToThenRun(action1.getSelected()),
            pathToThenRun(action2.getSelected()),
            pathToThenRun(action3.getSelected())

        );

    }

    // drives to the start of the chosen auto, then runs it
    private Command pathToThenRun(String autoName) {
        if (autoName == null || autoName.equals(NONE)) {
            return Commands.none();
        }

        PathPlannerAuto auto = new PathPlannerAuto(autoName);
        return new SequentialCommandGroup(
            PathToPose.pathToPose(auto.getStartingPose()),
            auto
        );
    }

}
