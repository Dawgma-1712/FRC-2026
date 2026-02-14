package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commandFactories.PathToPose;

public class ModularAutoHandler {
    
    // allow driver to choose which actions the auto will take
    public SendableChooser<PathPlannerAuto> startingSide;
    public SendableChooser<PathPlannerAuto> action1;
    public SendableChooser<PathPlannerAuto> action2;
    public SendableChooser<PathPlannerAuto> action3;


    public ModularAutoHandler() {
        
        // this all just sends dropdown menus to select autos from to our driver dashboard

        startingSide = new SendableChooser<PathPlannerAuto>();
        startingSide.addOption("Left", new PathPlannerAuto("Left"));
        // startingSide.addOption("Mid", new PathPlannerAuto("Mid"));
        // startingSide.addOption("Right", new PathPlannerAuto("Right"));
        SmartDashboard.putData("Starting Side", startingSide);

        action1 = new SendableChooser<PathPlannerAuto>();
        action1.addOption("Gather Depot", new PathPlannerAuto("Gather Depot"));
        action1.addOption("Gather Human", new PathPlannerAuto("Gather Human"));
        // action1.addOption("Gather Neutral", new PathPlannerAuto("Gather Neutral"));
        // action1.addOption("Shoot Q1", new PathPlannerAuto("S1"));
        // action1.addOption("Shoot Q2", new PathPlannerAuto("S2"));
        // action1.addOption("Shoot Q3", new PathPlannerAuto("S3"));
        // action1.addOption("Shoot Q4", new PathPlannerAuto("S4"));
        SmartDashboard.putData("Action 1", action1);

        action2 = new SendableChooser<PathPlannerAuto>();
        action2.addOption("Gather Depot", new PathPlannerAuto("Gather Depot"));
        action2.addOption("Gather Human", new PathPlannerAuto("Gather Human"));
        // action2.addOption("Gather Neutral", new PathPlannerAuto("Gather Neutral"));
        // action2.addOption("Shoot Q1", new PathPlannerAuto("S1"));
        // action2.addOption("Shoot Q2", new PathPlannerAuto("S2"));
        // action2.addOption("Shoot Q3", new PathPlannerAuto("S3"));
        // action2.addOption("Shoot Q4", new PathPlannerAuto("S4"));
        SmartDashboard.putData("Action 2", action2);

        action3 = new SendableChooser<PathPlannerAuto>();
        action3.addOption("Gather Depot", new PathPlannerAuto("Gather Depot"));
        action3.addOption("Gather Human", new PathPlannerAuto("Gather Human"));
        // action3.addOption("Gather Neutral", new PathPlannerAuto("Gather Neutral"));
        // action3.addOption("Shoot Q1", new PathPlannerAuto("S1"));
        // action3.addOption("Shoot Q2", new PathPlannerAuto("S2"));
        // action3.addOption("Shoot Q3", new PathPlannerAuto("S3"));
        // action3.addOption("Shoot Q4", new PathPlannerAuto("S4"));
        SmartDashboard.putData("Action 3", action3);

    }

    public Command getSelectedModularCommand() {

        System.out.println(String.join("-", startingSide.toString(), action1.toString(), action2.toString(), action3.toString()));

        // runs the selected autos one after another
        return new SequentialCommandGroup(

            startingSide.getSelected(), 
            PathToPose.pathToPose(action1.getSelected().getStartingPose()),
            action1.getSelected(), 
            PathToPose.pathToPose(action2.getSelected().getStartingPose()),
            action2.getSelected(), 
            PathToPose.pathToPose(action3.getSelected().getStartingPose()),
            action3.getSelected()

        );

    }

}
