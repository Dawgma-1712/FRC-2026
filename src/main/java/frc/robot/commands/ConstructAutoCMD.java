package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConstructAutoCMD extends Command {

    // allow driver to choose which actions the auto will take
    public SendableChooser<String> startingSide = new SendableChooser<String>();
    public SendableChooser<String> action1 = new SendableChooser<String>();
    public SendableChooser<String> action2 = new SendableChooser<String>();
    public SendableChooser<String> action3 = new SendableChooser<String>();

    public ConstructAutoCMD() {

        startingSide.addOption("Left", "L-");
        startingSide.addOption("Mid", "M-");
        startingSide.addOption("Right", "R-");
        SmartDashboard.putData("Starting Side", startingSide);

        action1.addOption("Gather Depot", "Gd-");
        action1.addOption("Gather Human", "Gh-");
        action1.addOption("Gather Neutral", "Gn-");
        action1.addOption("Shoot Q1", "S1-");
        action1.addOption("Shoot Q2", "S2-");
        action1.addOption("Shoot Q3", "S3-");
        action1.addOption("Shoot Q4", "S4-");
        SmartDashboard.putData("Action 1", action1);

        action2.addOption("Gather Depot", "Gd-");
        action2.addOption("Gather Human", "Gh-");
        action2.addOption("Gather Neutral", "Gn-");
        action2.addOption("Shoot Q1", "S1-");
        action2.addOption("Shoot Q2", "S2-");
        action2.addOption("Shoot Q3", "S3-");
        action2.addOption("Shoot Q4", "S4-");
        SmartDashboard.putData("Action 2", action2);

        action3.addOption("Gather Depot", "Gd");
        action3.addOption("Gather Human", "Gh");
        action3.addOption("Gather Neutral", "Gn");
        action3.addOption("Shoot Q1", "S1");
        action3.addOption("Shoot Q2", "S2");
        action3.addOption("Shoot Q3", "S3");
        action3.addOption("Shoot Q4", "S4");
        SmartDashboard.putData("Action 3", action3);
        
    }
    
}
