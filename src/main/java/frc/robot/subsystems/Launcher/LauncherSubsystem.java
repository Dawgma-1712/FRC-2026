package frc.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.ShooterConstants;

public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;

    //private final LauncherIOInputsAutoLogged LauncherInputs = new LauncherIOInputsAutoLogged();

    public LauncherSubsystem(LauncherIO io){
        this.io = io;
    }

    public void setShooterVelocity(double velocity){
        io.setFeederVelocity(velocity);
    }

    public double getShooter1Velocity(){
        return io.getShooter1Velocity();
    }

    public double getShooter2Velocity(){
        return io.getShooter2Velocity();
    }

    public void setFeederVelocity(double velocity){
        io.setFeederVelocity(velocity);
    }

    public double getFeederVelocity(){
        return io.getFeederVelocity();
    }

    public boolean hasFuel(){
        return io.hasFuel();
    }


    @Override
    public void periodic() {
        // io.updateInputs(LauncherIOInputs);
    }



    private double calculateVelocityFromDistance(double deltaX) {

        double g = ShooterConstants.GRAVITY;
        double theta = Math.toRadians(ShooterConstants.SHOOTER_ANGLE);
        double deltaY = ShooterConstants.HUB_HEIGHT_METERS;

        // in m/s
        return Math.sqrt(
            (g * Math.pow(deltaX, 2)) / 
            (2 * Math.pow(Math.cos(theta), 2) * (deltaX * Math.tan(theta) - deltaY))
        );

    }
}
