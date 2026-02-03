package frc.robot.subsystems.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
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
        SmartDashboard.putBoolean("Has Fuel", hasFuel());
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


    private double mpsToRps(double mps) {
        double circumferenceMeters = ShooterConstants.WHEEL_DIAMETER * 0.0254 * Math.PI;
        return mps / circumferenceMeters;
    }

    public Command adaptiveShoot(DoubleSupplier distanceSupplier) {
        return this.run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double requiredMPS = calculateVelocityFromDistance(distance);
            double targetRPS = mpsToRps(requiredMPS);

            io.setShooterVelocity(targetRPS);

            boolean isReady = ((io.getShooter1Velocity()+io.getShooter2Velocity())/2) >= (targetRPS * Constants.ShooterConstants.TARGET_VELOCITY_TOLERANCE) && targetRPS > 0.1;

            if (isReady) {
                io.setFeederVelocity(mpsToRps(ShooterConstants.FEEDER_SPEED_MPS));
            } else {
                io.setFeederVelocity(0);
            }
        })
        .finallyDo((interrupted) -> {
            io.setShooterVelocity(0);
            io.setFeederVelocity(0);
    });
    }

  public Command prepareLauncher(DoubleSupplier distanceSupplier) {
    return this.run(() -> {
        double distance = distanceSupplier.getAsDouble();
        double requiredMPS = calculateVelocityFromDistance(distance);
        double targetRPS = mpsToRps(requiredMPS);
        io.setShooterVelocity(targetRPS);
    })
    .finallyDo((interrupted) -> {
        io.setShooterVelocity(0);
    });
}

    public boolean readyToShoot(DoubleSupplier distanceSupplier){
        double distance = distanceSupplier.getAsDouble();
        double requiredMPS = calculateVelocityFromDistance(distance);
        double targetRPS = mpsToRps(requiredMPS);


        boolean isReady = ((io.getShooter1Velocity()+io.getShooter2Velocity())/2) >= (targetRPS * 0.95) && targetRPS > 0.1;

        return isReady;
    }
}
