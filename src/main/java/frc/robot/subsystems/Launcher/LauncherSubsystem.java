package frc.robot.subsystems.Launcher;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.Constants.VisionConstants;

public class LauncherSubsystem extends SubsystemBase {

    LauncherIO io;
    CommandSwerveDrivetrain drivetrain;

    //private final LauncherIOInputsAutoLogged LauncherInputs = new LauncherIOInputsAutoLogged();

    public LauncherSubsystem(LauncherIO io, CommandSwerveDrivetrain drivetrain){
        this.io = io;
        this.drivetrain = drivetrain;
    }

    public void setKickerVelocity(double velocity){
        io.setKickerVelocity(velocity);
    }

    public double getKickerVelocity(){
        return io.getKickerVelocity();
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

    public void setHoodPosition(double angle){
        io.setHoodPosition(angle);
    }

    public double getHoodPosition(){
        return io.getHoodPosition();
    }


    @Override
    public void periodic() {
        // io.updateInputs(LauncherIOInputs);
        SmartDashboard.putBoolean("Has Fuel", hasFuel());
        SmartDashboard.putNumber("Current Hood Angle", getHoodPosition());
        SmartDashboard.putNumber("Current Launcher RPS", getKickerVelocity());
    }

    @Override
    public void simulationPeriodic() {

        Pose3d hubPose = VisionConstants.BLUE_HUB_POSE;
        Pose3d robotPose = new Pose3d(drivetrain.getState().Pose);

        // chained functions of doom and despair
        double distance = hubPose.toPose2d().getTranslation().getDistance(robotPose.toPose2d().getTranslation());
        double launchAngle = calculateIdealLaunchAngle(distance);
        double hoodPos = calculateHoodPosFromDistance(distance);
        double velocity = calculateVelocityFromDistance(distance, hoodPos);
        

    }

    private double calculateIdealLaunchAngle(double distance) {
        double a = 0.22;
        double b = -2.85;
        double c = 5.5;
        double d = 78.5;

        double idealTheta = (a * Math.pow(distance, 3)) 
                        + (b * Math.pow(distance, 2)) 
                        + (c * distance) 
                        + d;
        return Math.max(41.0, Math.min(90-ShooterConstants.BASE_ANGLE, idealTheta));
    }

    private double calculateHoodPosFromDistance(double distance) {
        double targetTheta = calculateIdealLaunchAngle(distance);
        
        double hoodOffset = 90-ShooterConstants.BASE_ANGLE - targetTheta;

        return Math.max(0, Math.min(ShooterConstants.MAX_HOOD_OFFSET, hoodOffset));
    }

    private double calculateVelocityFromDistance(double deltaX, double hoodPos) {
        double g = ShooterConstants.GRAVITY;
        
        double thetaDegrees = 90.0 - ShooterConstants.BASE_ANGLE - hoodPos; //how we get the projectile angle 90-base-hood pos add on
        double thetaRadians = Math.toRadians(thetaDegrees);
        
        double deltaY = ShooterConstants.HUB_HEIGHT_METERS-ShooterConstants.LAUNCHER_HEIGHT_METERS;

        double cosTheta = Math.cos(thetaRadians);
        double tanTheta = Math.tan(thetaRadians);
        double denominator = 2 * Math.pow(cosTheta, 2) * (deltaX * tanTheta - deltaY);

        if (denominator <= 0) return 0; // Target is physically unreachable at this angle

        return Math.sqrt((g * Math.pow(deltaX, 2)) / denominator);
    }

    private double mpsToRps(double mps) {
        double circumferenceMeters = ShooterConstants.WHEEL_DIAMETER * 0.0254 * Math.PI;
        return mps / circumferenceMeters;
    }

    public Command adaptiveShoot(DoubleSupplier distanceSupplier) {
        return this.run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double targetHood = calculateHoodPosFromDistance(distance);
            double requiredMPS = calculateVelocityFromDistance(distance, targetHood);
            double targetRPS = mpsToRps(requiredMPS);

            io.setHoodPosition(targetHood);
            io.setKickerVelocity(targetRPS);

            if (readyToShoot(distanceSupplier)) {
                io.setFeederVelocity(mpsToRps(ShooterConstants.FEEDER_SPEED_MPS));
            } else {
                io.setFeederVelocity(0);
            }
        })
        .finallyDo((interrupted) -> {
            io.setKickerVelocity(0);
            io.setFeederVelocity(0);
        });
    }

    public Command prepareLauncher(DoubleSupplier distanceSupplier) {
        return this.run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double targetHood = calculateHoodPosFromDistance(distance);
            double targetRPS = mpsToRps(calculateVelocityFromDistance(distance, targetHood));
            
            io.setHoodPosition(targetHood);
            io.setKickerVelocity(targetRPS);
        })
        .finallyDo((interrupted) -> {
            io.setKickerVelocity(0);
        });
    }

    public boolean readyToShoot(DoubleSupplier distanceSupplier){
        double distance = distanceSupplier.getAsDouble();
        double targetHood = calculateHoodPosFromDistance(distance);
        double targetRPS = mpsToRps(calculateVelocityFromDistance(distance, targetHood));

        boolean velocityReady = Math.abs(io.getKickerVelocity() - targetRPS) <= (targetRPS * ShooterConstants.TARGET_VELOCITY_TOLERANCE);
        boolean hoodReady = Math.abs(io.getHoodPosition() - targetHood) <= ShooterConstants.TARGET_HOOD_TOLERANCE_DEGREES; 

        return velocityReady && hoodReady && targetRPS > 0.1;
    }
}
