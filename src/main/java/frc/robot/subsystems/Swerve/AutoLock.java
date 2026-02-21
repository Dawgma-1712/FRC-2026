package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoLock extends Command {

    private final CommandSwerveDrivetrain drivetrain; // Removed static
    private final PIDController rotationController;
    private final Translation2d autoLockTarget;
    
    // Joystick Suppliers
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;

    // Switched to FieldCentric so driving feels natural while spinning
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public AutoLock(CommandSwerveDrivetrain drivetrain, Translation2d autoLockTarget, Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
        this.drivetrain = drivetrain;
        this.autoLockTarget = autoLockTarget;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        this.rotationController = new PIDController(0.27, 0, 0.003);
        
        this.rotationController.enableContinuousInput(-180, 180); 
        
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double goalAngle = findAutoLockAngle(autoLockTarget);

        double rotateSpeed = rotationController.calculate(drivetrain.getState().Pose.getRotation().getDegrees(), goalAngle);

        drivetrain.setControl(driveRequest
            .withVelocityX(xSupplier.get())
            .withVelocityY(ySupplier.get())
            .withRotationalRate(rotateSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.setControl(new SwerveRequest.Idle());
    }

    private double findAutoLockAngle(Translation2d lockTarget) {
        Pose2d robotPose = drivetrain.getState().Pose;

        double distanceX = lockTarget.getX() - robotPose.getX();
        double distanceY = lockTarget.getY() - robotPose.getY();

        double angleRad = Math.atan2(distanceY, distanceX);
        return Units.radiansToDegrees(angleRad);
    }
}