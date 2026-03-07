package frc.robot.subsystems.Launcher;


import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import frc.Constants.FieldConstants;
import frc.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher.LaunchCalculations.ShotData;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.utils.FuelSim;

public class LauncherIOSim implements LauncherIO {

    private final CommandSwerveDrivetrain drivetrain;
    private final FuelSim fuelSim;
    private final int CAPACITY = 32;
    private int fuelStored = 0;
    private int update = 0;
    StructPublisher<Pose3d> launchPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Launcher/LaunchPose", Pose3d.struct).publish();

    public LauncherIOSim(CommandSwerveDrivetrain drivetrain, FuelSim fuelSim) {
        this.drivetrain = drivetrain;
        this.fuelSim = fuelSim;
    }

    public ChassisSpeeds fieldSpeedsFromRelativeSpeeds(ChassisSpeeds relativeSpeeds, Rotation2d rotation) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, rotation);
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    @Override
    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel(ShotData shot) {
        update++;
        if (update != 50) return;
        update = 0;
        fuelSim.launchFuel(
                shot.getExitVelocity(),
                shot.getHoodAngle(),
                Units.Degrees.of(0),
                Units.Meters.of(LauncherConstants.LAUNCHER_HEIGHT_METERS)
        );
    }

    @Override
    public void periodic() {

        
    }
}
