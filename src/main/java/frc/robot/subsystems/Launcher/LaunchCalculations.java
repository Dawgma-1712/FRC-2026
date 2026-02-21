package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose3d;

import frc.Constants.VisionConstants;

public class LaunchCalculations {
    
    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {

        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(Units.RadiansPerSecond), hoodAngle.in(Units.Radians), target);
        }
        
        public ShotData(AngularVelocity exitVelocity, Angle hoodAngle) {
            this(exitVelocity, hoodAngle, VisionConstants.BLUE_HUB_POSE.getTranslation());
        }

    }

}
