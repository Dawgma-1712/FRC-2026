package frc.robot.subsystems.Launcher;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;

public interface LauncherIO {


    public default void setLauncherVelocity(AngularVelocity shooterRPS) {}

    public default AngularVelocity getLauncherVelocity() {
        return Units.RadiansPerSecond.of(0);
    }
    
    public default void setKickerPercentOutput(double PercentOutput) {}

    public default AngularVelocity getKickerVelocity() {
        return Units.RadiansPerSecond.of(0);
    }

    public default void hoodControlLoop() {}

    public default void setHoodAngle(Angle goalAngle) {}

    public default Angle getHoodPosition() {
        return Units.Degrees.of(0);
    }

    public default boolean hasFuelIntaked() {
        return false;
    }

    public default boolean getFeedSensor() {
        return false;
    }

    public default boolean hasShotFuel() {
        return false;
    }

    public default void setLauncherPercentOutput(double goalOutput) {

    }

    public default void intakeFuel() {}
    
    public default void periodic() {}
}