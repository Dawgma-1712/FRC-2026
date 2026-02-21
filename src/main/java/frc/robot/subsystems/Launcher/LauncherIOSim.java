package frc.robot.subsystems.Launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.LinearSystem;

public class LauncherIOSim implements LauncherIO {

    DCMotor motor = DCMotor.getKrakenX60(1);
    double momentOfInertia = 0.002;

    LinearSystem plant = LinearSystemId.createFlywheelSystem(
        motor,
        momentOfInertia,
        1.0
    );
    FlywheelSim sim = new FlywheelSim(plant, motor);

    double kP = 999999.0;  // Try lowering this later
    double dt = 0.02;

    public LauncherIOSim() {}

    @Override
    public void setKickerVelocity(double shooterRPS) {

        double setpointRadPerSec = shooterRPS * 2 * Math.PI;

        double velocity = sim.getAngularVelocityRadPerSec();
        double error = setpointRadPerSec - velocity;

        double outputVolts = kP * error;

        outputVolts = Math.max(0, Math.min(12, outputVolts));
        sim.setInputVoltage(outputVolts);
        sim.update(dt);

        SmartDashboard.putNumber("Velocity", velocity);
        SmartDashboard.putNumber("Voltage", outputVolts);

    }
    


}
