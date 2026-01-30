package frc.robot.subsystems;
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.Constants.OperatorConstants;;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Constants.IdConstants.INTAKE_MOTOR_ID, MotorType.kBrushed);
    
    public Intaker() {}

    public void stop() {
        intakeMotor.set(0);
    }

    public void setMotor(double motorSpeed) {
        intakeMotor.set(motorSpeed);
    }

}