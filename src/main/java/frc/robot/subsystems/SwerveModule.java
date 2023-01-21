package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule extends SubsystemBase {

    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;


    public SwerveModule(int sMotorID, int dMotorID) {
        steerMotor = new CANSparkMax(sMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(dMotorID, MotorType.kBrushless);
        
    }

    public void setSteerSpeed(double speed) {
        steerMotor.set(speed);
        
    }
    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    @Override
    public void periodic() {

    }
    
}
