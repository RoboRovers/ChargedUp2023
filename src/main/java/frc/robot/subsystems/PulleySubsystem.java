package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.jni.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.PullyConstants;




public class PulleySubsystem extends SubsystemBase {
    
private CANSparkMax pulleyMotor;
private RelativeEncoder pullyEncoder;

public PulleySubsystem(int pulleyMotorNum) {

//int motor
pulleyMotor = new CANSparkMax(pulleyMotorNum, MotorType.kBrushless);
pulleyMotor.restoreFactoryDefaults();
pulleyMotor.setIdleMode(IdleMode.kBrake);
pulleyMotor.setSmartCurrentLimit(40);


//int encoder
pullyEncoder = pulleyMotor.getEncoder();

}

public void liftIntake() {
    pulleyMotor.set(2);
    pulleyMotor.setInverted(false);
}


public void dropIntake() {
    pulleyMotor.set(2);
    pulleyMotor.setInverted(true);
}









}
