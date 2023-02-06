package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.PullyConstants;




public class PulleySubsystem extends SubsystemBase {

    private static PulleySubsystem instance;


private CANSparkMax pulleyMotor;
private RelativeEncoder pullyEncoder;

public static PulleySubsystem getInstance() {
    if (instance == null) {
      instance = new PulleySubsystem(Constants.PullyConstants.pulleyMotorNum);
    }
    return instance;
  }

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
}


public void dropIntake() {
    pulleyMotor.set(-2);
}

public void stopMotor() {
    pulleyMotor.set(0);
}








}
