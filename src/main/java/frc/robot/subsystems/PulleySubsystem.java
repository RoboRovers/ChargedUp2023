package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.PullyConstants;




public class PulleySubsystem extends SubsystemBase {

    private static PulleySubsystem instance;


private CANSparkMax pulleyMotor;
private RelativeEncoder pulleyEncoder;
private SparkMaxPIDController pulleyPidController;
DigitalInput extendLimitSwitch;
DigitalInput retractLimitSwitch;


 
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
pulleyEncoder = pulleyMotor.getEncoder();
pulleyEncoder.setVelocityConversionFactor(1/60);


pulleyPidController = pulleyMotor.getPIDController();
pulleyPidController.setP(Constants.PullyConstants.pulleyPIDControllerPVal);
pulleyPidController.setPositionPIDWrappingEnabled(true);
pulleyPidController.setPositionPIDWrappingMinInput(720);
pulleyEncoder.setPositionConversionFactor(Constants.PullyConstants.pulleyEncoder2deg);

}

// this might be necessary?
public void idleBrake() {
    pulleyMotor.set(0);
}

public void autoHome() {
    retractLimitSwitch = new DigitalInput(Constants.PullyConstants.retractSwitchstatePort);
    boolean retractSwitchstate = retractLimitSwitch.get();

    while(retractSwitchstate != true) {
    pulleyMotor.set(-2);
    }
    pulleyMotor.set(0);
    pulleyEncoder.setPosition(0);
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

//2' 11" || 35" || 12600 deg || 35 rotations
public void topShelf() {
pulleyPidController.setReference(12600, ControlType.kPosition);
}

//1' 11" || 23" || 8280 deg || 23 rotations
public void midShelf() {
    pulleyPidController.setReference(8280, ControlType.kPosition);

}

//3' 10" || 46" || 16560 deg || 46 rotations
public void topPole() {
    pulleyPidController.setReference(16560, ControlType.kPosition);

}


//2' 10" || 34" || 12240 deg || 34 rotations
public void midPole() {
    pulleyPidController.setReference(12240, ControlType.kPosition);
}


public CommandBase homeCommand() {
    return runOnce(
        () -> {
            homeCommand();
        }
    );
}

public CommandBase liftIntakeCommand() {
    return run(
        () -> {
            liftIntake();
        }
    );
}

public CommandBase dropIntakeCommand() {
    return run(
        () -> {
            dropIntake();
        }
    );
}

public CommandBase topPoleCommand() {
    return runOnce(
        () -> {
            topPole();
        }
    );
}

public CommandBase midPoleCommand() {
    return runOnce(
        () -> {
            midPole();
        }
    );
}

public CommandBase topShelfCommand() {
    return runOnce(
        () -> {
            topShelf();
        }
    );
}

public CommandBase midShelfCommand() {
    return runOnce(
        () -> {
            midShelf();
        }
    );
}



}