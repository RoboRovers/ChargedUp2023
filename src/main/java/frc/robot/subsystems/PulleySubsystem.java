package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;




public class PulleySubsystem extends SubsystemBase {


private CANSparkMax pulleyMotor;
private RelativeEncoder pulleyEncoder;
private SparkMaxPIDController pulleyPidController;
DigitalInput extendLimitSwitch;
DigitalInput retractLimitSwitch;

// public DriveCommand driveCommand;

public boolean retractSwitchState;
public boolean isPulleyDone = false;


public PulleySubsystem(int pulleyMotorNum) {

// this.driveCommand = driveCommand;

//int motor
pulleyMotor = new CANSparkMax(pulleyMotorNum, MotorType.kBrushless);
pulleyMotor.restoreFactoryDefaults();
pulleyMotor.setIdleMode(IdleMode.kBrake);
pulleyMotor.setSmartCurrentLimit(60);
pulleyMotor.setOpenLoopRampRate(0.3);
// retractLimitSwitch = new DigitalInput(Constants.PullyConstants.retractSwitchstatePort);




//int encoder
pulleyEncoder = pulleyMotor.getEncoder();
//pulleyEncoder.setVelocityConversionFactor(1/60);


pulleyPidController = pulleyMotor.getPIDController();
pulleyPidController.setP(Constants.PullyConstants.pulleyPIDControllerPVal);
pulleyPidController.setPositionPIDWrappingEnabled(true);
pulleyPidController.setPositionPIDWrappingMaxInput(10000);
pulleyPidController.setPositionPIDWrappingMinInput(10000);
pulleyEncoder.setPositionConversionFactor(Constants.PullyConstants.pulleyEncoder2deg);

resetEncoder();

}

// this might be necessary?
public void idleBrake() {
    pulleyMotor.set(0);
}


public void resetEncoder() {
    pulleyEncoder.setPosition(0);
}

public void autoHome() {
    pulleyPidController.setReference(0, ControlType.kPosition);
}


public double pulleyEncoderValue() {
    return pulleyEncoder.getPosition();
}



//2' 11" || 35" || 12600 deg || 35 rotations
public void topShelf() {
pulleyPidController.setReference(2700, ControlType.kPosition); 
}


//1' 11" || 23" || 8280 deg || 23 rotations
public void midShelf() {
    pulleyPidController.setReference(1500, ControlType.kPosition);

}

//3' 10" || 46" || 16560 deg || 46 rotations
public void topPole() {
    pulleyPidController.setReference(3100, ControlType.kPosition); //3400
}
public void topPoleDrop() {
    pulleyPidController.setReference(2500, ControlType.kPosition); 
}



//2' 10" || 34" || 12240 deg || 34 rotations
public void midPole() {
    pulleyPidController.setReference(2000, ControlType.kPosition); //2900
}
public void midPoleDown () {
    pulleyPidController.setReference(1500, ControlType.kPosition);
}


//For auto Pickup
public void autoOutUp() {
    pulleyPidController.setReference(1000, ControlType.kPosition);
}

public void autoOutDown() {
    pulleyPidController.setReference(-1000, ControlType.kPosition);
}

public CommandBase StopCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            pulleyMotor.set(0);
            isPulleyDone = true;
        }
    );
}

public CommandBase autoGrabUpCommand() {
    return runOnce(
        () -> {
            autoOutUp();
        }
    );
}

public CommandBase autoGrabDownCommand() {
    return runOnce(
        () -> {
            autoOutDown();
        }
    );
}

public CommandBase homeCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            autoHome();
            System.out.print("Auto Home Command Ran");
            isPulleyDone = true;
        }
    );
}

public CommandBase liftIntakeCommand() {
    return run(
        () -> {
            pulleyMotor.set(5);
        }
    );
}

public CommandBase topPoleDropCommand() {
    return runOnce(
        () -> {
            topPoleDrop();
        }
    );
}

public CommandBase dropIntakeCommand() {
    return run(
        () -> {
            pulleyMotor.set(-5);
        }
    );
}

public CommandBase topPoleCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            topPole();
            System.out.print("Top Shelf Command Ran");
            Commands.waitSeconds(2);
            isPulleyDone = true;

        }
    );
}

public CommandBase midPoleCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            midPole();
            System.out.print("Mid pole Command Ran");
            isPulleyDone = true;
        }
    );
}
public CommandBase midPoleDownCommand() {
    return runOnce(
        () -> {
            midPoleDown();
        }
    );
}

public CommandBase topShelfCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            topShelf();
            System.out.print("Top Shelf Command Ran");
            isPulleyDone = true;
        }
    );
}

public CommandBase midShelfCommand() {
    return runOnce(
        () -> {
            isPulleyDone = false;
            midShelf();
            System.out.print("Mid Shelf Command Ran");
            isPulleyDone = true;
        }
    );
}


@Override
public void periodic() {
    // retractSwitchState = retractLimitSwitch.get();
     SmartDashboard.putNumber("Pulley Encoder Value", pulleyEncoderValue());

     

}



}