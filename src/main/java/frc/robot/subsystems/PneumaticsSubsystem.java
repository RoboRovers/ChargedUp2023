package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticsSubsystem extends SubsystemBase {
    

    //Grabber pneumatics
    private static DoubleSolenoid _intakeLeft = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.L_INTAKE_IN, Constants.PneumaticsConstants.L_INTAKE_OUT);
    //Extension pneumatics
    private static DoubleSolenoid _extension = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.EXTENSION_IN, Constants.PneumaticsConstants.EXTENSION_OUT);
    //FLipper pneumatics
    private static DoubleSolenoid _flipper = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.FLIPPER_IN, Constants.PneumaticsConstants.FLIPPER_OUT);
    /** Creates a new Pnuematics. */
    public PneumaticsSubsystem() {
      _intakeLeft.set(Value.kForward);
      _extension.set(Value.kReverse);
      _flipper.set(Value.kReverse);
    }
  
    
    public boolean intakeState = true;

  /* 
    public void intakeToggle() {
        _intakeLeft.toggle(); 
       
       if(intakeState = true) {
        intakeState = false;
       }
       else if(intakeState = false) {
        intakeState = true;
       }       
    } */
  
    public void intakeOpen(){
      _intakeLeft.set(Value.kReverse);
      intakeState = false;
    }
  
    public void intakeClose(){
      _intakeLeft.set(Value.kForward);
      intakeState = true;
    }

    public void flipperClose() {
      _flipper.set(Value.kReverse);
    }
    public void flipperExtend() {
      _flipper.set(Value.kForward);
    }

    public boolean extensionState = false;

    public void extensionRetract(){
      _extension.set(Value.kReverse);
      extensionState = false;
    }

    public void extensionOut(){
       _extension.set(Value.kForward);
       extensionState = true;
    }

   /*  
    public void extensionToggle(){
      _extension.toggle();
      if(extensionState = true) {
        extensionState = false;
       }
       else if(extensionState = false) {
        extensionState = true;
       }
    }
*/

//all commands for the pneumatic subsystem

public CommandBase intakeOpenCommand() {
  return runOnce(
    () -> {
      intakeOpen();
      System.out.print("Intake Open Command Ran");
    }
  );
}

public CommandBase intakeCloseCommand() {
  return runOnce(
    () -> {
      intakeClose();
      System.out.print("Intake Close Command Ran");
    }
  );
}   

public CommandBase flipperCloseCommand() {
  return runOnce(
    () -> {
      flipperClose();
    }
  );
}

public CommandBase flipperExtendCommand() {
  return runOnce(
    () -> {
      flipperExtend();
    }
  );
}

public CommandBase extensionRetractCommand() {
  return runOnce(
    () -> {
      extensionRetract();
      System.out.print("Extension Retract Command Ran");
    }
  );
}

public CommandBase extensionOutCommand() {
  return runOnce(
    () -> {
      extensionOut();
      System.out.print("Extension Out Command Ran");
    }
  );
}
/* 
public CommandBase extensionToggleCommand() {
  return runOnce(
    () -> {
        extensionToggle();
        System.out.print("Extension Toggle Command Ran");
    }
  );
  
}
*/
  }
