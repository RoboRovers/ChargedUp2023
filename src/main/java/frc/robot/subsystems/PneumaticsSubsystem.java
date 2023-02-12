package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticsSubsystem extends SubsystemBase {
    


    //Grabber pneumatics
    private static DoubleSolenoid _intakeLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.L_INTAKE_IN, Constants.PneumaticsConstants.L_INTAKE_OUT);
    private static DoubleSolenoid _intakeRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.R_INTAKE_IN, Constants.PneumaticsConstants.R_INTAKE_OUT);
    
    //Extension pneumatics
    private DoubleSolenoid _extension = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.EXTENSION_OUT, Constants.PneumaticsConstants.EXTENSION_IN);

    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
    /** Creates a new Pnuematics. */
    public PneumaticsSubsystem() {
      pcmCompressor.enableDigital();
      //pcmCompressor.disable();
      
      _intakeLeft.set(Value.kForward);
      _intakeRight.set(Value.kForward);
      _extension.set(Value.kForward);
  
    }
  
    
    public boolean intakeState = true;

  
    public void intakeToggle() {
        _intakeLeft.toggle(); 
        _intakeRight.toggle();
       
       if(intakeState = true) {
        intakeState = false;
       }
       else if(intakeState = false) {
        intakeState = true;
       }
    }
  
    public void intakeOpen(){
      _intakeLeft.set(Value.kReverse);
      _intakeRight.set(Value.kReverse);
      intakeState = false;
    }
  
    public void intakeClose(){
      _intakeLeft.set(Value.kForward);
      _intakeRight.set(Value.kForward);
      intakeState = true;
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

    public void extensionToggle(){
      _extension.toggle();
      if(extensionState = true) {
        extensionState = false;
       }
       else if(extensionState = false) {
        extensionState = true;
       }
    }


//all commands for the pneumatic subsystem

public CommandBase intakeOpenCommand() {
  return run(
    () -> {
      intakeOpen();
    }
  );
}

public CommandBase intakeCloseCommand() {
  return run(
    () -> {
      intakeClose();
    }
  );
}   

public CommandBase intakeToggleCommand() {
  return runOnce(
    () -> {
      intakeToggle();
    }
  );
}

public CommandBase extensionRetractCommand() {
  return run(
    () -> {
      extensionRetract();
    }
  );
}

public CommandBase extensionOutCommand() {
  return run(
    () -> {
      extensionOut();
    }
  );
}

public CommandBase extensionToggleCommand() {
  return runOnce(
    () -> {
        extensionToggle();
    }
  );
}

  }
