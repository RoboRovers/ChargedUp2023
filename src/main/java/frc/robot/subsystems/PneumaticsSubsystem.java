package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticsSubsystem extends SubsystemBase {
    

    private static PneumaticsSubsystem instance;

    //Grabber pneumatics
    private DoubleSolenoid _intakeLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.L_INTAKE_IN, Constants.PneumaticsConstants.L_INTAKE_OUT);
    private DoubleSolenoid _intakeRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.R_INTAKE_IN, Constants.PneumaticsConstants.R_INTAKE_OUT);
    
    //Extension pneumatics
    private DoubleSolenoid _extension = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.EXTENSION_OUT, Constants.PneumaticsConstants.EXTENSION_IN);

    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
    /** Creates a new Pnuematics. */
    private PneumaticsSubsystem() {
      pcmCompressor.enableDigital();
      //pcmCompressor.disable();
      
      _intakeLeft.set(Value.kForward);
      _intakeRight.set(Value.kForward);
      _extension.set(Value.kForward);
  
    }
  
    public static PneumaticsSubsystem getInstance() {
      if (instance == null) {
        instance = new PneumaticsSubsystem();
      }
      return instance;
    }
  
    public void intakeToggle() {
        _intakeLeft.toggle(); 
        _intakeRight.toggle();
    }
  
    public void intakeOpen(){
      _intakeLeft.set(Value.kReverse);
      _intakeRight.set(Value.kReverse);
    }
  
    public void intakeClose(){
      _intakeLeft.set(Value.kForward);
      _intakeRight.set(Value.kForward);
    }
    public void extensionClose(){
      _extension.set(Value.kReverse);
    }
    public void extensionOut(){
       _extension.set(Value.kForward);
    }
    public void extensionToggle(){
      _extension.toggle();
    }
  }
