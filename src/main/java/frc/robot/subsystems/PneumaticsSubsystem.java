package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class PneumaticsSubsystem extends SubsystemBase {
    

    private static PneumaticsSubsystem instance;

    private DoubleSolenoid _intakeLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.L_INTAKE_OUT, Constants.PneumaticsConstants.L_INTAKE_IN);
    private DoubleSolenoid _intakeRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.R_INTAKE_OUT, Constants.PneumaticsConstants.R_INTAKE_IN);
    
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
    /** Creates a new Pnuematics. */
    private PneumaticsSubsystem() {
      pcmCompressor.enableDigital();
      //pcmCompressor.disable();
      
      _intakeLeft.set(Value.kForward);
      _intakeRight.set(Value.kForward);
  
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
    
  }
