package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PulleySubsystem;

public class PulleyCommand extends CommandBase {
     
    private final PulleySubsystem pulleySubsystem;
    // DigitalInput m_extendlimitSwitch; //assign the limit switch to a GPIO
    // DigitalInput m_retractlimitSwitch; //assign the limit switch to a GPIO

public PulleyCommand(PulleySubsystem pulleySubsystem) {
        this.pulleySubsystem = pulleySubsystem;
        addRequirements(pulleySubsystem);

}

@Override
public void initialize() {
   // pulleySubsystem.autoHome();
    //System.out.print("Auto Home Complete");
}



    @Override
    public void execute() {
    }



    
    @Override
public boolean isFinished() {

    return false;
} 
}
