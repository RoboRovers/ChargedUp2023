package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PulleySubsystem;
import frc.OI;

public class PulleyCommand extends CommandBase {
    
    private final PulleySubsystem pulleySubsystem;
    private final OI opController;
    DigitalInput m_extendlimitSwitch; //assign the limit switch to a GPIO
    DigitalInput m_retractlimitSwitch; //assign the limit switch to a GPIO

public PulleyCommand(PulleySubsystem pulleySubsystem, OI opController) {
        this.pulleySubsystem = pulleySubsystem;
        this.opController = opController;
}



    @Override
    public void execute() {

        boolean extendSwitchstate = m_extendlimitSwitch.get();
        boolean retractSwitchstate = m_retractlimitSwitch.get();

        m_extendlimitSwitch = new DigitalInput(1); //use DIO 1
        m_retractlimitSwitch = new DigitalInput(2); //use DIO 2
SmartDashboard.putBoolean("Extension Switch State", extendSwitchstate);
SmartDashboard.putBoolean("Retraction Switch State", retractSwitchstate);

//if the extend switch is pressed then you can only retract
if(extendSwitchstate = true) {
    pulleySubsystem.stopMotor();
    System.out.print("Fully Extended");
    if(opController.buttonX.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }
}

//if the retract switch is pressed then you can only extend
if(retractSwitchstate = true) {
   pulleySubsystem.stopMotor();
   System.out.print("Fully Retracted");
    if(opController.buttonY.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }
}

//if both switches are not being pressed then you can go either way
if((retractSwitchstate = false) && (extendSwitchstate = false)) {
    if(opController.buttonY.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }

    if(opController.buttonX.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }
}

  


    }

}
