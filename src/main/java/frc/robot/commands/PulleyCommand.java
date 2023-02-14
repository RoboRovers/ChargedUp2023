package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PulleySubsystem;

public class PulleyCommand extends CommandBase {
     
    private final PulleySubsystem pulleySubsystem;
    private final CommandXboxController opController;
    DigitalInput m_extendlimitSwitch; //assign the limit switch to a GPIO
    DigitalInput m_retractlimitSwitch; //assign the limit switch to a GPIO

public PulleyCommand(PulleySubsystem pulleySubsystem, CommandXboxController opController) {
        this.pulleySubsystem = pulleySubsystem;
        this.opController = opController;
        addRequirements(pulleySubsystem);

}

@Override
public void initialize() {
    //pulleySubsystem.autoHome();
}



    @Override
    public void execute() {

       // boolean extendSwitchstate = m_extendlimitSwitch.get();
       // boolean retractSwitchstate = m_retractlimitSwitch.get();

       // m_extendlimitSwitch = new DigitalInput(Constants.PullyConstants.extendSwitchstatePort); //use DIO 1
       // m_retractlimitSwitch = new DigitalInput(Constants.PullyConstants.retractSwitchstatePort); //use DIO 2
      //  SmartDashboard.putBoolean("Extension Switch State", extendSwitchstate);
       // SmartDashboard.putBoolean("Retraction Switch State", retractSwitchstate);

//if the extend switch is pressed then you can only retract
       // if(extendSwitchstate = true) {
            //pulleySubsystem.stopMotor();
           // System.out.print("Fully Extended");
           //opController.povRight().whileTrue(pulleySubsystem.dropIntakeCommand());
       // }

//if the retract switch is pressed then you can only extend
        //if(retractSwitchstate = true) {
            //pulleySubsystem.stopMotor();
            //System.out.print("Fully Retracted");
          // opController.povLeft().whileTrue(pulleySubsystem.liftIntakeCommand());
        //}

//if both switches are not being pressed then you can go either way
        //if((retractSwitchstate = false) && (extendSwitchstate = false)) {
            opController.povLeft().whileTrue(pulleySubsystem.liftIntakeCommand());
            opController.povLeft().whileFalse(pulleySubsystem.StopCommand());
            opController.povRight().whileTrue(pulleySubsystem.dropIntakeCommand());
            opController.povRight().whileFalse(pulleySubsystem.StopCommand());

        //}   
    
    }

}
