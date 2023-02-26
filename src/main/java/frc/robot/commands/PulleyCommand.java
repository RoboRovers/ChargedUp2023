package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PulleySubsystem;

public class PulleyCommand extends CommandBase {
     
    private final PulleySubsystem pulleySubsystem;
    private final CommandXboxController opController;
    // DigitalInput m_extendlimitSwitch; //assign the limit switch to a GPIO
    // DigitalInput m_retractlimitSwitch; //assign the limit switch to a GPIO

public PulleyCommand(PulleySubsystem pulleySubsystem, CommandXboxController opController) {
        this.pulleySubsystem = pulleySubsystem;
        this.opController = opController;
        addRequirements(pulleySubsystem);

}

@Override
public void initialize() {
    pulleySubsystem.autoHome();
    System.out.print("Auto Home Complete");
}



    @Override
    public void execute() {
    }

}
