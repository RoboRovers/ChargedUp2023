package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.OI;

public class PneumaticsCommand extends CommandBase {
    public final CommandXboxController opController;
    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public PneumaticsCommand(PneumaticsSubsystem pneumaticsSubsystem, CommandXboxController opController) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.opController = opController;
}

    @Override
    public void execute() {
       
    //toggles
    opController.rightTrigger().onTrue(pneumaticsSubsystem.intakeToggleCommand());
    opController.leftTrigger().onTrue(pneumaticsSubsystem.extensionToggleCommand());
    //up+downs
    opController.povRight().onTrue(pneumaticsSubsystem.intakeOpenCommand());
    opController.povLeft().onTrue(pneumaticsSubsystem.intakeCloseCommand());
    opController.povDown().onTrue(pneumaticsSubsystem.extensionOutCommand());
    opController.povUp().onTrue(pneumaticsSubsystem.extensionRetractCommand());
       
    }


}

