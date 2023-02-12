package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsCommand extends CommandBase {
    public final CommandXboxController opController;
    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public PneumaticsCommand(PneumaticsSubsystem pneumaticsSubsystem, CommandXboxController opController) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.opController = opController;
    addRequirements(pneumaticsSubsystem);

}

    @Override
    public void execute() {
       
    //toggles
    opController.rightBumper().onTrue(pneumaticsSubsystem.intakeToggleCommand());
    opController.leftTrigger().onTrue(pneumaticsSubsystem.extensionToggleCommand());
    //up+downs
    opController.povUp().onTrue(pneumaticsSubsystem.extensionOutCommand());
    opController.povDown().onTrue(pneumaticsSubsystem.extensionRetractCommand());
    }


}

