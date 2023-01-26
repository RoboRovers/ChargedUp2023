package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.OI;
import frc.robot.subsystems.PneumaticsSubsystem;

public class ArmCommand extends CommandBase {
    
    public final OI opController;
    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public ArmCommand(PneumaticsSubsystem pneumaticsSubsystem, OI opController) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.opController = opController;
}

    @Override
    public void execute() {

        if(opController.rightTrigger.getAsBoolean()) {
            PneumaticsSubsystem.getInstance().intakeOpen();;
            }
        
        if(opController.leftTrigger.getAsBoolean()) {
            PneumaticsSubsystem.getInstance().intakeClose();
        }
       // if(drivecontroller.rightBumper.getAsBoolean()) {
           // PneumaticsSubsystem.getInstance().intakeToggle();
        //}
    }

}
