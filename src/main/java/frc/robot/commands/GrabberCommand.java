package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.OI;

public class GrabberCommand extends CommandBase {
    
    public final OI drivecontroller;
    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public GrabberCommand(PneumaticsSubsystem pneumaticsSubsystem, OI driveController) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.drivecontroller = driveController;
}

    @Override
    public void execute() {

if(drivecontroller.rightTrigger.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().intakeOpen();;
    }

if(drivecontroller.leftTrigger.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().intakeClose();
}
if(drivecontroller.rightBumper.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().intakeToggle();
}


}


}

