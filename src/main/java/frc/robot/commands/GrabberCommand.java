package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.OI;

public class GrabberCommand extends CommandBase {
    
    public final OI drivecontroller;
    public final OI flightStick;
    public final OI opController;

    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public GrabberCommand(PneumaticsSubsystem pneumaticsSubsystem, OI driveController, OI flightStick, OI opController) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.drivecontroller = driveController;
    this.flightStick = flightStick;
    this.opController = opController;
}

    @Override
    public void execute() {

if(opController.rightBumper.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().extensionOut();
    }

if(opController.leftBumper.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().extensionClose();
}
//if(opController.rightBumper.getAsBoolean()) {
  //  PneumaticsSubsystem.getInstance().intakeToggle();
//}

//if(flightStick.


}


}

