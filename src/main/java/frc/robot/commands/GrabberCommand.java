package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.OI;

public class GrabberCommand extends CommandBase {
    
    public final OI drivecontroller;
    public final OI flightStick;

    public final PneumaticsSubsystem pneumaticsSubsystem;
    
public GrabberCommand(PneumaticsSubsystem pneumaticsSubsystem, OI driveController, OI flightStick) {
    this.pneumaticsSubsystem =  pneumaticsSubsystem;
    this.drivecontroller = driveController;
    this.flightStick = flightStick;
}

    @Override
    public void execute() {

if(drivecontroller.rightBumper.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().extensionOut();
    }

if(drivecontroller.leftBumper.getAsBoolean()) {
    PneumaticsSubsystem.getInstance().extensionClose();
}
//if(drivecontroller.rightBumper.getAsBoolean()) {
  //  PneumaticsSubsystem.getInstance().intakeToggle();
//}

//if(flightStick.


}


}

