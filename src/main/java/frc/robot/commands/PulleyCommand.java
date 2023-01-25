package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.PulleySubsystem;
import frc.OI;

public class PulleyCommand extends CommandBase {
    
    private final PulleySubsystem pulleySubsystem;
    private final OI driveController;

public PulleyCommand(PulleySubsystem pulleySubsystem, OI driverController) {
        this.pulleySubsystem = pulleySubsystem;
        this.driveController = driverController;
}



    @Override
    public void execute() {

    if(driveController.buttonY.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }

    if(driveController.buttonX.getAsBoolean())
    {
        pulleySubsystem.liftIntake();
    }


    }

}
