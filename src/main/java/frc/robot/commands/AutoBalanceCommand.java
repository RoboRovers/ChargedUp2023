package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends CommandBase{
    
public SwerveSubsystem swerveSubsystem;
public DriveCommand driveCommand;

public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, DriveCommand driveCommand) {
    this.swerveSubsystem = swerveSubsystem;
    this.driveCommand = driveCommand;




}

@Override
public void initialize() {

}

@Override
public void execute() {



}



}
