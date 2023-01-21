package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class DriveCommand extends CommandBase {

    private DriveSubsystem _drive;
    private Joystick _driveStick;
    DriveCommand(DriveSubsystem driveSubsystem, Joystick stick) {
        this._drive = driveSubsystem;
        this._driveStick = stick;
        addRequirements(driveSubsystem);
    }

    public void initialize() {
        
    }
    public void execute() {
        double stickFoward = _driveStick.getY();
        double stickStrafe = _driveStick.getX();
        double stickOmega  = _driveStick.getZ();
        _drive.drive(stickFoward, stickStrafe, stickOmega);
    }
    public void end() {

    }
    public boolean isFinished() {
        return false;
    }
}
