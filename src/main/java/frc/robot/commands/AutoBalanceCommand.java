package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.OI;
import frc.robot.Constants;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCommand extends DriveCommand{
    
public SwerveSubsystem swerveSubsystem;
public PulleySubsystem pulleySubsystem;
public OI driveController;
public CommandXboxController opController;
public double xSpeed;
public double ySpeed;
public double turningSpeed;
PIDController balanceController;
Timer balanceTimer;
Timer isDoneTimer;



public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, OI driveController, CommandXboxController opController, PulleySubsystem pulleySubsystem) {
        super(swerveSubsystem, driveController, opController, pulleySubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.pulleySubsystem = pulleySubsystem;


}



@Override
public void initialize() {

    balanceController = new PIDController(0.008, 0, 0);
    balanceTimer = new Timer();
    isDoneTimer = new Timer();

}

@Override
public void execute() {


balanceTimer.start();

swerveSubsystem.getRoll();

ySpeed = 0;
turningSpeed = 0;

//xSpeed = balanceController.calculate(swerveSubsystem.getRoll(), 0);
balanceController.setTolerance(1.5);

ChassisSpeeds chassisSpeeds;

chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, swerveSubsystem.geRotation2d());
                    swerveSubsystem.lights.set(0.57);


                    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    swerveSubsystem.setModuleStates(moduleStates);


while(swerveSubsystem.getRoll() > -2 || swerveSubsystem.getRoll() < 2) {
    isDoneTimer.start();
    if(isDoneTimer.hasElapsed(3)) {

        isFinished();
        //end the command
    } else {
        isDoneTimer.stop();
        isDoneTimer.reset();
    }
}
if(balanceTimer.hasElapsed(10)) {
    isFinished();
}

    }

     public void autoBalance() {
       initialize();
        execute();
    }

    


@Override
public boolean isFinished() {
    return true;
} 

@Override
public void end(boolean interrupted) {
    balanceTimer.stop();
    isDoneTimer.stop();
    swerveSubsystem.stopModules();
}


}
