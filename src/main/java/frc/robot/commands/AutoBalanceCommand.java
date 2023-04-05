package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
// Timer isDoneTimer;
// public boolean balanceDone;



public AutoBalanceCommand(SwerveSubsystem swerveSubsystem, OI driveController, CommandXboxController opController, PulleySubsystem pulleySubsystem) {
        super(swerveSubsystem, driveController, opController, pulleySubsystem);
        this.swerveSubsystem = swerveSubsystem;
        this.pulleySubsystem = pulleySubsystem;


}



@Override
public void initialize() {

    balanceController = new PIDController(0.0425, 0, 0.03);
    balanceTimer = new Timer();
    // balanceDone = false;
    balanceTimer.start();

}

@Override
public void execute() {

SmartDashboard.putNumber("Balance Timer", balanceTimer.get());

swerveSubsystem.getRoll();

xSpeed = 0;
turningSpeed = 0;

ySpeed = (balanceController.calculate(swerveSubsystem.getRoll(), 0));
balanceController.setTolerance(10);



ChassisSpeeds chassisSpeeds;

chassisSpeeds = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);
                    swerveSubsystem.lights.set(0.57);


                    SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                    swerveSubsystem.setModuleStates(moduleStates);



if(swerveSubsystem.getRoll() < 3 && swerveSubsystem.getRoll() > -3) {
    turningSpeed = 0.01;
    Commands.waitSeconds(0.5);
    swerveSubsystem.stopModules();
}

if(swerveSubsystem.getRoll() < -50 || swerveSubsystem.getRoll() > 50) {
    swerveSubsystem.stopModules();
    
}



// if(balanceTimer.hasElapsed(15)) {
// balanceDone = true;
// }



    }

     public void autoBalance() {
       initialize();
        execute();
        System.out.print("Execute ran");
    }

    


@Override
public boolean isFinished() {
    // return balanceDone;
    return false;
} 

@Override
public void end(boolean interrupted) {
    balanceTimer.stop();
    // isDoneTimer.stop();
    swerveSubsystem.stopModules();
}


}
