// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.PneumaticsCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.commands.PulleyCommand;
import frc.robot.subsystems.PulleySubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.OI;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SendableChooser<Command> autonChooser;
  
  //Pneumatics
  //public static PneumaticsSubsystem _pneumatics = PneumaticsSubsystem.getInstance();
  public static PneumaticsSubsystem _pneumatics = new PneumaticsSubsystem();




  // Replace with CommandPS4Controller if needed
  private final OI driveController = new OI(OIConstants.kDriverControllerPort);
  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);
  //public static  PulleySubsystem P_systm = PulleySubsystem.getInstance();

  public RobotContainer() {

    autonChooser = new SendableChooser<>();
   // autonChooser.addOption("AutonTest", autontest);
    SmartDashboard.putData("AutonChooser", autonChooser);

//P_systm.setDefaultCommand(
 // new PulleyCommand(P_systm, opController));

    _pneumatics.setDefaultCommand(
      new PneumaticsCommand(
        _pneumatics, 
         opController));

    


    //_pneumatics.setDefaultCommand(
    //  new GrabberCommand(
      //  _pneumatics,
      //   driveController, opController)
    //);
    
    
    
    
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {

 

  }

  public Command getAutonCommand() {
    return autonChooser.getSelected();
  }
}