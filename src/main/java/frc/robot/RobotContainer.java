// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
//import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  
  // all controllers will be refrenced here

  // Replace with CommandPS4Controller if needed
  private final OI driverController = new OI(OIConstants.kDriverControllerPort);
  public SwerveSubsystem s_Swerve = new SwerveSubsystem();
  private final OI flightStick = new OI(OIConstants.kDriverStickPort);


  public RobotContainer() {

    autonChooser = new SendableChooser<>();
   // autonChooser.addOption("AutonTest", autontest);
    //SmartDashboard.putData("AutonChooser", autonChooser);


    s_Swerve.setDefaultCommand(
        new DriveCommand(
            s_Swerve,
            driverController, flightStick));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
  }

  public Command getAutonCommand() {
    return autonChooser.getSelected();
  }
}