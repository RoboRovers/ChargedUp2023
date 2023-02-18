// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
// import frc.robot.subsystems.PulleySubsystem;
//import frc.robot.commands.ZeroGyro;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.OI;
import frc.robot.Constants.OIConstants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;


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

  // init all our controllers and motors
  private final OI driveController = new OI(OIConstants.kDriverControllerPort);
  public SwerveSubsystem s_Swerve = new SwerveSubsystem();
  public static PneumaticsSubsystem _pneumatics = new PneumaticsSubsystem();
  private final OI flightStick = new OI(OIConstants.kDriverStickPort);
//   public static PulleySubsystem _pulley = new PulleySubsystem(Constants.PullyConstants.pulleyMotorNum);
  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);

  public RobotContainer() {


    autonChooser = new SendableChooser<>();
    SmartDashboard.putData("AutonChooser", autonChooser);  
    configureAutoCommands();
  

//set the swerve drive as a default command for the drive command using the driveController and the flightstick
    s_Swerve.setDefaultCommand(
        new DriveCommand(
            s_Swerve,
            driveController, flightStick));


            // _pulley.setDefaultCommand(
            //     new PulleyCommand(
            //       _pulley,
            //        opController));
          
              // _pneumatics.setDefaultCommand(
              //   new PneumaticsCommand(
                  // _pneumatics, 
              //      opController));


    // Configure the button bindings
    configureButtonBindings();


  }

  //dont think this is really necessary because we are defining them in the file OI.java
  private void configureButtonBindings() {
    /* Driver Buttons */
    opController.leftTrigger().toggleOnTrue(_pneumatics.extensionOutCommand());
    opController.leftBumper().toggleOnTrue(_pneumatics.extensionRetractCommand());
    opController.rightTrigger().whileTrue(_pneumatics.intakeOpenCommand());
    opController.rightTrigger().whileFalse(_pneumatics.intakeCloseCommand());

    // opController.povUp().whileTrue(_pulley.liftIntakeCommand());
    // opController.povUp().whileFalse(_pulley.StopCommand());
    // opController.povDown().whileTrue(_pulley.dropIntakeCommand());
    // opController.povDown().whileFalse(_pulley.StopCommand());
    
    opController.button(8).whileTrue(_pneumatics.flipperExtendCommand());
    opController.button(8).whileFalse(_pneumatics.flipperCloseCommand());

    
        //full close reset
        //opController.button(7).onTrue(_pneumatics.extensionRetractCommand().andThen(_pneumatics.flipperCloseCommand()).andThen(_pulley.homeCommand()));
               
    /* 
          
                 //auto mid shelf then reset
           opController.a().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(pulleySubsystem.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
           //mid pole command
           opController.b().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.midPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
            opController.x().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topShelfCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
    
            opController.y().toggleOnTrue((
                _pneumatics.extensionOutCommand().
                andThen(_pulley.topPoleCommand().
                andThen(_pneumatics.intakeOpenCommand().
               // andThen(_pulley.homeCommand().)
                andThen(_pneumatics.extensionRetractCommand().
                andThen(_pneumatics.intakeCloseCommand()))))));
                */
  }

  public void configureAutoCommands() {
    /*Auto Options Flow chart
 * 
 * START-
 * 1. leave commmunity and hold piece                            KEEP
 * 2. place cone or cube and stay
 * 3. place and leave commmunity                                 KEEP
 * 4. 3 + drive to second piece cone or cube                     KEEP
 * 5. 1-2 + drive back to deposit ce                             KEEP
 * 6. 1-3.5 + place game piece                                   KEEP
 * 7. 1-4 + drive onto charge station but don't level
 * 8. 1-4 + drive onto charge station but auto level             KEEP
 */

  // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
//RIGHT Side Grid RIGHT POLE paths
List<PathPlannerTrajectory> RRConeFLPath = PathPlanner.loadPathGroup("RR Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RRConeMLPath = PathPlanner.loadPathGroup("RR Cone, mid left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RRConeMRPath = PathPlanner.loadPathGroup("RR Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RRConeFRPath = PathPlanner.loadPathGroup("RR Cone, far right", new PathConstraints(2, 1));
//RIGHT Side Grid MID SHELF paths
List<PathPlannerTrajectory> RCubeFLPath = PathPlanner.loadPathGroup("R Cube, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RCubeFRPath = PathPlanner.loadPathGroup("R Cube, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RCubeMLPath = PathPlanner.loadPathGroup("R Cube, mid left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RCubeMRPath = PathPlanner.loadPathGroup("R Cube, mid right", new PathConstraints(2, 1));
//RIGHT Side Grid LEFT POLE
List<PathPlannerTrajectory> RLConeFLPath = PathPlanner.loadPathGroup("RL Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RLConeFRPath = PathPlanner.loadPathGroup("RL Cone, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RLConeMRPath = PathPlanner.loadPathGroup("RL Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> RLConeMLPath = PathPlanner.loadPathGroup("RL Cone, mid left", new PathConstraints(2, 1));


// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
PIDController thetaController = new PIDController(
  Constants.AutoConstants.kPThetaController, 0, 0);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("marker1", new PrintCommand("Passed marker 1"));
//eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(s_Swerve::getPose,
 s_Swerve::resetOdometry,
  Constants.DriveConstants.kDriveKinematics,
   new PIDConstants(0.025, 0, 0),
    new PIDConstants(0.025, 0, 0),
     s_Swerve::setModuleStates,
      eventMap,
       true,
        s_Swerve);

//sequential commands
    //Right Grid Right Cone Commands (JUST THE TRATECTORIES)
SequentialCommandGroup RRConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RRConeFRPath));
SequentialCommandGroup RRConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RRConeFLPath));
SequentialCommandGroup RRConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RRConeMRPath));
SequentialCommandGroup RRConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RRConeMLPath));
    //RIght Grid Shelf Commands (JUST THE TRAJECTORIES)
SequentialCommandGroup RCubeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RCubeFRPath));
SequentialCommandGroup RCubeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RCubeFLPath));
SequentialCommandGroup RCubeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RCubeMRPath));
SequentialCommandGroup RCubeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RCubeMLPath));
    //Right Grid Left Pole Commands (JUST THE TRAJECTORIES)
SequentialCommandGroup RLConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RLConeFRPath));
SequentialCommandGroup RLConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RLConeFLPath));
SequentialCommandGroup RLConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RLConeMRPath));
SequentialCommandGroup RLConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(RLConeMLPath));





//add command options
    //Right Grid Right Cone Commands (JUST THE TRATECTORIES)
autonChooser.addOption("RRConeFRPath", RRConeFRCommand);
autonChooser.addOption("RRConeFLPath", RRConeFLCommand);
autonChooser.addOption("RRConeMRPath", RRConeMRCommand);
autonChooser.addOption("RRConeMLPath", RRConeMLCommand);

    //Right Grid Shelf Commands (JUST THE TRAJECTORIES)
autonChooser.addOption("RCubeFRPath", RCubeFRCommand);
autonChooser.addOption("RCubeFLPath", RCubeFLCommand);
autonChooser.addOption("RCubeMRPath", RCubeMRCommand);
autonChooser.addOption("RCubeMLPath", RCubeMLCommand);
    //Right Grid Left Pole Commands (JUST THE TRAJECTORIES)
autonChooser.addOption("RLConeFRPath", RLConeFRCommand);
autonChooser.addOption("RLConeFLPath", RLConeFLCommand);
autonChooser.addOption("RLConeMRPath", RLConeMRCommand);
autonChooser.addOption("RLConeMLPath", RLConeMLCommand);


}
 


  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}







