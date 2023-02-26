// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.PulleyCommand;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final CommandXboxController opController = new CommandXboxController(OIConstants.kOPControllerPort);


  public SwerveSubsystem s_Swerve = new SwerveSubsystem(_pulley);
  public static PneumaticsSubsystem _pneumatics = new PneumaticsSubsystem();
  public static PulleySubsystem _pulley = new PulleySubsystem(Constants.PullyConstants.pulleyMotorNum);

  private final OI driveStick = new OI(OIConstants.kDriverStickPort);
   private final OI thetaStick = new OI(OIConstants.kDriverStickPort);




  public RobotContainer() {


    autonChooser = new SendableChooser<>();
    SmartDashboard.putData("AutonChooser", autonChooser);  
    configureAutoCommands();
  

//set the swerve drive as a default command for the drive command using the driveController and the flightstick
    s_Swerve.setDefaultCommand(
        new DriveCommand(
            s_Swerve,
            driveController, opController, _pulley));


    _pulley.setDefaultCommand(
        new PulleyCommand(
            _pulley));

    
    
          


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
        //opController.button(7).onTrue(_pneumatics.extensionRetractCommand().andThen(_pneumatics.flipperCloseCommand()).withTimeout(2).andThen(_pulley.homeCommand()));
               
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
//MID Grid RIGHT POLE paths
List<PathPlannerTrajectory> MRConeFRPath = PathPlanner.loadPathGroup("MR Cone, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MRConeMRPath = PathPlanner.loadPathGroup("MR Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MRConeFLPath = PathPlanner.loadPathGroup("MR Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MRConeMLPath = PathPlanner.loadPathGroup("MR Cone, mid left", new PathConstraints(2, 1));
//MID Grid MID SHELF paths
List<PathPlannerTrajectory> MCubeFRPath = PathPlanner.loadPathGroup("M Cube, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MCubeMRPath = PathPlanner.loadPathGroup("M Cube, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MCubeFLPath = PathPlanner.loadPathGroup("M Cube, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MCubeMLPath = PathPlanner.loadPathGroup("M Cube, mid left", new PathConstraints(2, 1));
//Mid Grid LEFT POLE paths
List<PathPlannerTrajectory> MLConeFRPath = PathPlanner.loadPathGroup("ML Cone, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MLConeMRPath = PathPlanner.loadPathGroup("ML Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MLConeFLPath = PathPlanner.loadPathGroup("ML Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> MLConeMLPath = PathPlanner.loadPathGroup("ML Cone, mid left", new PathConstraints(2, 1));
//LEFT Grid RIGHT POLE paths
List<PathPlannerTrajectory> LRConeFRPath = PathPlanner.loadPathGroup("LR Cone, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LRConeMRPath = PathPlanner.loadPathGroup("LR Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LRConeFLPath = PathPlanner.loadPathGroup("LR Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LRConeMLPath = PathPlanner.loadPathGroup("LR Cone, mid left", new PathConstraints(2, 1));
//LEFT Grid MID SHELF paths
List<PathPlannerTrajectory> LCubeFRPath = PathPlanner.loadPathGroup("L Cube, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LCubeMRPath = PathPlanner.loadPathGroup("L Cube, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LCubeFLPath = PathPlanner.loadPathGroup("L Cube, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LCubeMLPath = PathPlanner.loadPathGroup("L Cube, mid left", new PathConstraints(2, 1));
//LEFT Grid LEFT POLE paths
List<PathPlannerTrajectory> LLConeFRPath = PathPlanner.loadPathGroup("LL Cone, far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LLConeMRPath = PathPlanner.loadPathGroup("LL Cone, mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LLConeFLPath = PathPlanner.loadPathGroup("LL Cone, far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> LLConeMLPath = PathPlanner.loadPathGroup("LL Cone, mid left", new PathConstraints(2, 1));
//Just a few paths back tests
List<PathPlannerTrajectory> ReturnFL = PathPlanner.loadPathGroup("back from far left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> ReturnFR = PathPlanner.loadPathGroup("back from far right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> ReturnMR = PathPlanner.loadPathGroup("back from mid right", new PathConstraints(2, 1));
List<PathPlannerTrajectory> ReturnML = PathPlanner.loadPathGroup("back from mid left", new PathConstraints(2, 1));
List<PathPlannerTrajectory> Spin = PathPlanner.loadPathGroup("Spin", new PathConstraints(6, 4));

List<PathPlannerTrajectory> RCubeFR2RCube = PathPlanner.loadPathGroup("Complicated test 1", 2, 1);



// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
//eventMap.put("marker1", new PrintCommand("Passed marker 1"));
eventMap.put("Arrived at piece", _pneumatics.extensionOutCommand().andThen(_pneumatics.intakeOpenCommand()));
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
    //Mid Grid Right Cone Commands (JUST THE TRAJECTORIES)
SequentialCommandGroup MRConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MRConeFRPath));
SequentialCommandGroup MRConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MRConeFLPath));
SequentialCommandGroup MRConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MRConeMRPath));
SequentialCommandGroup MRConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MRConeMLPath));
    //Mid Grid Shelf Commands  (JUST THE TRAJECTORIES)
SequentialCommandGroup MCubeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MCubeFRPath));
SequentialCommandGroup MCubeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MCubeFLPath));
SequentialCommandGroup MCubeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MCubeMRPath));
SequentialCommandGroup MCubeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MCubeMLPath));
    //Mid Grid Left Cone Commands  (JUST THE TRAJECTORIES)
SequentialCommandGroup MLConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MLConeFRPath));
SequentialCommandGroup MLConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MLConeFLPath));
SequentialCommandGroup MLConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MLConeMRPath));
SequentialCommandGroup MLConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(MLConeMLPath));
    //Left Grid Right Cone Commands   (JUST THE TRAJECTORIES)
SequentialCommandGroup LRConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LRConeFRPath));
SequentialCommandGroup LRConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LRConeFLPath));
SequentialCommandGroup LRConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LRConeMRPath));
SequentialCommandGroup LRConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LRConeMLPath));
    //Left Grid Shelf Commands   (JUST THE TRAJECTORIES)
SequentialCommandGroup LCubeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LCubeFRPath));
SequentialCommandGroup LCubeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LCubeFLPath));
SequentialCommandGroup LCubeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LCubeMRPath));
SequentialCommandGroup LCubeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LCubeMLPath));
    //Left Grid Left Cone Commands   (JUST THE TRAJECTORIES)
SequentialCommandGroup LLConeFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LLConeFRPath));
SequentialCommandGroup LLConeFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LLConeFLPath));
SequentialCommandGroup LLConeMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LLConeMRPath));
SequentialCommandGroup LLConeMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(LLConeMLPath));
    //return commands
SequentialCommandGroup returnFRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(ReturnFR));
SequentialCommandGroup returnFLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(ReturnFL));
SequentialCommandGroup returnMRCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(ReturnMR));
SequentialCommandGroup returnMLCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(ReturnML));

SequentialCommandGroup SpinCommand = new SequentialCommandGroup(autoBuilder.followPathGroup(Spin).andThen(_pneumatics.extensionOutCommand().withTimeout(3).andThen(_pneumatics.intakeOpenCommand().withTimeout(2).andThen(_pneumatics.intakeCloseCommand()))));



SequentialCommandGroup Test = new SequentialCommandGroup(autoBuilder.followPathGroup(RLConeMRPath).andThen(autoBuilder.followPathGroup(Spin).andThen(autoBuilder.followPathGroup(ReturnMR))));

SequentialCommandGroup RCubeFR2RCubeCommand = new SequentialCommandGroup(autoBuilder.fullAuto(RCubeFR2RCube));



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
      //Mid Grid Right Cone Commands (JUST THE TRAJECTORIES)
autonChooser.addOption("MRConeFRPath", MRConeFRCommand);
autonChooser.addOption("MRConeFLPath", MRConeFLCommand);
autonChooser.addOption("MRConeMRPath", MRConeMRCommand);
autonChooser.addOption("MRConeMLPath", MRConeMLCommand);
      //Mid Grid Shelf Commands  (JUST THE TRAJECTORIES)
autonChooser.addOption("MCubeFRPath", MCubeFRCommand);
autonChooser.addOption("MCubeFLPath", MCubeFLCommand);
autonChooser.addOption("MCubeMRPath", MCubeMRCommand);
autonChooser.addOption("MCubeMLPath", MCubeMLCommand);
      //Mid Grid Left Cone Commands  (JUST THE TRAJECTORIES)
autonChooser.addOption("MLConeFRPath", MLConeFRCommand);
autonChooser.addOption("MLConeFLPath", MLConeFLCommand);
autonChooser.addOption("MLConeMRPath", MLConeMRCommand);
autonChooser.addOption("MLConeMLPath", MLConeMLCommand);
      //Left Grid Right Cone Commands   (JUST THE TRAJECTORIES)
autonChooser.addOption("LRConeFRPath", LRConeFRCommand);
autonChooser.addOption("LRConeFLPath", LRConeFLCommand);
autonChooser.addOption("LRConeMRPath", LRConeMRCommand);
autonChooser.addOption("LRConeMLPath", LRConeMLCommand);
      //Left Grid Shelf Commands   (JUST THE TRAJECTORIES)
autonChooser.addOption("LCubeFRPath", LCubeFRCommand);
autonChooser.addOption("LCubeFLPath", LCubeFLCommand);
autonChooser.addOption("LCubeMRPath", LCubeMRCommand);
autonChooser.addOption("LCubeMLPath", LCubeMLCommand);
      //Left Grid Left Cone Commands   (JUST THE TRAJECTORIES)
autonChooser.addOption("LLConeFRPath", LLConeFRCommand);
autonChooser.addOption("LLConeFLPath", LLConeFLCommand);
autonChooser.addOption("LLConeMRPath", LLConeMRCommand);
autonChooser.addOption("LLConeMLPath", LLConeMLCommand);
      //return commands
autonChooser.addOption("Return from mid right", returnMRCommand);
autonChooser.addOption("Return from far right", returnFRCommand);
autonChooser.addOption("Return from mid left", returnMLCommand);
autonChooser.addOption("Return from far left", returnFLCommand);
autonChooser.addOption("Spin", SpinCommand);



autonChooser.addOption("Auto Test", Test);

autonChooser.addOption("Complex test", RCubeFR2RCubeCommand);
}
 



  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }


}







