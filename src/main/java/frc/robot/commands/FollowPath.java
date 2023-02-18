package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
//controller imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//position tracker imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//trajectory imports
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//file imports
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
//path planner imports
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class FollowPath {

  SparkMaxPIDController turningPidController;
 SwerveSubsystem s_Swerve = new SwerveSubsystem();
 
/*public void GarageTest() {
        turningPidController = SwerveModule.steerMotor.getPIDController();
       
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        //ArrayList<PathPlannerTrajectory> GarageTestPaths = 
        List<PathPlannerTrajectory> GarageTestPaths =
        PathPlanner.loadPathGroup("Garage Test",
         new PathConstraints(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

           
                
            // 3. Define PID controllers for tracking trajectory
            PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
            PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
            PIDController thetaController = new PIDController(
              Constants.AutoConstants.kPThetaController, 0, 0);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
            SequentialCommandGroup GarageTest = new SequentialCommandGroup(
                new FollowPathWithEvents(s_Swerve.followTrajectoryCommand(GarageTestPaths, false), null, null)
          );
        }
*/

}




