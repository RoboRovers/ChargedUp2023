// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;


/*Auto Options Flow chart
 * 
 * START-
 * 1. leave commmunity and hold piece                            KEEP
 * 3. place and leave commmunity                                 KEEP
 * 4. 3 + drive to second piece cone or cube                     KEEP
 * 5. 1-2 + drive back to deposit ce                             KEEP
 * 6. 1-3.5 + place game piece                                   KEEP
 * 8. 1-4 + drive onto charge station but auto level             KEEP
 * END-
 */

public final class Autos {
     // A simple auto routine that drives forward a specified distance, and then stops.
     public final Command m_placeNLeave = new CommandBase() {
        
     };

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  }



