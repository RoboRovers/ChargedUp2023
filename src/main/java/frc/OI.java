// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class OI {
  public final CommandXboxController opController;
  
  public OI(int constant){
    opController = new CommandXboxController(constant);
    // init buttons
  }


/* ROBOT BINDINGS FOR OPCONTROLLER-
 * A- auto mid shelf
 * B- auto mid pole
 * X- auto high shelf
 * Y- auto high pole
 * LB- (Limelight function)
 * RB- Home
 * LT- Extension Toggle
 * RT- Intake Toggle
 * DP UP- Extend Out
 * DP DOWN- Extend In
 * DP LEFT- Intake Arm Out
 * DP RIGHT- Intake Arm In
 * 
 * ROBOT BINDINGS FOR DRIVECONTROLLER-
 * L STICK- Strafe 
 * R STICK- Steer
 * R MIDDLE- Field Orriented T/F
 * LB- Reset Gyro
 * A- Lock Wheels (endgame)
 * B- 
 * X- 
 * Y- 
 * 
 * 
 * 
 */



}