// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class OI {
  public final XboxController drivecontroller;
  //public final CommandXboxController opController;
  
  public OI(int constant){
    drivecontroller = new XboxController(constant);
   CommandXboxController opController = new CommandXboxController(constant);
    // init buttons
    Trigger aButton = opController.a();
    Trigger bButton = opController.b();
    Trigger xButton = opController.x();
    Trigger yButton = opController.y();
    //
    Trigger startButton = opController.start();
    Trigger menuButton = opController.button(7);
    //
    Trigger rTrigger = opController.rightTrigger();
    Trigger lTrigger = opController.leftTrigger();
    //
    Trigger upButton = opController.povUp();
    Trigger downButton = opController.povDown();
    Trigger leftButton = opController.povLeft();
    Trigger rightButton = opController.povRight();

    //
    Trigger rbumper = opController.rightBumper();
    Trigger lbumper = opController.leftBumper();
    //

  }


/* ROBOT BINDINGS FOR OPCONTROLLER-
 * A- auto mid shelf
 * B- auto mid pole
 * X- auto high shelf
 * Y- auto high pole
 * LB- Home
 * RB- (Limelight function)
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