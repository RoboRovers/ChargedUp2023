// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI {
  public final CommandXboxController opController;
  public final XboxController controller;
  public final Joystick flightStick;
  public final POVButton povNorth, povEast, povSouth, povWest;
  public final JoystickButton buttonA, buttonB, buttonY, buttonX, startButton, backButton, rightBumper, leftBumper;

  public OI(int constant){
    opController = new CommandXboxController(constant);
  //Drive Controller buttons
    // init buttons
      controller = new XboxController(constant);
      // init buttons
      buttonA = new JoystickButton(controller, Button.kA.value);
      buttonB = new JoystickButton(controller, Button.kB.value);
      buttonX = new JoystickButton(controller, Button.kX.value);
      buttonY = new JoystickButton(controller, Button.kY.value);
      //
      startButton = new JoystickButton(controller, Button.kStart.value);
      backButton = new JoystickButton(controller, Button.kBack.value);
      //
      povNorth = new POVButton(controller, 0);
      povEast = new POVButton(controller, 90);
      povSouth = new POVButton(controller, 180);
      povWest = new POVButton(controller, 270);
      //
      rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
      leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
      //
      flightStick = new Joystick(constant);
  
    }
  


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
 * DP UP- Field Orriented T/F
 * DP DOWN- Reset Gyro
 * A- Lock Wheels (endgame)
 * B- 
 * X- 
 * Y- 
 * 
 * 
 * 
 */



