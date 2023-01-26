// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Joystick.AxisType;


public class OI {
  public final XboxController drivecontroller;
  public final XboxController opController;
  public final Joystick flightStick;
  public final JoystickButton buttonA, buttonB, buttonY, buttonX, startButton, backButton, rightBumper, leftBumper, rightTrigger, leftTrigger;
  public final POVButton povNorth, povEast, povSouth, povWest;
  
  public OI(int constant){
    drivecontroller = new XboxController(constant);
    opController = new XboxController(constant);
    // init buttons
    buttonA = new JoystickButton(opController, Button.kA.value);
    buttonB = new JoystickButton(opController, Button.kB.value);
    buttonX = new JoystickButton(opController, Button.kX.value);
    buttonY = new JoystickButton(opController, Button.kY.value);
    //
    startButton = new JoystickButton(opController, Button.kStart.value);
    backButton = new JoystickButton(opController, Button.kBack.value);
    //
    povNorth = new POVButton(opController, 0);
    povEast = new POVButton(opController, 90);
    povSouth = new POVButton(opController, 180);
    povWest = new POVButton(opController, 270);
    //
    rightBumper = new JoystickButton(opController, Button.kRightBumper.value);
    leftBumper = new JoystickButton(opController, Button.kLeftBumper.value);
    //
    leftTrigger = new JoystickButton(opController, Button.kLeftStick.value);
    rightTrigger = new JoystickButton(opController, Button.kRightStick.value);

    //flight stick
    flightStick = new Joystick(constant);
    
  }
}