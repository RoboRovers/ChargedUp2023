// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.OI;
// import frc.robot.subsystems.PneumaticsSubsystem;

// public class PneumaticsCommands extends CommandBase{
    
// public PneumaticsSubsystem _pneumatics;
// public CommandXboxController opController;
// public OI testController;


// public PneumaticsCommands(PneumaticsSubsystem pneumaticsSubsystem, CommandXboxController opController, OI testController) {
// this._pneumatics = pneumaticsSubsystem;
// this.testController = testController;


// }



// @Override
// public void execute() {
//     opController.leftTrigger().toggleOnTrue(_pneumatics.extensionOutCommand());
//     opController.leftBumper().toggleOnTrue(_pneumatics.extensionRetractCommand());
//     opController.rightTrigger().whileTrue(_pneumatics.intakeOpenCommand());
//     opController.rightTrigger().whileFalse(_pneumatics.intakeCloseCommand());

// if(testController.buttonY.getAsBoolean()) {
// _pneumatics.extensionOut();


// } else {

// }
    
// }

// @Override
// public boolean isFinished() {
//    // return 
// }

// }
