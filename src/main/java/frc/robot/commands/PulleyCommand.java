package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PulleySubsystem;

public class PulleyCommand extends CommandBase {
     
    private final PulleySubsystem pulleySubsystem;
    private final CommandXboxController opController;
    // DigitalInput m_extendlimitSwitch; //assign the limit switch to a GPIO
    // DigitalInput m_retractlimitSwitch; //assign the limit switch to a GPIO

public PulleyCommand(PulleySubsystem pulleySubsystem, CommandXboxController opController) {
        this.pulleySubsystem = pulleySubsystem;
        this.opController = opController;
        addRequirements(pulleySubsystem);

}

@Override
public void initialize() {
    //pulleySubsystem.autoHome();
}



    @Override
    public void execute() {
        SmartDashboard.putNumber("Pulley Encoder Value", pulleySubsystem.pulleyEncoderValue());
    }

}
