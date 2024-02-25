package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoStepsSequentialCommands extends SequentialCommandGroup {
    public TwoStepsSequentialCommands(Command command1, Command command2
    ) {
        addCommands(command1,command2);
    }
}