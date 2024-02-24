package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class MySequentialCommands extends SequentialCommandGroup {
    public MySequentialCommands(Command command1, Command command2, Command command3,
    Command command4, Command command5, Command command6
    ) {
        addCommands(command1,command2,command3, command4, command5, command6);
    }
}