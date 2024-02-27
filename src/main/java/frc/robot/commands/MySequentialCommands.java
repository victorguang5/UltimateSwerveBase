package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class MySequentialCommands extends SequentialCommandGroup {
    public MySequentialCommands(Command command1, Command command2, Command command3,
    Command command4, Command command5, Command command6
    ) {
        if (command1 == null) command1 = new EmptyCommand();
        if (command2 == null) command2 = new EmptyCommand();
        if (command3 == null) command3 = new EmptyCommand();
        if (command4 == null) command4 = new EmptyCommand();
        if (command5 == null) command5 = new EmptyCommand();
        if (command6 == null) command6 = new EmptyCommand();
        addCommands(command1,command2,command3, command4, command5, command6);
    }
}