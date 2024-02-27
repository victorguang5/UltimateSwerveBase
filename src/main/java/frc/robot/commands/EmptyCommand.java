package frc.robot.commands;
import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.Command;

public class EmptyCommand extends Command {
    public EmptyCommand() {
    }

    public void initialize() 
    {}

    public void execute()
    {}

    public boolean isFinished()
    {
        return true; 
    }

}


