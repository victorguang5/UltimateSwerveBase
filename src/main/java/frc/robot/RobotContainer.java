package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveBase;

import frc.robot.commands.*;

public class RobotContainer {
    /* Shuffleboard */
    public static ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final SwerveBase s_Swerve = new SwerveBase();
    

    /* Network Tables Elements */

    SendableChooser<Command> movementChooser = new SendableChooser<Command>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        SmartDashboard.putBoolean("auto driving", false);
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),                
                () -> driver.getRawButtonPressed(XboxController.Button.kY.value),
                () -> false
            )
        );
        /* Auto */
        //PathPlannerServer.startServer(5811);
        //movementChooser.setDefaultOption("taxi", new Taxi(s_Swerve));
        movementChooser.addOption("No Movement", new InstantCommand());
        SmartDashboard.putData("Movement", movementChooser);

        /* Networking */
        PortForwarder.add(5800, "10.75.20.40", 5800);
        PortForwarder.add(1181, "10.75.20.40", 1181);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return movementChooser.getSelected();
    }

    public SwerveBase getSwerveBase() {
        return s_Swerve;
    }
}
