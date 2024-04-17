package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;

public class WheelDrive 
{
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

    private final double MAX_VOLTS = 4.95;

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkPIDController pidController;

    // two integers for the motor ports, one integer for the encoder port, 
    // and using default values for the PIDController. 
    // We also need to write the relevant initialization code for the PIDController.
    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        
        this.pidController = this.angleMotor.getPIDController();
        this.pidController.setI(Constants.Swerve.angleKI,0);
        this.pidController.setD(Constants.Swerve.angleKD,0);
        this.pidController.setFF(Constants.Swerve.angleKFF,0);
        this.pidController.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        //this.pidController.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        //pidController.enableContinuousInput(-Math.PI, Math.PI);
        //pidController.setTolerance(THETA_TOLERANCE);
    }    

    public void drive (double speed, double angle) {
        speedMotor.set(speed);

        // Optimization offset can be calculated here.
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); 
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }

        pidController.setReference(setpoint, ControlType.kPosition);
    }
}
