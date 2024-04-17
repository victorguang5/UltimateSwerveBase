package frc.robot;

import edu.wpi.first.math.util.Units;

public class SwerveDrive
{
    public final double L = Units.inchesToMeters(23.75); // length
    public final double W = Units.inchesToMeters(23.75); // width

    // Swerve drive operates using two joysticks from a single controller. 
    // The first joystick we will call the ‘strafing’ joystick, 
    // as pushing it will cause the robot to drive in the pointed direction. 
    // The second joystick we will call the ‘rotation’ joystick, 
    // where pushing to the left causes a counterclockwise spin and right causes a clockwise spin.
    public void drive(double x1, double y1, double x2)
    {
        // x1: speed of x
        // y1: speed of y
        // x2: speed of rotation
        double r = Math.sqrt((L * L) + (W * W));
        y1 *= -1;

        // compute variables a, b, c, and d
        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        // compute the speeds for each of the motors
        double backRightSpeed = Math.sqrt((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt((b * b) + (c * c));

        // The wheel angles must also be computed for the four wheels. 
        // These will be in a range of -1 to 1, 
        // if you wish to turn this range into real degrees then simply multiply by 180.
        double backRightAngle = Math.atan2(a, d) / Math.PI;
        double backLeftAngle = Math.atan2(a, c) / Math.PI;
        double frontRightAngle = Math.atan2(b, d) / Math.PI;
        double frontLeftAngle = Math.atan2(b, c) / Math.PI;
    }
}