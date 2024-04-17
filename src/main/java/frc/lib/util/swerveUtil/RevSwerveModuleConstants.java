package frc.lib.util.swerveUtil;

import edu.wpi.first.math.geometry.Rotation2d;

public class RevSwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;

    public RevSwerveModuleConstants(int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset)
    {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }
}
