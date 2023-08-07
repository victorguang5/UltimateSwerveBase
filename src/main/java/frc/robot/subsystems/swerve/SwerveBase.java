package frc.robot.subsystems.swerve;

import frc.lib.math.GeometryUtils;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NavXGyro;

public class SwerveBase extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometer;
    public RevSwerveModule[] SwerveMods;
    public NavXGyro gyro;

    private int moduleSynchronizationCounter = 0;
    private double avgOmega = 0;



    public SwerveBase() {

        gyro = new NavXGyro();

        SwerveMods = new RevSwerveModule[] {

            new RevSwerveModule(0, Constants.Swerve.Modules.Mod0.constants),
            new RevSwerveModule(1, Constants.Swerve.Modules.Mod1.constants),
            new RevSwerveModule(2, Constants.Swerve.Modules.Mod2.constants),
            new RevSwerveModule(3, Constants.Swerve.Modules.Mod3.constants)
        };

        swerveOdometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();

    }
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        rotation,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : SwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : SwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }
    public Pose2d getPose() {
        Pose2d p =  swerveOdometer.getPoseMeters();
        return new Pose2d(-p.getX(),-p.getY(),  p.getRotation());
    }
    public void resetOdometry(Pose2d pose) {

        swerveOdometer.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());

    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : SwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : SwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(double deg) {
        gyro.zeroNavHeading();
        swerveOdometer.update(getYaw(), getModulePositions());
    }

    public void zeroGyro() {
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getHeading()) : Rotation2d.fromDegrees(gyro.getHeading());
    }

    public void synchronizeModuleEncoders() {
        for(RevSwerveModule mod : SwerveMods) {
            mod.synchronizeEncoders();
        }
    }
    public double getAvgOmega() {
        double sum = 0;
        for(RevSwerveModule mod : SwerveMods) {
            sum += Math.abs(mod.getOmega());
        }
        return sum / 4;
    }

    @Override
    public void periodic() {
        avgOmega = getAvgOmega();

        for(SwerveModule mod : SwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
        // lib)
        // To ensure that everytime we initialize it works.
        if (avgOmega <= .03 && ++moduleSynchronizationCounter > 20)
        {
            SmartDashboard.putBoolean("Synchronizing Encoders", !SmartDashboard.getBoolean("Synchronizing Encoders", false));
            synchronizeModuleEncoders();
            moduleSynchronizationCounter = 0;
        }
        if(avgOmega <= .005){
            SmartDashboard.putBoolean("Can Synchronizing Encoders", true);
        }else {
            SmartDashboard.putBoolean("Can Synchronizing Encoders", false);
        }
        SmartDashboard.putNumber("avgOmega", avgOmega);
    }
}
