package frc.robot.subsystems.swerve.rev;

import frc.lib.math.GeometryUtils;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.robot.constants.RevSwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.sql.Time;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.FaultID;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.lang.Math;

public class RevSwerve extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    double velocityX = 1;
    double velocityY = 0;
    double rotation = 0;
    double MillisecondsToWait = 0;

    public RevSwerve() {
        
        gyro = new AHRS(SPI.Port.kMXP);
        
     

        mSwerveMods = new SwerveModule[] {
           
            new RevSwerveModule(0, RevSwerveConstants.Swerve.Mod0.constants),
            new RevSwerveModule(1, RevSwerveConstants.Swerve.Mod1.constants),
            new RevSwerveModule(2, RevSwerveConstants.Swerve.Mod2.constants),
            new RevSwerveModule(3, RevSwerveConstants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(RevSwerveConfig.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();
        SmartDashboard.putNumber("velocityX", velocityX);
        SmartDashboard.putNumber("velocityY", velocityY);
        SmartDashboard.putNumber("rotation", rotation);
        SmartDashboard.putNumber("MillisecondsToWait", MillisecondsToWait);
        // SmartDashboard.putNumber("RobotYaw", gyro.getYaw());
        // SmartDashboard.putNumber("RobotX", swerveOdometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("RobotY", swerveOdometry.getPoseMeters().getY());
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

    public void MoveWithKinematics() {
        MillisecondsToWait = SmartDashboard.getNumber("MillisecondsToWait",1000);
        velocityX = SmartDashboard.getNumber("velocityX",1);
        velocityY = SmartDashboard.getNumber("velocityY",0);
        rotation = SmartDashboard.getNumber("rotation",0);
        Translation2d translation = new Translation2d(velocityX, velocityY);
        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], false);
        }
        WaitTime(MillisecondsToWait);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX()/4,
        translation.getY()/4,
        rotation/4,
        getYaw())
        : new ChassisSpeeds(
                translation.getX()/4,
                translation.getY()/4,
                rotation/4);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

        /*SmartDashboard.putNumber("CanEncoder FrontLeft", mSwerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder FrontRight", mSwerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackLeft", mSwerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackRight", mSwerveMods[3].getCanCoder().getDegrees());

        SmartDashboard.putNumber("REncoder FrontLeft", mSwerveMods[0].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder FrontRight", mSwerveMods[1].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackLeft", mSwerveMods[2].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackRight", mSwerveMods[3].getAngle().getDegrees());
*/
    }    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, RevSwerveConfig.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }    
    public Pose2d getPose() {
        Pose2d p =  swerveOdometry.getPoseMeters();
        return new Pose2d(-p.getX(),-p.getY(),  p.getRotation());
    }
    // public void resetOdometry(Pose2d pose) {
        
    //     swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
    //     zeroGyro(pose.getRotation().getDegrees());
       
    // }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    // public void zeroGyro(double deg) {
    //     if(RevSwerveConfig.invertGyro) {
    //         deg = -deg;
    //     }
    //     gyro.setYaw(deg);
    //     swerveOdometry.update(getYaw(), getModulePositions());  
    // }

    public void zeroGyro() {  
       gyro.reset();
       swerveOdometry.update(getYaw(), getModulePositions());
    }

    public void TurnAngle(double TurnSpeed, boolean isOpenLoop) {
        SwerveModuleState Mod0 = new SwerveModuleState(TurnSpeed, Rotation2d.fromDegrees(135));
        SwerveModuleState Mod1 = new SwerveModuleState(TurnSpeed, Rotation2d.fromDegrees(45));
        SwerveModuleState Mod2 = new SwerveModuleState(TurnSpeed, Rotation2d.fromDegrees(225));
        SwerveModuleState Mod3 = new SwerveModuleState(TurnSpeed, Rotation2d.fromDegrees(315));
        mSwerveMods[0].setDesiredState(Mod0, false);
        mSwerveMods[1].setDesiredState(Mod1, false);
        mSwerveMods[2].setDesiredState(Mod2, false);
        mSwerveMods[3].setDesiredState(Mod3, false);
        WaitTime(1800);
        Mod0 = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(Mod0, false);
        }
    }

    private void WaitTime(double milliseconds) {
        try {
            Thread.sleep((int)milliseconds);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    public void DriveForward(double DriveSpeed, double Distance, boolean isOpenLoop) {
        SwerveModuleState desiredState = new SwerveModuleState(DriveSpeed, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredState, false);
        }
        WaitTime(Distance/DriveSpeed*1000);
        desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredState, false);
        }
    }

    public void moveforward() {
        double Distance = SmartDashboard.getNumber("Distance",1);
        double DriveSpeed = SmartDashboard.getNumber("DriveSpeed",1);
        double TurnAngleSpeed = SmartDashboard.getNumber("AngleSpeed",1);
        DriveForward(DriveSpeed, Distance, false);
        //WaitTime(2700);
        WaitTime(500);
        TurnAngle(TurnAngleSpeed, false);
        //WaitTime(1800);
    }

    public Rotation2d getYaw() {
        return (RevSwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);    
            SmartDashboard.putNumber("RobotYaw", swerveOdometry.getPoseMeters().getRotation().getDegrees());
            SmartDashboard.putNumber("RobotX", swerveOdometry.getPoseMeters().getX());
            SmartDashboard.putNumber("RobotY", swerveOdometry.getPoseMeters().getY());
        }
    }
}