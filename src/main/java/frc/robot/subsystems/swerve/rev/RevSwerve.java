package frc.robot.subsystems.swerve.rev;

import frc.lib.math.GeometryUtils;
import frc.robot.constants.RevSwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.ToLongFunction;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.lang.Math;

public class RevSwerve extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;



    public RevSwerve() {
        
        gyro = new Pigeon2(RevSwerveConstants.REV.pigeonID);
        gyro.configFactoryDefault();
        
     

        mSwerveMods = new SwerveModule[] {
           
            new RevSwerveModule(0, RevSwerveConstants.Swerve.Mod0.constants),
            new RevSwerveModule(1, RevSwerveConstants.Swerve.Mod1.constants),
            new RevSwerveModule(2, RevSwerveConstants.Swerve.Mod2.constants),
            new RevSwerveModule(3, RevSwerveConstants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(RevSwerveConfig.swerveKinematics, getYaw(), getModulePositions());
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
        translation.getX()/4,
        translation.getY()/4,
        rotation/4,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

        SmartDashboard.putNumber("CanEncoder FrontLeft", mSwerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder FrontRight", mSwerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackLeft", mSwerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackRight", mSwerveMods[3].getCanCoder().getDegrees());

        SmartDashboard.putNumber("REncoder FrontLeft", mSwerveMods[0].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder FrontRight", mSwerveMods[1].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackLeft", mSwerveMods[2].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackRight", mSwerveMods[3].getAngle().getDegrees());

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
    public void resetOdometry(Pose2d pose) {
        
        swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());
       
    }
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


    // d
    public void allSwerveModsSetSpeed(SwerveModuleState desiredState){
        for (SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredState,false);
        }
    }
    public void TurnDegrees(double degrees, double speedMetersPerSecond){
        // 360 degrees = 92.189821 inches
        double metersPerDegree = (92.189821 * 0.0254)/360;

        double metersDist = degrees * metersPerDegree;
        double timeSeconds = metersDist/speedMetersPerSecond;

        SwerveModuleState Mod0State = new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(135));
        SwerveModuleState Mod1State = new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(45));
        SwerveModuleState Mod2State = new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(225));
        SwerveModuleState Mod3State = new SwerveModuleState(speedMetersPerSecond, Rotation2d.fromDegrees(315));

        mSwerveMods[0].setDesiredState(Mod0State,false);
        mSwerveMods[1].setDesiredState(Mod1State,false);
        mSwerveMods[2].setDesiredState(Mod2State,false);
        mSwerveMods[3].setDesiredState(Mod3State,false);

        waitSleep(timeSeconds);

        SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        allSwerveModsSetSpeed(desiredState);
    }

    public void waitSleep(double seconds){
        try {
            Thread.sleep((long)(seconds*1000));
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    public void DriveOneMeter(double travelDistMeters, double speedMetersPerSecond, double driveAngle){
        double travelTimeSeconds = travelDistMeters/speedMetersPerSecond;
        Rotation2d wheelRotation2d = Rotation2d.fromDegrees(driveAngle);
        SwerveModuleState desiredState = new SwerveModuleState(speedMetersPerSecond, wheelRotation2d);

        allSwerveModsSetSpeed(desiredState);
        // new WaitCommand((long)travelTimeSeconds*1000)
        waitSleep(travelTimeSeconds);
        desiredState = new SwerveModuleState(0, wheelRotation2d);
        allSwerveModsSetSpeed(desiredState);

    }
    public void DriveAll(){
        double travelDistMeters = SmartDashboard.getNumber("Distance",1);
        double driveAngle = SmartDashboard.getNumber("DriveAngle",0);
        double speedMetersPerSecond = SmartDashboard.getNumber("Speed",0.2);
        double TurnDegrees = SmartDashboard.getNumber("TurnDegrees",90);
        DriveOneMeter(travelDistMeters, speedMetersPerSecond, driveAngle);
        TurnDegrees(TurnDegrees, speedMetersPerSecond);
    }
    // d end
    
    public void zeroGyro(double deg) {
        if(RevSwerveConfig.invertGyro) {
            deg = -deg;
        }
        gyro.setYaw(deg);
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void zeroGyro() {  
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (RevSwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}