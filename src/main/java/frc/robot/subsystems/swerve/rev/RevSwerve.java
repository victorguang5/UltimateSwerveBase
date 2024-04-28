package frc.robot.subsystems.swerve.rev;

import frc.lib.math.GeometryUtils;
import frc.robot.constants.RevSwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RevSwerve extends SubsystemBase {


    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    boolean syncDelay[] = {false, false, false, false};



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
        /* Depending on boolean, create chassis speed relative to bot or field */
        ChassisSpeeds desiredChassisSpeeds = fieldRelative ?
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX()/4, translation.getY()/4, rotation/4,  getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

        syncToAbsoluteAll();
        /* When the CanCoder resets from + to -, the module begins spazzing out. because offset is constant and
        Rencoder changes and resets in 1:1 proportion. Could the problem be in setDesiredState()   */

        SmartDashboard.putNumber("CanEncoder FrontLeft", mSwerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder FrontRight", mSwerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackLeft", mSwerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder BackRight", mSwerveMods[3].getCanCoder().getDegrees());

        SmartDashboard.putNumber("REncoder FrontLeft", mSwerveMods[0].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder FrontRight", mSwerveMods[1].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackLeft", mSwerveMods[2].getAngle().getDegrees());
        SmartDashboard.putNumber("REncoder BackRight", mSwerveMods[3].getAngle().getDegrees());

    }    

    /**
     * Resets all swerve module encoders to sync with CAN encoder. 
     * <p>
     * When modules get close to their reset positions,
     * syncing is delayed by one tick. Normally, sync every 20ms, but when any wheel approaches reset, it will not sync. Without delay,
     * modules will spasm and flicker because of the fast changes in the relative encoders position; PID which
     * sets a refernce based on that relative encoder position goes back and forth around the reset position.
     * By delaying the sync, the relative encoder may go out of bounds briefly, allowing the module to pass the
     * reset position less violently. However, when the wheels leave the the "delay" zone, they will reset eventually,
     * and still cause a slight flicker, but not as continuous and amplified. 
     * <p>
     * 2024.4.27 The program only mitigates the spasm, but does not remove it. PID ultimately will spasm no matter what when
     * the current position is reset.
     * <p>
     * By Robin
     */
    public void syncToAbsoluteAll() {
        for(int i = 0; i < 4; i++){
            double lowerResetPos = -180 - mSwerveMods[i].getAngleOffset().getDegrees(); //negative zone
            double upperResetPos = 180 - mSwerveMods[i].getAngleOffset().getDegrees(); //positive zone
            int bounds = 5; //How close to reset position modules must be to activate delay
            double currentPos = mSwerveMods[i].getAngle().getDegrees();
            if (((currentPos < lowerResetPos + bounds)&&(currentPos > upperResetPos - bounds - 360))
            || ((currentPos > upperResetPos - bounds)&&(currentPos < lowerResetPos + bounds + 360))) {
                // if (syncDelay[i]) { // Lag the sync if PID is referencing out of bounds
                //     mSwerveMods[i].resetToAbsolute();
                //     syncDelay[i] = false;
                // } else {
                //     syncDelay[i] = true;
                // }
                //Makes use of periodic calling.
            } else { //When not near reset positions, sync normally.
                mSwerveMods[i].resetToAbsolute();
                for (int j = 0; j < 4; j++) {
                    syncDelay[j] = false;
                }
                
            }
            
        }
        
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