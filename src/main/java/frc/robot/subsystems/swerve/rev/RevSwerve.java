package frc.robot.subsystems.swerve.rev;

import frc.lib.math.GeometryUtils;
import frc.robot.constants.RevSwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

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

    Translation2d m_frontLeftlocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontrightlocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftlocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backrightlocation = new Translation2d(-0.381, -0.381);
    
    SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
        m_frontLeftlocation, m_frontrightlocation, m_backLeftlocation, m_backrightlocation
    );

    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    SwerveModuleState[] moduleStates = m_Kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontright = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backright = moduleStates[3];

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
    public void MovewithKinematics() {
        Translation2d translation = new Translation2d(2, 1);    
        double rotation = 0;
        ChassisSpeeds desireChassisSpeeds = new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation);
            desireChassisSpeeds = correctForDynamics(desireChassisSpeeds);
        SwerveModuleState[] SwerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desireChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(SwerveModuleStates[mod.getModuleNumber()], false);
            }
        WaitTime(3000);
    }

    public void dosomething() {
                Translation2d translation = new Translation2d(2, 0);    
        double rotation = 0;
        ChassisSpeeds desireChassisSpeeds = new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation);
            desireChassisSpeeds = correctForDynamics(desireChassisSpeeds);
        SwerveModuleState[] SwerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desireChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(SwerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(SwerveModuleStates[mod.getModuleNumber()], false);
            }
        double distance = SmartDashboard.getNumber("Movedistance", 0);
        double speed = SmartDashboard.getNumber("RobotSpeed", 0);
        WaitTime(distance/speed);
        SwerveModuleState state1 = new SwerveModuleState(0.5328, Rotation2d.fromDegrees(135));
        SwerveModuleState state2 = new SwerveModuleState(0.5328, Rotation2d.fromDegrees(45));
        SwerveModuleState state3 = new SwerveModuleState(0.5328, Rotation2d.fromDegrees(225));
        SwerveModuleState state4 = new SwerveModuleState(0.5328, Rotation2d.fromDegrees(315));
        mSwerveMods[0].setDesiredState(state1, false);
        mSwerveMods[1].setDesiredState(state2, false);
        mSwerveMods[2].setDesiredState(state3, false);
        mSwerveMods[3].setDesiredState(state4, false);
        WaitTime(1000);
        new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        new SwerveModuleState(0, Rotation2d.fromDegrees(225));
        new SwerveModuleState(0, Rotation2d.fromDegrees(135));
        new SwerveModuleState(0, Rotation2d.fromDegrees(315));
        mSwerveMods[0].setDesiredState(state1, false);
        mSwerveMods[1].setDesiredState(state2, false);
        mSwerveMods[2].setDesiredState(state3, false);
        mSwerveMods[3].setDesiredState(state4, false);
        WaitTime(1000);
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
        SwerveModuleState[] swerveModuleStates = RevSwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RevSwerveConfig.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
        SmartDashboard.putNumber("CanEncoder frontleft", mSwerveMods[0].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder frontright", mSwerveMods[1].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder backleft", mSwerveMods[2].getCanCoder().getDegrees());
        SmartDashboard.putNumber("CanEncoder backtright", mSwerveMods[3].getCanCoder().getDegrees());


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

    public void DriveTest() {
        SwerveModuleState state = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        try {
            Thread.sleep(3000); 
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
        state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
    }

    public void WaitTime(double milliseconds) {
      try {
            Thread.sleep((int)milliseconds); 
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }  
    }

    public void DriveAndTurn() {
        //Move forward for 2 seconds, stop for 1 second
        double distance = SmartDashboard.getNumber("Movedistance", 0);
        double speed = SmartDashboard.getNumber("robotspeed", 0);
        double spin = SmartDashboard.getNumber("robotspin", 0);
        double speedspin = SmartDashboard.getNumber("robotspeen", 0);
        double distance1 = SmartDashboard.getNumber("Movedistance", 0);
        double speed1 = SmartDashboard.getNumber("robotspeed", 0);
        double spin1 = SmartDashboard.getNumber("robotspin", 0);
        double speedspin1 = SmartDashboard.getNumber("robotspeen", 0);

        System.out.printf("Distance is: %f Speed is: %f", distance, speed);
        SwerveModuleState state = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        WaitTime(distance/speed*2500);
        state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        
        WaitTime(1000);
        //Turn for 2 seconds, stop for 1 second
        //Angles could be wrong, but since you aren't working on this right now, it doesn't matter right?
        SwerveModuleState state1 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(135));
        SwerveModuleState state2 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(45));
        SwerveModuleState state3 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(225));
        SwerveModuleState state4 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(315));
        mSwerveMods[0].setDesiredState(state1, false);
          mSwerveMods[1].setDesiredState(state2, false);
        mSwerveMods[2].setDesiredState(state3, false);
        mSwerveMods[3].setDesiredState(state4, false);
        WaitTime(34.375 / speedspin * spin / 2);
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        System.out.printf("Distance1 is: %f Speed1 is: %f", distance1, speed1);
        SwerveModuleState state = new SwerveModuleState(speed1, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        WaitTime(distance/speed*2500);
        state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        
        WaitTime(1000);
        //Turn for 2 seconds, stop for 1 second
        //Angles could be wrong, but since you aren't working on this right now, it doesn't matter right?
        SwerveModuleState state1 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(135));
        SwerveModuleState state2 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(45));
        SwerveModuleState state3 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(225));
        SwerveModuleState state4 = new SwerveModuleState(speedspin, Rotation2d.fromDegrees(315));
        mSwerveMods[0].setDesiredState(state1, false);
          mSwerveMods[1].setDesiredState(state2, false);
        mSwerveMods[2].setDesiredState(state3, false);
        mSwerveMods[3].setDesiredState(state4, false);
        WaitTime(34.375 / speedspin * spin / 2);
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(state, false);
        }
        // WaitTime(1000);
        // //Move for 2 seconds, stop
        // state = new SwerveModuleState(0.5, Rotation2d.fromDegrees(0));
        // for(SwerveModule mod : mSwerveMods) {
        //     mod.setDesiredState(state, false);
        // }
        // WaitTime(2000);
        // state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        // for(SwerveModule mod : mSwerveMods) {
        //     mod.setDesiredState(state, false);
        // }
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