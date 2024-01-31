package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.lib.util.PhotonCameraWrapper;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.NavXGyro;

import java.util.Map;

public class SwerveBase extends SubsystemBase {

    public PhotonCameraWrapper cam = new PhotonCameraWrapper(Constants.CameraConstants.CAMERA_NAME,
            Constants.CameraConstants.KCAMERA_TO_ROBOT.inverse());

    public SwerveDrivePoseEstimator swerveOdometer;
    public RevSwerveModule[] swerveMods;
    public NavXGyro gyro = NavXGyro.getInstance();

    private int moduleSynchronizationCounter = 0;
    private double avgOmega = 0;
    static int smartPositionCounter=0;
    static int driveCounter = 0;
    static int setDesireCounters = 0;
    static int stopCounter = 0;
    static int wheelinCounter =0;
    static int smartDirectionCounter = 0;
//    private Rotation2d fieldOffset = new Rotation2d(gyro.getYaw()).rotateBy(new Rotation2d(180));
    private final Field2d field = new Field2d();
    private boolean hasInitialized = false;

    private GenericEntry aprilTagTarget = RobotContainer.autoTab
            .add("Currently Seeing April Tag", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("Color when true", "green", "Color when false", "red"))
            .withPosition(8, 4).withSize(2, 2).getEntry();



    public SwerveBase() {

        swerveMods = new RevSwerveModule[] {

            new RevSwerveModule(0, Constants.Swerve.Modules.Mod0.constants),
            new RevSwerveModule(1, Constants.Swerve.Modules.Mod1.constants),
            new RevSwerveModule(2, Constants.Swerve.Modules.Mod2.constants),
            new RevSwerveModule(3, Constants.Swerve.Modules.Mod3.constants)
        };

        swerveOdometer = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d());
        zeroGyro();

    }

    public void wheelsIn() {
        SmartDashboard.putNumber("wheelin Counter",wheelinCounter++);
        swerveMods[0].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(45)), false);
        swerveMods[1].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(135)), false);
        swerveMods[2].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-45)), false);
        swerveMods[3].setDesiredState(new SwerveModuleState(2, Rotation2d.fromDegrees(-135)),
                false);
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) 
    {
        SmartDashboard.putNumber("drive Counter", driveCounter++);
        ChassisSpeeds desiredChassisSpeeds = fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveOdometer.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(DriverStation.getAlliance() == DriverStation.Alliance.Red ? 180 : 0))
        )
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
        );
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }
    
    /**
     * Sets the overal robot state based on a desired velocities, and angle if the given velocity is meant to be field oriented. <code>By Robin</code>
     * This is a testing method
     * @param velocity the Translation2d vector 
     * @param angularVelocity the Rotation2d angular velocity
     * @param angleOfRobot the Rotation2d angle that the robot is facing according to the field orientation
     * @param fieldOriented a boolean indicating if the given velocities are meant to be field oriented
     */
    public void setDriveDirection(Translation2d velocity, Rotation2d angularVelocity, Rotation2d angleOfRobot, boolean fieldOriented) {
        //SmartDashboard.putNumber("drive Counter", driveCounter++);
        ChassisSpeeds desiredChassisSpeeds;
        if (fieldOriented) {
            ChassisSpeeds fieldRSpeed = new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity.getRadians());
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRSpeed, angleOfRobot);
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity.getRadians());
        }
            // I'm guessing that this line is required to prevent errors or exceptions in cases where the set values exceed certain boundaries?
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds); 
            // Give me the states of all modules if the chassis was theoretical at the given velocity
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
            // Make sure that any calculated wheel speeds do not pass maximum limit
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
            // For every instance of swerve module part of this base, set each module it's desired state
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], false);
        }
    }

    /**
     * Drives the robot from one location to another within a given time range. The locations must be field oriented.
     * <code>By Robin</code>
     * 
     * @param locationTo the Translation2d destination
     * @param locationFrom the Translation2d starting point
     * @param timeToTravel the amount of time the trip should take <code>NOT USED</code>
     * @param angleOfRobot the facing of the robot relative to field
     * 
     */
    public void setSmartPositionPoint(Translation2d locationTo, Translation2d locationFrom, double timeToTravel, Rotation2d angleOfRobot) {
        //SmartDashboard.putNumber("drive Counter", driveCounter++);
        ChassisSpeeds desiredChassisSpeeds;
        // Determine a vector velocity using the change in position
        double deltaX = locationTo.getX() - locationFrom.getX(); // In meters
        double deltaY = locationTo.getY() - locationFrom.getY();
        Translation2d velocity = new Translation2d(deltaX/timeToTravel, deltaY/timeToTravel);
        double distance = velocity.getNorm();

        ChassisSpeeds fieldRSpeed = new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRSpeed, angleOfRobot);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds); 
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for(RevSwerveModule mod : swerveMods){
            mod.setAngle(swerveModuleStates[mod.getModuleNumber()]);
            mod.setPosition(distance);
        }
    }

    public void setSmartPosition()
    {
        double speedMetersPerSecond = 1;
        Rotation2d angle = Rotation2d.fromDegrees(45);

        SwerveModuleState state = new SwerveModuleState(speedMetersPerSecond, angle);
        for(RevSwerveModule mod : swerveMods){
            mod.setPosition(1);
            mod.setAngle(state);
        }
        SmartDashboard.putNumber("setSmartPosition",smartPositionCounter++);
    }

    public void setSmartDirection(double angle)
    {
        Rotation2d direction = Rotation2d.fromDegrees(45);
        SwerveModuleState state = new SwerveModuleState(0.0, direction);
        swerveMods[0].setAngle(state);
        SmartDashboard.putNumber("wheel 1",state.angle.getDegrees());
        state.angle = Rotation2d.fromDegrees(-45);
        swerveMods[1].setAngle(state);
        SmartDashboard.putNumber("wheel 2",state.angle.getDegrees());
        state.angle = Rotation2d.fromDegrees(135);
        swerveMods[2].setAngle(state);
        SmartDashboard.putNumber("wheel 3",state.angle.getDegrees());
        state.angle = Rotation2d.fromDegrees(-135);
        swerveMods[3].setAngle(state);
        SmartDashboard.putNumber("wheel 4",state.angle.getDegrees());
        for(RevSwerveModule mod : swerveMods){
            mod.setPosition(angle * Constants.Swerve.turnRatio);
        } 
        SmartDashboard.putNumber("setSmartDirection",smartDirectionCounter++);
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        SmartDashboard.putNumber("setDesireStatess",setDesireCounters++);
       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometer.getEstimatedPosition();
    }
    public void resetOdometry(Pose2d pose) {

        swerveOdometer.resetPosition(new Rotation2d(), getModulePositions(), pose);
        zeroGyro(pose.getRotation().getDegrees());

    }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(double deg) {
        gyro.reset();
    }

    public void zeroGyro() {
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360).minus(gyro.getRotation2d()) : gyro.getRotation2d();
    }
    public double getPitch() {
        return gyro.getRoll();
    }

    public void synchronizeModuleEncoders() {
        for(RevSwerveModule mod : swerveMods) {
            mod.synchronizeEncoders();
        }
    }
    public double getAvgOmega() {
        double sum = 0;
        for(RevSwerveModule mod : swerveMods) {
            sum += Math.abs(mod.getOmega());
        }
       return sum / 4;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro", gyro.getHeading());

        swerveOdometer.update(getYaw(), getModulePositions());
        SmartDashboard.putBoolean("photonGood", cam.latency() < 0.6);
        if (!hasInitialized /* || DriverStation.isDisabled() */) {
            var robotPose = cam.getInitialPose();
            if (robotPose.isPresent()) {
                swerveOdometer.resetPosition(getYaw(), getModulePositions(), robotPose.get());
                hasInitialized = true;
            }
        } else {
            var result = cam.getEstimatedGlobalPose(swerveOdometer.getEstimatedPosition());
            if (result.isPresent()) {
                var camPose = result.get();
                if (camPose.targetsUsed.get(0).getArea() > 0.7) {
                    swerveOdometer.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
                            camPose.timestampSeconds);
                }
                field.getObject("Cam Est Pose").setPose(camPose.estimatedPose.toPose2d());
            } else {
                field.getObject("Cam Est Pose").setPose(new Pose2d(-100, -100, new Rotation2d()));
            }
        }

        SmartDashboard.putData("field", field);

        field.setRobotPose(getPose());
        aprilTagTarget.setBoolean(cam.seesTarget());

        avgOmega = getAvgOmega();

        for(SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Position", mod.getPosition().distanceMeters);
            
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

        SmartDashboard.putBoolean("isRed", DriverStation.getAlliance() == DriverStation.Alliance.Red);
    }

    public void stop() 
    {
        SmartDashboard.putNumber("stopCounter",stopCounter++);
        SmartDashboard.putNumber("avgOmega", avgOmega);
        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(mod.getState(), false);
        }
    }
}
