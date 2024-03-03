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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.lib.util.PhotonCameraWrapper;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.RobotMoveTargetParameters;
import frc.robot.TargetDetection;
import frc.robot.util.NavXGyro;

import static frc.robot.Constants.Swerve.DegreesPerTurnRotation;
import static frc.robot.Constants.Swerve.chosenModule;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.util.PathPlannerLogging;

public class SwerveBase extends SubsystemBase {

    public Field2d field1 = new Field2d();

    //public PhotonCameraWrapper cam = new PhotonCameraWrapper(Constants.CameraConstants.CAMERA_NAME,
            //Constants.CameraConstants.KCAMERA_TO_ROBOT.inverse());

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
        // reset both gyro and odometer
        resetOdometry(new Pose2d());
        SmartDashboard.putNumber("setDistance", 1);
        SmartDashboard.putNumber("setAngle",90);
        SmartDashboard.putNumber("setDirection",0);
        SmartDashboard.putNumber("goto_x", 0.01);
        SmartDashboard.putNumber("goto_y", 0.01);
        SmartDashboard.putNumber("goto_yaw", 0);
        SmartDashboard.putNumber("endAngle", 0);
        PathPlannerLogging.setLogActivePathCallback((poses) -> field1.getObject("path").setPoses(poses));
        SmartDashboard.putData("field", field1);

    }

    public void wheelsIn() {
  //      SmartDashboard.putNumber("wheelin Counter",wheelinCounter++);
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

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double angle = 0;
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
           if (ally.get() == Alliance.Red) {
               angle = 180;
           }
       }
        ChassisSpeeds desiredChassisSpeeds =
       // fieldRelative ?
       false ?
       //true ?
        ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                swerveOdometer.getEstimatedPosition()
                    .getRotation()
                    .plus(Rotation2d.fromDegrees(angle))
        )
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
        );
        SmartDashboard.putNumber("harry rotate 1", rotation);
        SmartDashboard.putNumber("harry omegaRadiansPerSecond 1", desiredChassisSpeeds.omegaRadiansPerSecond);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        SmartDashboard.putNumber("harry omegaRadiansPerSecond 2", desiredChassisSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("harry omegaRadiansPerSecond 2", desiredChassisSpeeds.omegaRadiansPerSecond);
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : swerveMods){
            SmartDashboard.putNumber("harry swerveModuleStates [" + mod.getModuleNumber() + "]", swerveModuleStates[mod.getModuleNumber()].angle.getDegrees());
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
    public void setDriveSpeed(Translation2d velocity, Rotation2d angularVelocity, Rotation2d angleOfRobot, boolean fieldOriented) {
        ChassisSpeeds desiredChassisSpeeds;
        if (fieldOriented) {
            ChassisSpeeds fieldRSpeed = new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity.getRadians());
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRSpeed, angleOfRobot);
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(), angularVelocity.getRadians());
        }

        //desiredChassisSpeeds = new ChassisSpeeds(0, 0, Math.PI/4);
            // I'm guessing that this line is required to prevent errors or exceptions in cases where the set values exceed certain boundaries?
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds); 
            // Give me the states of all modules if the chassis was theoretical at the given velocity
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
            // Make sure that any calculated wheel speeds do not pass maximum limit
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
            // For every instance of swerve module part of this base, set each module it's desired state
        for(RevSwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], false);
        }
        SmartDashboard.putNumber("setSmartDirection",smartDirectionCounter++);
    }

    /**
     * Turn the robot on the spot to an angle
     * Using Kinematics to calculate the swerve state
     * Can be use for relative angle turnning.
     * @param angle in radians
     * @param time
     */
    private TargetDetection   m_TargetTest = new TargetDetection("HD_Pro_Webcam_C920", TargetDetection.PipeLineType.APRIL_TAG);
    private TargetDetection   m_TargetGamePiece = new TargetDetection("SL-Camera-1", TargetDetection.PipeLineType.COLORED_SHAPE);

    public Pose2d GetPhotonvisionPose2d()
    {
        // this is new for game piece
      RobotMoveTargetParameters data = m_TargetTest.GetSwerveTrainMoveParameters();
      return new Pose2d(
        new Translation2d(
            -data.move.getX(),
            -data.move.getY()
        ),
        data.turn
      );
      //RobotMoveTargetParameters data = m_TargetGamePiece.GetRobotMoveforGamePiece();
        //Pose2d data = m_TargetTest.GetCurrentRobotFieldPose();
         /*if(!data.IsValid) {
            System.out.println("Target not found");
            return null;
        }*/
        /*
        if(data == null) {
             System.out.println("Target not found");
            return null;
        }  */ 
        //System.out.println("Target OK");
     //System.out.printf("new : x %f, y:%f, angle:%f\n", data.getX(), data.getY(), data.getRotation().getDegrees());
       // var angle = data.turn;
       // Translation2d move = data.move;
        
      // Translation2d myInputPosition =  data.move;
        // System.out.printf("Game piece : x %f, y:%f, turn: %f\n", move.getX(), move.getY(), data.turn.getDegrees());

       // return new Pose2d(myInputPosition, angle);
       //return data; //new Pose2d(myInputPosition, angle);
        
    }


    public void setDriveHeading(Rotation2d angle1) {
        //SmartDashboard.putNumber("drive Counter", driveCounter++);
        Rotation2d angle;
        RobotMoveTargetParameters data = m_TargetTest.GetRobotTurnParamters();
        // Rotation2d angle = new Rotation2d().fromDegrees(angleInput);
        angle = data.turn;
        //double angleInput = SmartDashboard.getNumber("setAngle", 45);
       
       // SmartDashboard.putNumber("PhotoAngle", angle.getDegrees());

        ChassisSpeeds desiredChassisSpeeds;
        desiredChassisSpeeds = new ChassisSpeeds(0,0, angle.getRadians());
            // I'm guessing that this line is required to prevent errors or exceptions in cases where the set values exceed certain boundaries?
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds); 
            // Give me the states of all modules if the chassis was theoretical at the given velocity
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
            // Make sure that any calculated wheel speeds do not pass maximum limit
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
            // For every instance of swerve module part of this base, set each module it's desired state
        for(RevSwerveModule mod : swerveMods){
            mod.setOptimizedAngle(swerveModuleStates[mod.getModuleNumber()], 0.6); //Insert any nomber for now
        }
        final double TURNING_CONSTANT = 0.58/90.0;
        for(RevSwerveModule mod: swerveMods) {
            mod.setPositionOtpimized(SmartDashboard.getNumber("setAngle", 0)*TURNING_CONSTANT);
        }
        SmartDashboard.putNumber("setSmartDirection",smartDirectionCounter++);
    }

    /**
     * Drives the robot from one location to another within a given time range. The locations must be field oriented.
     * <code>By Robin</code>
     * Usign ChassisSpeed and Kinematics to calculate the state of swerve drive
     * Then optimize the best rotating direction to reduce the rotation angle.
     * Can be used to drive to relative position withou change the robot Yaw.
     * @param locationTo the Translation2d destination
     * @param locationFrom the Translation2d starting point
     * @param timeToTravel the amount of time the trip should take <code>NOT USED</code>
     * @param angleOfRobot the facing of the robot relative to field
     * 
     */
    // SeanLIU modify this code for game piece
    public void setSmartPositionPoint(Translation2d locationTo, Translation2d locationFrom, double timeToTravel, Rotation2d angleOfRobot) { 
        
        /* comment out below 2 lines for game piece 
        RobotMoveTargetParameters data = m_TargetTest.GetRobotMoveToTargetParamters();
        angleOfRobot = Rotation2d.fromDegrees(0); */

        // this is new for game piece
        RobotMoveTargetParameters data = m_TargetGamePiece.GetRobotMoveforGamePiece();
        if(!data.IsValid) {
            //SmartDashboard.putString("Game piece Data", "ERROR!!!");
            System.out.println("Game piece error");
            return;
        }
        angleOfRobot = data.turn;
        System.out.println("Game piece OK");

        double inputDistance = SmartDashboard.getNumber("setDistance", 1);
        double inputAngle = SmartDashboard.getNumber("setDirection", 0);
        
        Translation2d myInputPosition =  data.move;




        ChassisSpeeds desiredChassisSpeeds;
        // Determine a vector velocity using the change in position
        // Counter from current location, (0,0)
        double deltaX = myInputPosition.getX(); // In meters
        double deltaY = myInputPosition.getY();
        System.out.printf("deltaX %f, deltaY %f, angle %f\n",deltaX, deltaY, angleOfRobot.getDegrees());
        /*if(true) {
            return;
        }*/
        SmartDashboard.putNumber("DeltaX", deltaX);
        SmartDashboard.putNumber("DeltaY", deltaY);
        Translation2d velocity = new Translation2d(deltaX/timeToTravel, deltaY/timeToTravel);
        double distance = velocity.getNorm();


        ChassisSpeeds fieldRSpeed = new ChassisSpeeds(velocity.getX(), velocity.getY(), 0);
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRSpeed, angleOfRobot);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds); 
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(RevSwerveModule mod : swerveMods){
            mod.desiredState = CTREModuleState.optimize(swerveModuleStates[mod.getModuleNumber()], mod.getState().angle);
            mod.setAngle(mod.desiredState);
            if(mod.desiredState.speedMetersPerSecond > 0)
            {
                 mod.setPosition(distance);
            }
            else
            {
                mod.setPosition(-distance);
            }
        }
    }

    /**
     * Yan Hongtao. Drive a distance diagonally 45 degrees
     * Manual calculate the position and moving state. 
     * For test purpose. Not recommend for use
     */
    public void setSmartPosition()
    {
        double distance, direction;
        double currentAngle, deltaAngle;
        distance = SmartDashboard.getNumber("setDistance", 1);
        direction = SmartDashboard.getNumber("setDirection",45);
        double speedMetersPerSecond = 1;
        Rotation2d setDirection = Rotation2d.fromDegrees(direction);
        currentAngle = getPose().getRotation().getDegrees();
        SwerveModuleState state = new SwerveModuleState(speedMetersPerSecond, setDirection);
        deltaAngle = Math.abs(direction - currentAngle);
        // Compensate the distance as turning wheel need time
        if(deltaAngle <= 60)
        {
                distance = (1+ 0.07 / 45 * deltaAngle) * distance;
        }
        for(RevSwerveModule mod : swerveMods)
        {
            // Set Angle first as setPosition will affect the encoder sync
            mod.setAngle(state);
            mod.setPosition(distance);

        }
     //   SmartDashboard.putNumber("setSmartPosition",smartPositionCounter++);
    }

    /**
     * Yan Hongtao. Hard code the angle, make simple logic to turn the swerve
     * Not recommend for use. Just for test purpose
     * @param angle
     */
    public void setSmartAngle(double angl2)
    {

    //SmartDashboard.putNumber("drive Counter", driveCounter++);
        Rotation2d angle0 = new Rotation2d().fromDegrees(0);
        RobotMoveTargetParameters data = m_TargetTest.GetRobotTurnParamters();
        if(!data.IsValid)
        {
            return;
        }
        angle0 = data.turn;
        double angle1 = angle0.getDegrees();

        //double angle1 = SmartDashboard.getNumber("setAngle", 90);
        SmartDashboard.putNumber("TurnAngle", angle1);
        double setAngle;
        Rotation2d direction = Rotation2d.fromDegrees(315);
        SwerveModuleState state = new SwerveModuleState(0.0, direction);
        swerveMods[0].setAngle(state);
        // hard code value, need to change
        state.angle = Rotation2d.fromDegrees(45);
        swerveMods[1].setAngle(state);
        state.angle = Rotation2d.fromDegrees(45);
        swerveMods[2].setAngle(state);
        state.angle = Rotation2d.fromDegrees(315);
        swerveMods[3].setAngle(state);
        for(RevSwerveModule mod : swerveMods){
            if(mod.moduleNumber == 0 || mod.moduleNumber == 2) setAngle = -angle1;
            else setAngle = angle1;
            mod.setPosition(setAngle * Constants.Swerve.turnRatio);
        } 
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
     //   SmartDashboard.putNumber("setDesireStatess",setDesireCounters++);
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
        Rotation2d yaw =  (Constants.Swerve.invertGyro) ? 
        Rotation2d.fromDegrees(360).minus(gyro.getRotation2d()) : 
        gyro.getRotation2d();
        
        SmartDashboard.putNumber("gyro.getRotation2d harry", gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("yaw harry before", yaw.getDegrees());
        
        //yaw = Rotation2d.fromDegrees(yaw.getDegrees() * -1);
        SmartDashboard.putNumber("yaw harry", yaw.getDegrees());
        return yaw;
    }
    public double getPitch() {
        return gyro.getRoll();
    }

    public void synchronizeModuleEncoders(boolean smartMotion) {
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
        SmartDashboard.putNumber("gyro.getRotation2d", gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("gyro.getYaw", gyro.getYaw());
        Rotation2d yaw = getYaw();
        SwerveModulePosition[] latestPosition;
        SmartDashboard.putNumber("yaw", yaw.getDegrees());
        latestPosition = getModulePositions();
        //swerveOdometer.update(getYaw(), getModulePositions());
        swerveOdometer.update(yaw, latestPosition);
        SmartDashboard.putNumber("Odometer.X", swerveOdometer.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometer.Y", swerveOdometer.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Odometer.Angle", swerveOdometer.getEstimatedPosition().getRotation().getDegrees());

        SmartDashboard.putNumber("driveGearRatio", chosenModule.driveGearRatio);
        SmartDashboard.putNumber("wheelCircumference", chosenModule.wheelCircumference);
                
        
        //SmartDashboard.putBoolean("photonGood", cam.latency() < 0.6);
       /*  
        if (!hasInitialized  || DriverStation.isDisabled() ) {
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
*/
        SmartDashboard.putData("field", field);

        field.setRobotPose(getPose());
      //  aprilTagTarget.setBoolean(cam.seesTarget());

    

        avgOmega = getAvgOmega();

        for(SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees() / DegreesPerTurnRotation);
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("REV Mod " + mod.getModuleNumber() + " Position", mod.getPosition().distanceMeters);
            
        }

        // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
        // lib)
        // To ensure that everytime we initialize it works.
        if (avgOmega <= .03 && ++moduleSynchronizationCounter > 20)
        {
            SmartDashboard.putBoolean("Synchronizing Encoders", !SmartDashboard.getBoolean("Synchronizing Encoders", false));
           // synchronizeModuleEncoders(smartMotion);
            moduleSynchronizationCounter = 0;
        }
        if(avgOmega <= .005){
            SmartDashboard.putBoolean("Can Synchronizing Encoders", true);
        }else {
            SmartDashboard.putBoolean("Can Synchronizing Encoders", false);
        }
        SmartDashboard.putNumber("avgOmega", avgOmega);

        SmartDashboard.putBoolean("isRed", DriverStation.getAlliance().equals(DriverStation.Alliance.Red));

        field1.setRobotPose(getPose());
    }

    public void stop() {
        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(mod.getState(), false);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
                Arrays.stream(swerveMods).map(RevSwerveModule::getState).toArray(SwerveModuleState[]::new)
        );
    }

    /**
     * Sets the chassis speeds of the robot.
     *
     * @param chassisSpeeds the target chassis speeds
     * @param isOpenLoop    if open loop control should be used for the drive velocity
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
     * Sets the desired states for all swerve modules.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void  setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
        // makes sure speeds of modules don't exceed maximum allowed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);                
        for(SwerveModule mod : swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }
}