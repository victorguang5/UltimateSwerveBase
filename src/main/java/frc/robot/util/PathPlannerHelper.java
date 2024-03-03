package frc.robot.util;

import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveBase;

public class PathPlannerHelper {
    static Pose2d lastPhotonPose2d;
    static Pose2d lastEndPose2d;
    static Pose2d lastMidPose2d;

    public static void initializeAutoBuilder(SwerveBase s_Swerve){
        AutoBuilder.configureHolonomic(
            ()->s_Swerve.getPose(),
            (pose) -> {s_Swerve.resetOdometry(pose);},
            ()->s_Swerve.getChassisSpeeds(),
            (chassisSpeeds) -> {s_Swerve.setChassisSpeeds(chassisSpeeds,false);},
            AutoConstants.config,
            getAllianceColorBooleanSupplier(),
            s_Swerve
        );
    }

    public static Command GoToPoseCommand_AprilTag_1Step(SwerveBase s_Swerve)
    {
        lastPhotonPose2d = null;
        lastEndPose2d = null;
        lastMidPose2d = null;
        return new SequentialCommandGroup(
            new InstantCommand(()->{
                var photonPose = GetPhotonPose2d(s_Swerve);
                lastPhotonPose2d = photonPose;

                if (lastPhotonPose2d != null)
                {
                    SmartDashboard.putNumber("photonPose first step endPose_x", lastPhotonPose2d.getX());
                    SmartDashboard.putNumber("photonPose first step endPose_y", lastPhotonPose2d.getY());
                    SmartDashboard.putNumber("photonPose first step endPose_yaw", lastPhotonPose2d.getRotation().getDegrees());
                }
            }),
            new InstantCommand(()->{
                if (lastPhotonPose2d != null)
                {
                    lastEndPose2d = lastPhotonPose2d;
                    var cmd = goToPose_photon(s_Swerve, lastEndPose2d);
                    if (cmd != null)
                        CommandScheduler.getInstance().schedule(cmd);
                }
            })
        );
    }

    public static Command GoToPoseCommand_AprilTag_2Steps(SwerveBase drivetrainSubsystem)
    {
        lastPhotonPose2d = null;
        lastEndPose2d = null;
        lastMidPose2d = null;
        SwerveBase s_Swerve = drivetrainSubsystem;
        return new SequentialCommandGroup(
            new InstantCommand(()->{
                var photonPose = GetPhotonPose2d(s_Swerve);
                lastPhotonPose2d = photonPose;

                if (lastPhotonPose2d != null)
                {
                    SmartDashboard.putNumber("photonPose first step endPose_x", lastPhotonPose2d.getX());
                    SmartDashboard.putNumber("photonPose first step endPose_y", lastPhotonPose2d.getY());
                    SmartDashboard.putNumber("photonPose first step endPose_yaw", lastPhotonPose2d.getRotation().getDegrees());
                }
            }),
            new InstantCommand(()->{
                if (lastPhotonPose2d != null)
                {
                    lastEndPose2d = lastPhotonPose2d;
                    var cmd = goToPose_photon_midPose(s_Swerve, lastEndPose2d);
                    if (cmd != null)
                        CommandScheduler.getInstance().schedule(cmd);
                }
            }),
            new InstantCommand(()->{
                var photonPose = GetPhotonPose2d(s_Swerve);
                if (photonPose != null)
                    lastPhotonPose2d = photonPose;

                if (lastPhotonPose2d != null)
                {
                    SmartDashboard.putNumber("photonPose second step endPose_x", lastPhotonPose2d.getX());
                    SmartDashboard.putNumber("photonPose second step endPose_y", lastPhotonPose2d.getY());
                    SmartDashboard.putNumber("photonPose second step endPose_yaw", lastPhotonPose2d.getRotation().getDegrees());
                }
            }),
            new InstantCommand(()->{
                if (lastPhotonPose2d != null)
                {
                    lastEndPose2d = lastPhotonPose2d;
                    var cmd = goToPose_photon(s_Swerve, lastEndPose2d);
                    if (cmd != null)
                        CommandScheduler.getInstance().schedule(cmd);
                }
            })
        );
    }

    public static Command GoToPoseCommand_Preplanned(SwerveBase s_Swerve, String pathName)
    {
        lastPhotonPose2d = null;
        lastEndPose2d = null;
        lastMidPose2d = null;
        return goToPoseCommand_preplanned(s_Swerve, pathName);
    } 

    public static Command goToPose_photon(SwerveBase s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return null;

        // go to the endPose directly, finish cart heading rotation in the middle
        Pose2d startPose = s_Swerve.getPose();
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );
        
        RotationTarget rt = new RotationTarget(
            0.5, 
            endPose.getRotation(), false);
        List<RotationTarget> rts = Arrays.asList(rt);
        List<ConstraintsZone> czs = Arrays.asList();
        List<EventMarker> ems = Arrays.asList();
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                rts,
                czs, // list ConstraintsZone
                ems, // list event marker
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, endPose.getRotation()), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                false
        );
        
        path.preventFlipping =true;
        
        return (AutoBuilder.followPath(path))
            .andThen(goToPose(s_Swerve, endPose))
            .andThen(goToPose(s_Swerve, endPose))
            .andThen(goToPose(s_Swerve, endPose))
            ;
    }

    public static Command goToPose_photon_midPose(
        SwerveBase s_Swerve, Pose2d endPose)
    {
        return goToPose_photon_midPose(s_Swerve, endPose, 1);
    }

    public static Command goToPose_photon_midPose(
        SwerveBase s_Swerve, Pose2d endPose, 
        double distanceToEndPose)
    {
        if (endPose == null) return null;

        try
        {
            // get the endPose from photonvision
            // if it very close, go there directly
            // otherwise, go to the middle pose first, check photovision again, 
            // get the new endPose, then go to the final endPose
            Pose2d startPose = s_Swerve.getPose();
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                startPose,
                endPose
            );
            double lengthToCompare = distanceToEndPose;
            int size = bezierPoints.size();
            var totalDistance = getTotalDistance(bezierPoints);
            if (totalDistance < lengthToCompare || size < 3)
            {
                return goToPose_photon(s_Swerve, endPose);
            }
            else
            {
                double distance = 0; 
                Translation2d midT = bezierPoints.get(0);
                for(int i=1; i<size; i++)
                {
                    distance += bezierPoints.get(i).getDistance(bezierPoints.get(i-1));
                    midT = bezierPoints.get(i);
                    if(totalDistance - distance <= lengthToCompare)
                        break;
                }
                Pose2d midPose = new Pose2d(midT, endPose.getRotation());
                SmartDashboard.putNumber("midPose_x", midPose.getX());
                SmartDashboard.putNumber("midPose_y", midPose.getY());
                SmartDashboard.putNumber("midPose_yaw", midPose.getRotation().getDegrees());
                
                lastMidPose2d = midPose;

                return goToPose_photon(s_Swerve, midPose);
            }
        }
        catch(Exception ex)
        {
            System.out.println("ERROR: " + ex.getMessage());
            return null;
        }
    }
    
    public static Command goToPose(SwerveBase s_Swerve, Pose2d endPose)
    {
        if (endPose == null) return null;

        // go to the endPose directly
        double endAngle=0;
        //endAngle = SmartDashboard.getNumber("endAngle", 0);
        Pose2d startPose = s_Swerve.getPose();
        endAngle = endPose.getRotation().getDegrees();

        var lastP = endPose.getTranslation();
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = endPose.getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();
        if (curPose.getDistance(lastP) < 0.05)
        {
            if (Math.abs(curAngle - lastAngle) < 1)
                return null;
        }

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
            startPose,
            endPose
        );

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(0.4, 1, Math.PI,  Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(endAngle)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping =true;

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        
        return AutoBuilder.followPath(path);
    }

    public static Command goToPoseCommand_preplanned(SwerveBase s_Swerve, String pathName)
    {
        //s_Swerve.resetOdometry(s_Swerve.getPose());
        PathPlannerPath path = PathPlannerPath.fromPathFile(
            pathName
            //"Example Path"
            //"straight line x"
            // "straight line y"
            //"turn 90"
            //"Path Rotation Target"
            //"turn big 90"
            );
        //path.preventFlipping =true;
        var points = path.getAllPathPoints();
        var lastP = points.get(points.size() - 1).position;
        var curPose = s_Swerve.getPose().getTranslation();
        var lastAngle = path.getGoalEndState().getRotation().getDegrees();
        var curAngle = s_Swerve.getPose().getRotation().getDegrees();

        lastEndPose2d = new Pose2d(lastP, path.getGoalEndState().getRotation());

        if (curPose.getDistance(lastP) < 0.1)
        {
            if (Math.abs(curAngle - lastAngle) < 2)
                return null;
        }

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)
            .andThen(goToPose(s_Swerve, lastEndPose2d))
            .andThen(goToPose(s_Swerve, lastEndPose2d))
            .andThen(goToPose(s_Swerve, lastEndPose2d));
    }

    private static BooleanSupplier getAllianceColorBooleanSupplier(){
        return () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };
    }
    
    private static Pose2d GetPhotonPose2d(SwerveBase s_Swerve)
    {
        var photonPose = s_Swerve.GetPhotonvisionPose2d();
        photonPose = ConvertToAbsolutePose(s_Swerve, photonPose);
        
        SmartDashboard.putBoolean("photonPose found ", (photonPose != null));
        return photonPose;
    }

    private static Pose2d ConvertToAbsolutePose(SwerveBase s_Swerve, Pose2d srcPose)
    {
        if (srcPose == null) return null;

        var deltaPose = srcPose;
        Pose2d startPose = s_Swerve.getPose();

        Translation2d startTranslation2d = startPose.getTranslation();
        Translation2d deltaTranslation2d = deltaPose.getTranslation();
        Translation2d endTranslation2d = startTranslation2d.plus(deltaTranslation2d);
        Rotation2d starRotation2d = startPose.getRotation();
        Rotation2d deltaRotation2d = deltaPose.getRotation();
        Rotation2d endRotation2d = starRotation2d.plus(deltaRotation2d);

        Pose2d endPose = new Pose2d(endTranslation2d,endRotation2d);

        return endPose;
    }

    private static double getTotalDistance(List<Translation2d> waypoints) {
        double totalDistance = 0.0;

        for (int i = 1; i < waypoints.size(); i++) {
            Translation2d currentPoint = waypoints.get(i);
            Translation2d prevPoint = waypoints.get(i - 1);

            double distance = currentPoint.getDistance(prevPoint);
            totalDistance += distance;
        }

        return totalDistance;
    }
}
