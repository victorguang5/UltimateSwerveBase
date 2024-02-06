// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

//import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TargetDetection {

    private PhotonCamera camera;
    private final PipeLineType type;
    private boolean IsOpen = false;
    private boolean already_turned_to_face_target = false;
    private String camera_name;

    public enum PipeLineType {
        APRIL_TAG,
        COLORED_SHAPE,
        REFLECTIVE
    }

    private class PhotonVisonData {
        public boolean is_vaild;
        // 2D Mode
        public double yaw;
        public double pitch;
        public double area;
        public double skew;
        // 3D Mode
        public int april_tag_id;
        public double x_distance;
        public double y_distance;
        public double z_distance;
        public double x_rotate;
        public double y_rotate;
        public double z_rotate;
        public double angle_rotate;
        public double ambiguity;
    }

    public TargetDetection(String camera_name, PipeLineType type) {
        this.type = type;
        this.camera_name = camera_name;
        camera = new PhotonCamera(camera_name);
        if(camera.isConnected()) {
            IsOpen = true;
        } else {
            System.out.printf("Open Camera %s Fail !!!\n", camera_name);
        }
    }

    private RobotMoveTargetParameters GetSwerveTrainMoveParameters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        if(!IsOpen) {
            System.out.printf("Check PhotonVison Camera %s, which is NOT open\n", camera_name);
            camera = new PhotonCamera(camera_name);
            if(camera.isConnected()) {
                IsOpen = true;
                System.out.printf("PhotonVison Camera %s, opened now\n", camera_name);
            } else {    
                return para;
            }
        }
        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get information from target.
            // The yaw of the target in degrees (positive right).            
            double yaw = target.getYaw();
            
            // The pitch of the target in degrees (positive up).
            double pitch = target.getPitch();
            
            // The area (how much of the camera feed the bounding box takes up) as a percent (0-100).            
            double area = target.getArea();           
            if(type == PipeLineType.APRIL_TAG) {
                
                // The ID of the detected fiducial marker.
                if(target.getFiducialId() == Constants.APRILTAG_ID_BE_DETECTED) {
                   /* Get the transform that maps camera space (X = forward, Y = left, Z = up) to object/fiducial tag space 
                      (X forward, Y left, Z up) with the lowest reprojection error.     */
                    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                    //Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
                
                    double x_distance = bestCameraToTarget.getX();
                    double y_distance = bestCameraToTarget.getY();
                    double z_distance = bestCameraToTarget.getZ();
                    Rotation3d trans_3d_rotate = bestCameraToTarget.getRotation();
                    double x_rotate = trans_3d_rotate.getX();
                    double y_rotate = trans_3d_rotate.getY();
                    double z_rotate = trans_3d_rotate.getZ();
                    double a_rotate = trans_3d_rotate.getAngle();
                    System.out.printf("AprilTag raw Data: yaw:%f,pitch:%f,area:%f, 3D: X:%f,Y:%f,Z:%f, RX:%f,RY:%f,RZ:%f,RW:%f\n",
                        yaw, pitch, area,x_distance,y_distance,z_distance,x_rotate,y_rotate,z_rotate,a_rotate); 

                    /*
                    SmartDashboard.putNumber("Yaw", yaw);
                    SmartDashboard.putNumber("Pitch", pitch);
                    SmartDashboard.putNumber("Area", area);
                    SmartDashboard.putNumber("X Distance", x_distance);
                    SmartDashboard.putNumber("Y Distance", y_distance);
                    SmartDashboard.putNumber("Z Distance", z_distance);
                    SmartDashboard.putNumber("Rotate X", x_rotate);
                    SmartDashboard.putNumber("Rotate Y", y_rotate);
                    SmartDashboard.putNumber("Rotate Z", z_rotate);
                    SmartDashboard.putNumber("Rotate Angel", a_rotate);
                    */

                    double tmp_radian = Math.abs(z_rotate);
                    double need_turn_radian = (3.1415926 - tmp_radian);
                    boolean is_turn_left = false;
                    if(tmp_radian > 3.05 && tmp_radian < 3.2) {
                        System.out.println("Already face to ArpilTag, no need to turn");
                    } else {
                        System.out.println("Nee to check turn ......");

                        // this Y distance need to adjust, basically the detect is not accaray if Y is too small
                        if(Math.abs(y_distance) < 0.1) {
                            // if Y is too small, RZ not stable, need YAW assit
                            System.out.println("Y too small, need to move and recheck");
                            return para;
                        } else {        
                            if(y_distance < 0) {
                                if(z_rotate > 0) {
                                   if(true /*para.yaw < 0*/) {
                                        System.out.printf("Tank need to trun right, %f\n", need_turn_radian);
                                        is_turn_left = false;
                                    } else {
                                        System.out.println("cannot process, need to move left/right, check again");
                                        return para;
                                    }
                                } else {
                                    if(yaw > 0) {
                                        System.out.printf("Tank need to trun left, %f\n", need_turn_radian);
                                        is_turn_left = true;
                                    } else {
                                        System.out.println("cannot process, need to move left/right, check again");
                                        return para;
                                    }
                                }
                            } else if(y_distance > 0) {
                                if(z_rotate > 0) {
                                    if(yaw < 0) {
                                        System.out.printf("Tank move trun right, %f\n", need_turn_radian);
                                        is_turn_left = false;
                                    } else {
                                        System.out.println("cannot process, need to move left/right, check again");
                                        return para;
                                    }
                                } else {
                                    if(true /*para.yaw > 0*/){
                                       System.out.printf("Tank move trun left, %f\n", need_turn_radian);
                                       is_turn_left = true;
                                    } else {
                                        System.out.println("cannot process, need to move left/right, check again");
                                        return para;
                                    }
                                } 
                            }
                        }
                    }    
                    // calculate distance and angle                  
                    double z_angle = (3.14159265 - Math.abs(z_rotate));
                    if(z_angle > 0.1) {
                        //double y_dist = Math.abs(y_distance);
                        double y_dist = y_distance;
                        double z_degree = (3.14159265 - z_rotate);
                        double x1 = y_dist * (Math.tan(z_degree));
                        
                        if(Math.cos(z_degree) == 0) {
                            // normally shouldn't be, but just protect 
                            System.out.println("should not be here, but hit here, means 90 degree, still can see target, think over later");
                            return para;
                        }
                        double x2 = y_dist / (Math.cos(z_degree));
                        double x5 = (x_distance - x1);
                        double x4 = x5 * (Math.cos(z_degree));
                        double x6 = (x4 - Constants.SPEAKER_SUBWOOFER_WIDTH);
                        double x3 = x5 * (Math.sin(z_degree));
                        if(x6 == 0) {
                            // normally shoultn't be, but just protect 
                            System.out.println("should not be here, just in case, think over later if still happen");
                            return para;
                        }
                        // direction: first get radian
                        double move_degree = (Math.atan((x2 + x3) / x6));
                        // convert to degree for verify
                        double move_degree_unit_degree = Math.toDegrees(move_degree);
                        double move_distance = Math.sqrt(Math.pow((x2+x3), 2) +  Math.pow(x6, 2));
                        if(is_turn_left) {
                            para.TurnRadian_swerve = (-need_turn_radian);
                            System.out.printf("Moving Result, Left turn angle: %f, MoveAngle: %f, Distance, %f\n", need_turn_radian, move_degree_unit_degree, move_distance);
                        } else {
                            para.TurnRadian_swerve = need_turn_radian;
                            System.out.printf("Moving Result, Right turn angle: %f, MoveAngle: %f, Distance, %f\n", need_turn_radian, move_degree_unit_degree, move_distance);
                        }
                        para.MoveRadian = move_degree; 
                        para.MoveDistance = move_distance;
                    } else {
                        // already face
                        para.TurnRadian_swerve = 0;
                        double x1 = (x_distance - Constants.SPEAKER_SUBWOOFER_WIDTH);
                        if(y_distance != 0.0) {
                            para.MoveRadian = (Math.atan(x1 / y_distance));
                            para.MoveDistance = Math.sqrt(Math.pow(x1, 2) +  Math.pow(y_distance, 2));
                        } else {
                            para.MoveRadian = 0;
                            para.MoveDistance = x1;
                        }                        
                        System.out.printf("Moving Result, No Trun, MoveAngle: %f, Distance, %f\n", para.MoveRadian, para.MoveDistance);
                    }                
                    para.IsValid = true;

                } else {
                    System.out.println("AprilTag ID wrong");
                    return para;
                }
            } else if(type == PipeLineType.COLORED_SHAPE) {
                
                // The skew of the target in degrees (counter-clockwise positive).
                double skew = target.getSkew(); 
                // calculate tank move
                para.IsValid = true;
            } else {
                System.out.println("Currently NOT support");
            }
        }
        
        return para;
    }

    // Get PhotonVison Target Data
    private PhotonVisonData GetPVTargetData() {
        PhotonVisonData target_data = new PhotonVisonData();
        target_data.is_vaild = false;
        if(!IsOpen) {
            System.out.printf("Check PhotonVison Camera %s, which is NOT open\n", camera_name);
            camera = new PhotonCamera(camera_name);
            if(camera.isConnected()) {
                IsOpen = true;
                System.out.printf("PhotonVison Camera %s, opened now\n", camera_name);
            } else {    
                return target_data;
            }
        }

        var result = camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
            // Get information from target.
            // The yaw of the target in degrees (positive right).            
            target_data.yaw = target.getYaw();
            
            // The pitch of the target in degrees (positive up).
            target_data.pitch = target.getPitch();
            
            // The area (how much of the camera feed the bounding box takes up) as a percent (0-100).            
            target_data.area = target.getArea();

            if(type == PipeLineType.APRIL_TAG) {
                
                // The ID of the detected fiducial marker.
                target_data.april_tag_id = target.getFiducialId();
                
                // How ambiguous the pose of the target is must less than 0.2 
                target_data.ambiguity = target.getPoseAmbiguity();

                /* Get the transform that maps camera space 
                   (X = forward, Y = left, Z = up) to object/fiducial tag space 
                   (X forward, Y left, Z up) with the lowest reprojection error.
                */
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                
                target_data.x_distance = bestCameraToTarget.getX();
                target_data.y_distance = bestCameraToTarget.getY();
                target_data.z_distance = bestCameraToTarget.getZ();

                Rotation3d trans_3d_rotate = bestCameraToTarget.getRotation();
                target_data.x_rotate = trans_3d_rotate.getX();
                target_data.y_rotate = trans_3d_rotate.getY();
                target_data.z_rotate = trans_3d_rotate.getZ();
                target_data.angle_rotate = trans_3d_rotate.getAngle();

                // alternate Target, normally don't use it.
                //Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
                target_data.is_vaild = true;
                System.out.printf("AprilTag Raw Data: yaw:%f,pitch:%f,area:%f, 3D: X:%f,Y:%f,Z:%f, RX:%f,RY:%f,RZ:%f,RW:%f, ID:%d, ambiguity:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.x_distance,
                            target_data.y_distance,
                            target_data.z_distance,
                            target_data.x_rotate,
                            target_data.y_rotate,
                            target_data.z_rotate,
                            target_data.angle_rotate,
                            target_data.april_tag_id, 
                            target_data.ambiguity);
            } else if(type == PipeLineType.COLORED_SHAPE) {               
                // The skew of the target in degrees (counter-clockwise positive).
                target_data.skew = target.getSkew(); 
                target_data.is_vaild = true;
                System.out.printf("ColoredShape Raw Data: yaw:%f,pitch:%f,area:%f,skew:%f\n",
                            target_data.yaw, 
                            target_data.pitch, 
                            target_data.area, 
                            target_data.skew);
            } else {
                System.out.println("Currently NOT support");
            }
        } else {
            System.out.println("Target is NOT found");
        }

        return target_data;
    }

    // Get Robot trun to face target paramters
    public RobotMoveTargetParameters GetRobotTurnParamters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        PhotonVisonData data = GetPVTargetData();
        if(data.is_vaild) {
            double tmp_radian = Math.abs(data.z_rotate);
            double need_turn_radian = (3.1415926 - tmp_radian);
            if(tmp_radian > 3.05 && tmp_radian < 3.2) {
                System.out.println("Already face to ArpilTag, no need to turn");
                para.TurnRadian_swerve = 0;
                para.TurnRadian_tank = 1.5707963; // 90 degree
                para.turn = Rotation2d.fromRadians(0);
                para.IsValid = true;
            } else {
                System.out.println("Nee to check turn ......");
                /* this Y distance need to adjust, basically the detect is not accaray if Y is too small
                if(Math.abs(data.y_distance) < 0.1) {
                    // if Y is too small, RZ not stable, need YAW assit
                    System.out.println("Y too small, need to move and recheck");
                    return para;
                } */
                para.TurnRadian_swerve = (data.z_rotate > 0) ? need_turn_radian : -need_turn_radian;
                // here 1.5707963: pi/2
                para.TurnRadian_tank = (data.z_rotate > 0) ? (need_turn_radian - 1.5707963) : (1.5707963 - need_turn_radian);
                para.turn = (data.z_rotate > 0) ? (Rotation2d.fromRadians(-need_turn_radian)) : (Rotation2d.fromRadians(-need_turn_radian));
                System.out.printf("Need to turn (negative left), swerve: %f, tank:%f\n", para.TurnRadian_swerve, para.TurnRadian_tank);
                para.IsValid = true;
            }
            already_turned_to_face_target = true;        
        } else {
            System.out.println("Target Data invalid before detect facing");
        }
        return para;
    }

    // Get Robot move to target paramters
    public RobotMoveTargetParameters GetRobotMoveToTargetParamters() {
        RobotMoveTargetParameters para = new RobotMoveTargetParameters();
        para.IsValid = false;
        if(already_turned_to_face_target) {
            PhotonVisonData data = GetPVTargetData();
            if(data.is_vaild) {
                Optional<DriverStation.Alliance> team = DriverStation.getAlliance();
                if(team.isPresent()) {
                    // different moving strategy based on AprilTag ID 
                    double boundary_distance = 0;
                    double y_dist = data.y_distance;
                    if(team.get() == DriverStation.Alliance.Red) {
                        switch (data.april_tag_id) {
                            case 9: //src right
                                boundary_distance = 0;
                                y_dist += (Constants.SRC_WIDTH + Constants.APRILTAG_WIDTH);    break;
                            case 10: // src left
                                boundary_distance = 0;
                                y_dist -= (Constants.SRC_WIDTH + Constants.APRILTAG_WIDTH); break;
                            case 3: //speaker
                                boundary_distance = Constants.SPEAKER_SUBWOOFER_WIDTH;
                                y_dist += (Constants.SPEAKER_APRIL_TAG_WIDTH + Constants.APRILTAG_WIDTH);    break;
                            case 4: //speaker
                                boundary_distance = Constants.SPEAKER_SUBWOOFER_WIDTH;    break;
                            case 5: //amp
                                boundary_distance = 0;    break;
                            case 11: //stage
                                boundary_distance = 0;    break;
                            case 12: //stage
                                boundary_distance = 0;    break;
                            case 13: //stage
                                boundary_distance = 0;    break;
                            default:
                                System.out.printf("Not support this AprilTag ID:%d\n", data.april_tag_id);
                                return para;
                        }
                    } else if (team.get() == DriverStation.Alliance.Blue){
                        switch (data.april_tag_id) {
                            case 6: // AMP center
                                boundary_distance = 0;    break;
                            case 7: // speaker
                                boundary_distance = Constants.SPEAKER_SUBWOOFER_WIDTH;    break;
                            case 8: //speaker
                                boundary_distance = Constants.SPEAKER_SUBWOOFER_WIDTH;
                                y_dist -= (Constants.SPEAKER_APRIL_TAG_WIDTH + Constants.APRILTAG_WIDTH);   break;
                            case 1: //src right
                                boundary_distance = 0;
                                y_dist += (Constants.SRC_WIDTH + Constants.APRILTAG_WIDTH);    break;
                            case 2: //src left
                                boundary_distance = 0;
                                y_dist -= (Constants.SRC_WIDTH + Constants.APRILTAG_WIDTH);    break;
                            case 14: //stage
                                boundary_distance = 0;    break;
                            case 15: //stage
                                boundary_distance = 0;    break;
                            case 16: //stage
                                boundary_distance = 0;    break;
                            default:
                                System.out.printf("Not support this AprilTag ID:%d\n", data.april_tag_id);
                                return para;
                        }
                    } else {
                        System.out.println("Alliance team Unknown");
                        return para;
                    }
                    System.out.printf("Boundary: %f,Y Distance: %f\n", boundary_distance, y_dist);
                    double x1 = (data.x_distance - boundary_distance);                
                    para.MoveRadian = (-1 * (Math.atan(x1 / y_dist)));
                    para.MoveDistance = Math.sqrt(Math.pow(x1, 2) +  Math.pow(y_dist, 2));
                    para.MoveForward = x1;
                    para.MoveUpward = y_dist;
                    para.move = new Translation2d(x1, y_dist);
                    para.IsValid = true;
                    already_turned_to_face_target = false;
                } else {
                    System.out.println("Allicance Team NOT Correct");
                }
            } else {
                System.out.println("Target Data invalid before detect moving");
            }
        } else {
            System.out.println("Can NOT move to target before turn to face target");
        }
        return para;
    }
}
