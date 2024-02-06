// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class RobotMoveTargetParameters {
    // for swerve, turn: face to AprilTag
    public double TurnRadian_swerve; // negative value: left, postive: right, unit: radian
    public double MoveRadian; // same as above, for moving
    public double MoveDistance;

    Translation2d move;
    Rotation2d turn;

    // for tank, turn is parallel to AprilTag
    public double TurnRadian_tank; // negative value: left, postive: right, unit: radian
    public double MoveForward; // same as above, for moving
    public double MoveUpward;

    public boolean IsValid;  // data valid flag
}


 
       


