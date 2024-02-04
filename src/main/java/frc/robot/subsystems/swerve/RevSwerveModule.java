
package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;

import static frc.robot.Constants.Swerve.swerveCANcoderConfig;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANCoder absolute encoders.
 */
public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;
    private int angleCounter =0;

    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

        /* Angle Encoder Config */

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


        // lastAngle = getState().angle;
    }


    private void configEncoders()
    {
        // absolute encoder
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);


       // relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);


        relAngleEncoder = mAngleMotor.getEncoder();
        //relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        //relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);


        synchronizeEncoders();
       // mDriveMotor.burnFlash();
      // mAngleMotor.burnFlash();

    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(Constants.Swerve.angleKP, 0);
        controller.setI(Constants.Swerve.angleKI,0);
        controller.setD(Constants.Swerve.angleKD,0);
        controller.setFF(Constants.Swerve.angleKFF,0);
        controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        mAngleMotor.setClosedLoopRampRate(Constants.Swerve.angleRampRate);

        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_p, 0);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_p, 0);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_p, 0);
    

    }

    private void configDriveMotor()
    {
         mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);

        controller.setP(Constants.Swerve.driveKP_v,0);
        controller.setI(Constants.Swerve.driveKI,0);
        controller.setD(Constants.Swerve.driveKD,0);
        controller.setFF(Constants.Swerve.driveKFF,0);
        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_v, 0);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_v, 0);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_v, 0);
    
        // set pid profile in slot 1 for position control

        controller.setP(Constants.Swerve.driveKP_p,1);
        controller.setI(Constants.Swerve.driveKI,1);
        controller.setD(Constants.Swerve.driveKD,1);
        controller.setFF(Constants.Swerve.driveKFF,1);
        controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 1);
        controller.setSmartMotionMaxVelocity(Constants.Swerve.maxVel_p, 1);
        controller.setSmartMotionMaxAccel(Constants.Swerve.maxAcc_p, 1);
        controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedErr_p, 1);




    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {


        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, false);

        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kSmartVelocity, 0);

    }

    public void setAngle(SwerveModuleState desiredState)
    {

        if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        {
            mAngleMotor.stopMotor();
            return;

        }
        synchronizeEncoders();
        Rotation2d angle = desiredState.angle;
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        SparkPIDController controller = mAngleMotor.getPIDController();

        double degReference = angle.getDegrees();
        //controller.setReference (degReference, ControlType.kSmartMotion, 1);
        controller.setReference (degReference/15, ControlType.kSmartMotion, 0);
        SmartDashboard.putNumber("Angle Counter",angleCounter++);
         SmartDashboard.putNumber("Turn angle",degReference);

    }



    public Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {

        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    public void synchronizeEncoders()
    {

        double absolutePosition =getCanCoder().getDegrees() - angleOffset.getDegrees();

            SmartDashboard.putNumber("can value" + this.moduleNumber, absolutePosition);
            SmartDashboard.putNumber("sync number" + this.moduleNumber, absolutePosition/15);
            relAngleEncoder.setPosition(absolutePosition/15);
    }



    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega()
    {
        return angleEncoder.getVelocity().getValueAsDouble()/360;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition(),
                getAngle()
        );
    }


    

    public void setPosition(double position)
    {
        synchronizeEncoders();
        SparkPIDController controller = mDriveMotor.getPIDController();
        double encoderDelta = position / Constants.Swerve.driveRevToMeters;
        double currentPosition = mDriveMotor.getEncoder().getPosition();
        controller.setReference (currentPosition + encoderDelta, ControlType.kSmartMotion,1);
        SmartDashboard.putNumber("SetPosition",encoderDelta);
     //   Translation2d e;
    }
}
