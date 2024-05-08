
package frc.robot.subsystems.swerve.rev;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
<<<<<<< HEAD
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
=======
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
>>>>>>> ad07e2ce00615c343219afa265d54c0743fbf6be

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
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
      
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(new RevSwerveConfig().canCoderConfig);
       
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

         
        relDriveEncoder.setPositionConversionFactor(RevSwerveConfig.driveRevToMeters);
        relDriveEncoder.setVelocityConversionFactor(RevSwerveConfig.driveRpmToMetersPerSecond);

        
        relAngleEncoder = mAngleMotor.getEncoder();
        relAngleEncoder.setPositionConversionFactor(RevSwerveConfig.DegreesPerTurnRotation);
        // in degrees/sec
        relAngleEncoder.setVelocityConversionFactor(RevSwerveConfig.DegreesPerTurnRotation / 60);
    

        resetToAbsolute();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
        
    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(RevSwerveConfig.angleKP, 0);
        controller.setI(RevSwerveConfig.angleKI,0);
        controller.setD(RevSwerveConfig.angleKD,0);
        controller.setFF(RevSwerveConfig.angleKF,0);
        controller.setOutputRange(-RevSwerveConfig.anglePower, RevSwerveConfig.anglePower);
        mAngleMotor.setSmartCurrentLimit(RevSwerveConfig.angleContinuousCurrentLimit);
       
        mAngleMotor.setInverted(RevSwerveConfig.angleMotorInvert);
        mAngleMotor.setIdleMode(RevSwerveConfig.angleIdleMode);

        
       
    }

    private void configDriveMotor()
    {        
        mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(RevSwerveConfig.driveKP,0);
        controller.setI(RevSwerveConfig.driveKI,0);
        controller.setD(RevSwerveConfig.driveKD,0);
        controller.setFF(RevSwerveConfig.driveKF,0);
        controller.setOutputRange(-RevSwerveConfig.drivePower, RevSwerveConfig.drivePower);
        mDriveMotor.setSmartCurrentLimit(RevSwerveConfig.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(RevSwerveConfig.driveMotorInvert);
        mDriveMotor.setIdleMode(RevSwerveConfig.driveIdleMode); 
    
       
       
       
    }

    public Rotation2d getAngleOffset() {
        return angleOffset;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.

        /* Robin: The swerve module has spasm everytime CANcode resets.
         * When a desired state is passed into this method by the status of the joystick,
         * optimize() is called to find a shortcut to the angle needed.
         * Because the range of relative encoder has a maximum of 360 and cannot go
         * beyond for it is periodically (every 20 ms) synced, if the REncoder resets
         * while PID brings the module to the desired state, the REncoder's position is changed,
         * and therefore path to required location changes (PID has a heart attack). 
         * Optimization is called again, and calculates the shortcut, travels, resets, and
         * has a spasm again. 
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

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
            double percentOutput = desiredState.speedMetersPerSecond / RevSwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);
        
    }

    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (RevSwerveConfig.maxSpeed * 0.01)) 
        {
            mAngleMotor.stopMotor();
            return;
        
        }
        Rotation2d angle = desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        SparkPIDController controller = mAngleMotor.getPIDController();
        
        double degReference = angle.getDegrees(); //degReference is correct, within -180 +180 bound
        double lowerResetPos = -180 - angleOffset.getDegrees(); //negative zone
        double upperResetPos = 180 - angleOffset.getDegrees(); //positive zone

        double currentPos = getAngle().getDegrees();
        // I only care about the heading, I don't care whether it is optimized or not because
        // it only resets if the heading passes the boundary

        // if (degReference > upperResetPos) {
        //     degReference -= 360;
        // } else if (degReference < lowerResetPos) {
        //     degReference += 360; 
        // }
        
        /* You need to build a model for this */
       
        /* This is the real problem: because PID is based off of RelEncoder,
        so when CAN resets, the difference to reference varies */ 
        controller.setReference (degReference, ControlType.kPosition, 0);
        
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

    public void resetToAbsolute()
    {
    
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

  

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            relDriveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }
}