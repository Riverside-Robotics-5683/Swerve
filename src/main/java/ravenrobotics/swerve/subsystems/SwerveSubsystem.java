package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.swerve.Constants;
import ravenrobotics.swerve.Constants.SwerveDriveKinematic;
import ravenrobotics.swerve.Constants.SwerveModuleConfigs;

public class SwerveSubsystem extends SubsystemBase
{
    //Swerve modules
    private final SwerveModule frontLeftModule = new SwerveModule(SwerveModuleConfigs.kFrontLeftConfig, SwerveModuleConfigs.kDriveCANbus, "FL");
    private final SwerveModule frontRightModule = new SwerveModule(SwerveModuleConfigs.kFrontRightConfig, SwerveModuleConfigs.kDriveCANbus, "FR");
    private final SwerveModule backLeftModule = new SwerveModule(SwerveModuleConfigs.kBackLeftConfig, SwerveModuleConfigs.kDriveCANbus, "BL");
    private final SwerveModule backRightModule = new SwerveModule(SwerveModuleConfigs.kBackRightConfig, SwerveModuleConfigs.kDriveCANbus, "BR");
    //Pigeon2 IMU
    private final Pigeon2 pigeon2 = new Pigeon2(Constants.kPigeon2, SwerveModuleConfigs.kDriveCANbus);
    //Odometry and field stuff
    private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(SwerveDriveKinematic.kDrivetrainKinematics, Rotation2d.fromDegrees(0), getModulePositions());
    private final Field2d field = new Field2d();

    public SwerveSubsystem()
    {
        //Create and run a new thread to reset the gyroscope.
        new Thread(() ->
        {
            try 
            {
                pigeon2.setYaw(0);
            }
            catch (Exception exception)
            {
                throw exception;
            }
        }).start();
        SmartDashboard.putData(field);
    }

    public void drive(ChassisSpeeds chassisSpeeds)
    {
        SwerveModuleState[] states = SwerveDriveKinematic.kDrivetrainKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states)
    {
        frontLeftModule.setTargetState(states[0]);
        SmartDashboard.putNumber("FL Target", states[0].angle.getRotations());
        frontRightModule.setTargetState(states[1]);
        SmartDashboard.putNumber("FR Target", states[1].angle.getRotations());
        backLeftModule.setTargetState(states[2]);
        SmartDashboard.putNumber("BL Target", states[2].angle.getRotations());
        backRightModule.setTargetState(states[3]);
        SmartDashboard.putNumber("BR Target", states[3].angle.getRotations());
    }
    
    public void stopModules()
    {
        frontLeftModule.stopMotors();
        frontRightModule.stopMotors();
        backLeftModule.stopMotors();
        backRightModule.stopMotors();
    }

    public double getYaw()
    {
        return pigeon2.getYaw().refresh().getValue();
    }

    public void zeroYaw()
    {
        pigeon2.setYaw(0);
    }

    public SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[]{
            frontLeftModule.getModulePosition(true),
            frontRightModule.getModulePosition(true),
            backLeftModule.getModulePosition(true),
            backRightModule.getModulePosition(true)
        };
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("FL Encoder", frontLeftModule.getAbsoluteEncoderRotations());
        SmartDashboard.putNumber("FR Encoder", frontRightModule.getAbsoluteEncoderRotations());
        SmartDashboard.putNumber("BL Encoder", backLeftModule.getAbsoluteEncoderRotations());
        SmartDashboard.putNumber("BR Encoder", backRightModule.getAbsoluteEncoderRotations());

        SmartDashboard.putNumber("Pigeon2 Yaw", getYaw());
    }
}
