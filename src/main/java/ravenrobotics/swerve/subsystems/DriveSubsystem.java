package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.swerve.Constants;
import ravenrobotics.swerve.Constants.CanCoderConstants;
import ravenrobotics.swerve.Constants.MotorConstants;
import ravenrobotics.swerve.Constants.SwerveConstants;

public class DriveSubsystem extends SubsystemBase
{
    private final SwerveModule frontLeft = new SwerveModule(MotorConstants.kFrontLeftDrive, MotorConstants.kFrontLeftTurn, CanCoderConstants.kFrontLeftCanCoder, false, false, false);
    private final SwerveModule frontRight = new SwerveModule(MotorConstants.kFrontRightDrive, MotorConstants.kFrontRightTurn, CanCoderConstants.kFrontRightCanCoder, false, false, false);
    private final SwerveModule backLeft = new SwerveModule(MotorConstants.kBackLeftDrive, MotorConstants.kBackLeftTurn, CanCoderConstants.kBackLeftCanCoder, false, false, false);
    private final SwerveModule backRight = new SwerveModule(MotorConstants.kBackRightDrive, MotorConstants.kBackRightTurn, CanCoderConstants.kBackRightCanCoder, false, false, false);

    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    //private final Pigeon2 pigeon2 = new Pigeon2(Constants.kPigeon2);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.kDrivetrainKinematics, Rotation2d.fromDegrees(0), getModulePositions());
    private final Field2d field = new Field2d();

    public DriveSubsystem()
    {
        new Thread(() ->
        {
            try
            {
                resetTurnMotorEncoders();
                navx.calibrate();
                //pigeon2.zeroGyroBiasNow();
            }
            catch (Exception e)
            {
                throw e;
            }
        }).start();
        SmartDashboard.putData(field);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kPhysialMaxSpeedMPS);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public void stopModules()
    {
        frontLeft.stopMotors();
        frontRight.stopMotors();
        backLeft.stopMotors();
        backRight.stopMotors();
    }

    public Rotation2d getYaw()
    {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    public void zeroYaw()
    {
        navx.zeroYaw();
    }

    public void resetTurnMotorEncoders()
    {
        frontLeft.resetTurnMotorPosition();
        frontRight.resetTurnMotorPosition();
        backLeft.resetTurnMotorPosition();
        backRight.resetTurnMotorPosition();
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        return modulePositions;
    }

    @Override
    public void periodic()
    {
        odometry.update(getYaw(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
    }
}
