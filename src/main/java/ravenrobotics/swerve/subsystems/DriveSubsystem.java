package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
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
    private final SwerveModule frontLeft = new SwerveModule(MotorConstants.kFrontLeftDrive, MotorConstants.kFrontLeftTurn, CanCoderConstants.kFrontLeftCanCoder, false, false, SwerveConstants.frontLeftStart);
    private final SwerveModule frontRight = new SwerveModule(MotorConstants.kFrontRightDrive, MotorConstants.kFrontRightTurn, CanCoderConstants.kFrontRightCanCoder, false, false, SwerveConstants.frontRightStart);
    private final SwerveModule backLeft = new SwerveModule(MotorConstants.kBackLeftDrive, MotorConstants.kBackLeftTurn, CanCoderConstants.kBackLeftCanCoder, false, false, SwerveConstants.backLeftStart);
    private final SwerveModule backRight = new SwerveModule(MotorConstants.kBackRightDrive, MotorConstants.kBackRightTurn, CanCoderConstants.kBackRightCanCoder, false, false, SwerveConstants.backRightStart);

    //private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon2 = new Pigeon2(12, "rio");

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(SwerveConstants.kDrivetrainKinematics, Rotation2d.fromDegrees(0), getModulePositions());
    private final Field2d field = new Field2d();

    public DriveSubsystem()
    {
        new Thread(() ->
        {
            try
            {
                resetTurnMotorEncoders();
                //navx.calibrate();
                pigeon2.setYaw(0);
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
        return Rotation2d.fromDegrees(pigeon2.getYaw().getValue());
    }

    public void zeroYaw()
    {
        pigeon2.setYaw(0);
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

    public void updatePIDs(double p, double i, double d)
    {
        frontLeft.setPIDController(p, i, d);
        frontRight.setPIDController(p, i, d);
        backLeft.setPIDController(p, i, d);
        backRight.setPIDController(p, i, d);
    }

    public void zeroCancoders()
    {
        frontLeft.resetCanCoder();
        frontRight.resetCanCoder();
        backLeft.resetCanCoder();
        backRight.resetCanCoder();
    }

    @Override
    public void periodic()
    {
        odometry.update(getYaw(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putNumber("Pigeon2 Yaw", pigeon2.getYaw().getValue());

        SmartDashboard.putNumber("P", 4);
        SmartDashboard.putNumber("I", 0);
        SmartDashboard.putNumber("D", 0);

        SmartDashboard.putNumber("FL CANcoder", frontLeft.getCanCoderPosition());
        SmartDashboard.putNumber("FR CANcoder", frontRight.getCanCoderPosition());
        SmartDashboard.putNumber("BL CANcoder", backLeft.getCanCoderPosition());
        SmartDashboard.putNumber("BR CANcoder", backRight.getCanCoderPosition());

        SmartDashboard.putNumber("FL Turn Value", frontLeft.getTurnMotorPosition());
        SmartDashboard.putNumber("FR Turn Value", frontRight.getTurnMotorPosition());
        SmartDashboard.putNumber("BL Turn Value", backLeft.getTurnMotorPosition());
        SmartDashboard.putNumber("BR Turn Value", backRight.getTurnMotorPosition());
    }
}
