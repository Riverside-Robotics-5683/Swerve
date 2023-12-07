package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ravenrobotics.swerve.Constants;
import ravenrobotics.swerve.Constants.CanCoderConstants;
import ravenrobotics.swerve.Constants.MotorConstants;

public class DriveSubsystem extends SubsystemBase
{
    private final SwerveModule frontLeft = new SwerveModule(MotorConstants.kFrontLeftDrive, MotorConstants.kFrontLeftTurn, CanCoderConstants.kFrontLeftCanCoder, false, false, false);
    private final SwerveModule frontRight = new SwerveModule(MotorConstants.kFrontRightDrive, MotorConstants.kFrontRightTurn, CanCoderConstants.kFrontRightCanCoder, false, false, false);
    private final SwerveModule backLeft = new SwerveModule(MotorConstants.kBackLeftDrive, MotorConstants.kBackLeftTurn, CanCoderConstants.kBackLeftCanCoder, false, false, false);
    private final SwerveModule backRight = new SwerveModule(MotorConstants.kBackRightDrive, MotorConstants.kBackRightTurn, CanCoderConstants.kBackRightCanCoder, false, false, false);

    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final Pigeon2 pigeon2 = new Pigeon2(Constants.kPigeon2);

    public DriveSubsystem()
    {
        navx.calibrate();
        //pigeon2.setYaw(0); maybe?
    }

    public Rotation2d getYaw()
    {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    public void zeroYaw()
    {
        navx.zeroYaw();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
    }
}
