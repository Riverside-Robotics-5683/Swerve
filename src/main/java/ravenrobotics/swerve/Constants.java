package ravenrobotics.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import ravenrobotics.swerve.util.SwerveModuleConfig;

public class Constants 
{
    public class DriverStationConstants
    {
        public static final int kDriverPort = 0;
    }

    public class SwerveModuleConstants
    {
        /////////////////////////
        ///////Angle Motor///////
        /////////////////////////
        //PID Values
        public static final double kAngleP = 1.0;
        public static final double kAngleI = 0.0;
        public static final double kAngleD = 0.0;
        //Current Limits
        public static final int kAngleContinuousCurrentLimit = 10;
        public static final int kAnglePeakCurrentLimit = 11;
        public static final double kAnglePeakCurrentDuration = 0.1;
        //Gear Ratio
        public static final double kAngleGearRatio = 12.8;

        /////////////////////////
        ///////Drive Motor///////
        /////////////////////////
        //PID Values
        public static final double kDriveP = 2.0;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        //Feedforward Values
        public static final double kDriveS = 0.2;
        public static final double kDriveV = 2.0;
        //Current Limits
        public static final int kDriveContinuousCurrentLimit = 35;
        public static final int kDrivePeakCurrentLimit = 39;
        public static final double kDrivePeakCurrentDuration = 0.1;
        //Gear Ratio
        public static final double kDriveGearRatio = 6.75;

        /////////////////////////
        ///////Other Stuff///////
        /////////////////////////
        public static final double kMaxSpeedMPS = 16.7;

    }
    
    public static class SwerveModuleConfigs
    {
        public static final String kDriveCANbus = "rio";
        public static final SwerveModuleConfig kFrontLeftConfig = new SwerveModuleConfig(5, false, 0, false, 11, false, 0.390380859375);
        public static final SwerveModuleConfig kFrontRightConfig = new SwerveModuleConfig(2, false, 6, true, 10, false, 0.195068359375);
        public static final SwerveModuleConfig kBackLeftConfig = new SwerveModuleConfig(7, false, 3, false, 8, false, 0.861083984375);
        public static final SwerveModuleConfig kBackRightConfig = new SwerveModuleConfig(4, false, 1, false, 9, false, 0.15087890625);
    }

    public static final int kPigeon2 = 12;

    public static class SwerveDriveKinematic
    {
        //Measurements for wheels.
        public static final double kTrackWidth = 0.6604;
        public static final double kWheelBase = 0.6604;
        public static final double kWheelXOffset = kTrackWidth / 2;
        public static final double kWheelYOffset = kWheelBase / 2;

        //Kinematics
        public static final Translation2d kFrontLeftCenter = new Translation2d(kWheelXOffset, kWheelYOffset);
        public static final Translation2d kFrontRightCenter = new Translation2d(kWheelXOffset, -kWheelYOffset);
        public static final Translation2d kBackLeftCenter = new Translation2d(-kWheelXOffset, kWheelYOffset);
        public static final Translation2d kBackRightCenter = new Translation2d(-kWheelYOffset, -kWheelYOffset);
        public static final SwerveDriveKinematics kDrivetrainKinematics = new SwerveDriveKinematics(
            kFrontLeftCenter,
            kFrontRightCenter,
            kBackLeftCenter,
            kBackRightCenter);
        
    }
}
