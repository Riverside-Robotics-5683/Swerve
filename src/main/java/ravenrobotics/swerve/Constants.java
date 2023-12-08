package ravenrobotics.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import ravenrobotics.swerve.util.DistanceConversions;

public class Constants
{
    public static class DSConstants
    {
        public static final int DRIVER_PORT = 0;
    }

    public static class MotorConstants
    {
        public static final int kFrontLeftDrive = 5;
        public static final int kFrontLeftTurn = 0;

        public static final int kFrontRightDrive = 2;
        public static final int kFrontRightTurn = 6;

        public static final int kBackLeftDrive = 7;
        public static final int kBackLeftTurn = 3;

        public static final int kBackRightDrive = 4;
        public static final int kBackRightTurn = 1;
    }

    public static class CanCoderConstants
    {
        public static final int kFrontLeftCanCoder = 8;
        public static final int kFrontRightCanCoder = 9;
        public static final int kBackLeftCanCoder = 10;
        public static final int kBackRightCanCoder = 11;
    }

    public static class SwerveConstants
    {
        public static final double kGearRatio = 6.75;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kPhysialMaxSpeedMPS = 16.5;

        public static final double kTrackWidth = DistanceConversions.inchesToMeters(26);
        public static final double kWheelBase = DistanceConversions.inchesToMeters(28);
        public static final double kWheelXOffset = kTrackWidth / 2;
        public static final double kWheelYOffset = kWheelBase / 2;

        public static final Translation2d kFrontLeftModuleCenter = new Translation2d(kWheelYOffset, -kWheelXOffset);
        public static final Translation2d kFrontRightModuleCenter = new Translation2d(kWheelYOffset, kWheelXOffset);
        public static final Translation2d kBackLeftModuleCenter = new Translation2d(-kWheelYOffset, -kWheelXOffset);
        public static final Translation2d kBackRightModuleCenter = new Translation2d(-kWheelYOffset, kWheelXOffset);

        public static final SwerveDriveKinematics kDrivetrainKinematics = new SwerveDriveKinematics(
            kFrontLeftModuleCenter,
            kFrontRightModuleCenter,
            kBackLeftModuleCenter,
            kBackRightModuleCenter);
    }

    public static final int kPigeon2 = 12;
}
