package ravenrobotics.swerve;

public class Constants
{
    public static class DSConstants
    {
        public static final int DRIVER_PORT = 0;
    }

    public static class MotorConstants
    {
        public static final int kFrontLeftDrive = 0;
        public static final int kFrontLeftTurn = 1;

        public static final int kFrontRightDrive = 2;
        public static final int kFrontRightTurn = 3;

        public static final int kBackLeftDrive = 4;
        public static final int kBackLeftTurn = 5;

        public static final int kBackRightDrive = 6;
        public static final int kBackRightTurn = 7;
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
    }

    public static final int kPigeon2 = 12;
}
