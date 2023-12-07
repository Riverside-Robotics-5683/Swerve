package ravenrobotics.swerve.util;

import ravenrobotics.swerve.Constants;

public class CTREConversions 
{
    public static double degreesToTalonFX(double angle)
    {
        return angle / (360.0 / (Constants.SwerveConstants.kGearRatio * 2048.0));
    }

    public static double talonFXToDegrees(double encoderCount)
    {
        return encoderCount * (360.0 /  (Constants.SwerveConstants.kGearRatio * 2048.0));
    }
    
    public static double canCodertoTalonFX(double encoderCount)
    {
        return encoderCount / 2.0;
    }
}