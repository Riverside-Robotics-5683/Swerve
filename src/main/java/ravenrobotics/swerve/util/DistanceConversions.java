package ravenrobotics.swerve.util;

public class DistanceConversions 
{
    public static double inchesToMeters(double inches)
    {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters)
    {
        return meters / 0.0254;
    }
}
