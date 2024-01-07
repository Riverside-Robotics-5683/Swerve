package ravenrobotics.swerve.util;

public class SwerveModuleConfig 
{
    public final int driveMotorID;
    public final boolean isDriveMotorInverted;

    public final int angleMotorID;
    public final boolean isAngleMotorInverted;

    public final int absoluteEncoderID;
    public final boolean isAbsoluteEncoderInverted;
    public final double absoluteEncoderOffset;

    public SwerveModuleConfig(int driveMotorID, boolean isDriveMotorInverted, int angleMotorID, boolean isAngleMotorInverted, int absoluteEncoderID, boolean isAbsoluteEncoderInverted, double absoluteEncoderOffset)
    {
        this.driveMotorID = driveMotorID;
        this.isDriveMotorInverted = isDriveMotorInverted;

        this.angleMotorID = angleMotorID;
        this.isAngleMotorInverted = isAngleMotorInverted;
        
        this.absoluteEncoderID = absoluteEncoderID;
        this.isAbsoluteEncoderInverted = isAbsoluteEncoderInverted;
        this.absoluteEncoderOffset = absoluteEncoderOffset;
    }    
}
