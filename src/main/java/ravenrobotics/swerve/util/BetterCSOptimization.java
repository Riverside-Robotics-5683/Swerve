package ravenrobotics.swerve.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class BetterCSOptimization 
{
    /**
     * Optimize the target state for a shorter path. Takes into account that there is a closed-loop system for CTRE devices.
     * @param targetState The target state to reach.
     * @param currentAngle The curent angle of the robot.
     * @return The optimized target state.
     */
    public static SwerveModuleState optimize(SwerveModuleState targetState, Rotation2d currentAngle)
    {
        double targetAngle = placeIn0To360Scope(currentAngle.getDegrees(), targetState.angle.getDegrees());
        double targetSpeed = targetState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle.getDegrees();

        if(Math.abs(delta) > 90)
        {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }    

    /**
     * Places the given angle into the appropriate scope.
     * @param reference The reference for the scope.
     * @param newAngle The new angle to be placed in the scope.
     * @return The new angle.
     */
    private static double placeIn0To360Scope(double reference, double newAngle)
    {
        double lowerBound, upperBound;

        double lowerOffset = reference % 360;

        if (lowerOffset >= 0)
        {
            lowerBound = reference - lowerOffset;
            upperBound = reference + (360 - lowerOffset);
        }
        else
        {
            upperBound = reference - lowerOffset;
            lowerBound = reference - (360 + lowerOffset);
        }

        while (newAngle < lowerBound)
        {
            newAngle += 360;
        }
        while (newAngle > upperBound)
        {
            newAngle -= 360;
        }

        if (newAngle - reference > 180)
        {
            newAngle -= 360;
        }
        else if (newAngle - reference < -180)
        {
            newAngle += 360;
        }

        return newAngle;
    }
}
