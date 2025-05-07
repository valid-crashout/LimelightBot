package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;

public class LimelightSubsystem
{
    public static double limelight_aim_proportional()
    {    
        double kP = -.005; //Lower? Invert? 
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight-tigears") * kP;
        targetingAngularVelocity *= SwerveConstants.k_maxAngularSpeedRadiansPS;
        targetingAngularVelocity *= -1.0; //Invert?
        return targetingAngularVelocity;
    }

    public static double limelight_range_proportional()
    {    
        double kP = .0009; //lower?
        double targetingForwardSpeed = (LimelightHelpers.getTA("limelight-tigears") != 0) ? Math.max(40 - LimelightHelpers.getTA("limelight-tigears"), 0) * kP : 0; //Look into TA
        targetingForwardSpeed *= SwerveConstants.k_maxWheelSpeedsMPS;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}