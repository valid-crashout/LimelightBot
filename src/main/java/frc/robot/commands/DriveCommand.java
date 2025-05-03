package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, zSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, zLimiter;
  
  public DriveCommand(SwerveSubsystem p_swerveSub, Supplier<Double> p_xSpdFunction, Supplier<Double> p_ySpdFunction, 
                      Supplier<Double> p_zSpdFunction, Supplier<Boolean> p_fieldOrientedFunction, Supplier<Boolean> p_limelight) {
    m_swerveSubsystem = p_swerveSub;
    xSpdFunction = (p_limelight.get()) ? () -> LimelightSubsystem.limelight_range_proportional() : p_xSpdFunction;
    ySpdFunction = p_ySpdFunction;
    zSpdFunction = (p_limelight.get()) ? () -> LimelightSubsystem.limelight_aim_proportional() : p_zSpdFunction;
    fieldOrientedFunction = (p_limelight.get()) ? () -> false : p_fieldOrientedFunction;
    xLimiter = new SlewRateLimiter(SwerveConstants.k_maxAccelerationMPSSquared);
    yLimiter = new SlewRateLimiter(SwerveConstants.k_maxAccelerationMPSSquared);
    zLimiter = new SlewRateLimiter(SwerveConstants.k_maxAngularAccelerationMPSSquared);
    addRequirements(p_swerveSub);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    double xSpeed = OperatorConstants.k_driveXInverted * xSpdFunction.get();
    double ySpeed = OperatorConstants.k_driveYInverted * ySpdFunction.get();
    double zSpeed = OperatorConstants.k_driveZInverted * zSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > OperatorConstants.k_deadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.k_deadband ? ySpeed : 0;
    zSpeed = Math.abs(zSpeed) > OperatorConstants.k_deadband ? zSpeed : 0;

    xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.k_maxWheelSpeedsMPS * 0.5;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveConstants.k_maxWheelSpeedsMPS * 0.5;
    zSpeed = zLimiter.calculate(zSpeed) * SwerveConstants.k_maxAngularSpeedRadiansPS;

    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, m_swerveSubsystem.getRotation2d());
    }
    else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, zSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.k_driveKinematics.toSwerveModuleStates(chassisSpeeds);
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
