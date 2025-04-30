package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.MotorLocation;

public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule m_frontLeft = new SwerveModule(
        SwerveConstants.k_frontLeftDriveID,
        SwerveConstants.k_frontLeftTurnID,
        SwerveConstants.k_frontLeftAbsID,
        SwerveConstants.k_frontLeftChassisOffset,
        SwerveConstants.k_frontLeftInverted,
        SwerveConstants.k_frontLeftAbsInverted,
        SwerveConstants.k_frontLeftAbsOffset,
        Configs.SwerveModule.frontLeftDrivingConfig,
        Configs.SwerveModule.frontLeftTurningConfig,
        MotorLocation.FRONT_LEFT);

    private final SwerveModule m_frontRight = new SwerveModule(
        SwerveConstants.k_frontRightDriveID,
        SwerveConstants.k_frontRightTurnID,
        SwerveConstants.k_frontRightAbsID,
        SwerveConstants.k_frontRightChassisOffset,
        SwerveConstants.k_frontRightInverted,
        SwerveConstants.k_frontRightAbsInverted,
        SwerveConstants.k_frontRightAbsOffset,
        Configs.SwerveModule.frontRightDrivingConfig,
        Configs.SwerveModule.frontRightTurningConfig,
        MotorLocation.FRONT_RIGHT);

    private final SwerveModule m_backLeft = new SwerveModule(
        SwerveConstants.k_backLeftDriveID,
        SwerveConstants.k_backLeftTurnID,
        SwerveConstants.k_backLeftAbsID,
        SwerveConstants.k_backLeftChassisOffset,
        SwerveConstants.k_backLeftInverted,
        SwerveConstants.k_backLeftAbsInverted,
        SwerveConstants.k_backLeftAbsOffset,
        Configs.SwerveModule.backLeftDrivingConfig,
        Configs.SwerveModule.backLeftTurningConfig,
        MotorLocation.BACK_LEFT);

    private final SwerveModule m_backRight = new SwerveModule(
        SwerveConstants.k_backRightDriveID,
        SwerveConstants.k_backRightTurnID,
        SwerveConstants.k_backRightAbsID,
        SwerveConstants.k_backRightChassisOffset,
        SwerveConstants.k_backRightInverted,
        SwerveConstants.k_backRightAbsInverted,
        SwerveConstants.k_backRightAbsOffset,
        Configs.SwerveModule.backRightDrivingConfig,
        Configs.SwerveModule.backRightTurningConfig,
        MotorLocation.BACK_RIGHT);

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry m_odometer = new SwerveDriveOdometry(SwerveConstants.k_driveKinematics,
                                                   new Rotation2d(0), getSwerveModulePositions());

    private SwerveModuleState[] m_desiredModuleStates;

    public SwerveSubsystem() {
        m_desiredModuleStates = new SwerveModuleState[] {new SwerveModuleState(), new SwerveModuleState(), 
                                                         new SwerveModuleState(), new SwerveModuleState()};
        
        //delay for gyro boot
        new Thread(() -> {
            try {
                Thread.sleep(1500);
                zeroHeading();
                resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
            } catch (Exception e) {}
        }).start();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), 
                                           m_backLeft.getPosition(), m_backRight.getPosition()};
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {m_frontLeft.getState(), m_frontRight.getState(), 
                                           m_backLeft.getState(), m_backRight.getState()};
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.k_driveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return m_odometer.getPoseMeters();
    }
    
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public DoubleSupplier[] getWheelRotationSupplier() {
        return new DoubleSupplier[] {
            () -> m_frontLeft.getTurnPosition(),
            () -> m_frontRight.getTurnPosition(),
            () -> m_backLeft.getTurnPosition(),
            () -> m_backRight.getTurnPosition()
        };
    }

    public void setModuleStates(ChassisSpeeds p_chassisSpeeds) {
        setModuleStates(SwerveConstants.k_driveKinematics.toSwerveModuleStates(p_chassisSpeeds));
    }

    public void setModuleStates(SwerveModuleState[] p_desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(p_desiredStates, SwerveConstants.k_maxWheelSpeedsMPS);
        m_frontLeft.setDesiredState(p_desiredStates[0]);
        m_frontRight.setDesiredState(p_desiredStates[1]);
        m_backLeft.setDesiredState(p_desiredStates[2]);
        m_backRight.setDesiredState(p_desiredStates[3]);
        m_desiredModuleStates = p_desiredStates;
    }

    public void resetOdometry(Pose2d p_pose) {
        m_odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), p_pose);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }
}

