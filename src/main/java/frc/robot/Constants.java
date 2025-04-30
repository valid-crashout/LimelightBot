// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants{
    
    //Update EVERYTHING and check
    public static final int k_frontLeftDriveID = 4;
    public static final int k_frontLeftTurnID = 3;
    public static final int k_frontLeftAbsID = 9;
    public static final boolean k_frontLeftInverted = true;
    public static final boolean k_frontLeftAbsInverted = false;
    public static final double k_frontLeftChassisOffset = 0.0; //help
    public static final double k_frontLeftAbsOffset = 0.0 * 2 * Math.PI;

    public static final int k_frontRightDriveID = 6;
    public static final int k_frontRightTurnID = 5;
    public static final int k_frontRightAbsID = 10;
    public static final boolean k_frontRightInverted = false;
    public static final boolean k_frontRightAbsInverted = false;
    public static final double k_frontRightChassisOffset = 0.0; //help
    public static final double k_frontRightAbsOffset = 0.0 * 2 * Math.PI;

    public static final int k_backLeftDriveID = 2;
    public static final int k_backLeftTurnID = 1;
    public static final int k_backLeftAbsID = 12;
    public static final boolean k_backLeftInverted = true;
    public static final boolean k_backLeftAbsInverted = false;
    public static final double k_backLeftChassisOffset = 0.0; //help
    public static final double k_backLeftAbsOffset = 0.0 * 2 * Math.PI;

    public static final int k_backRightDriveID = 8;
    public static final int k_backRightTurnID = 7;
    public static final int k_backRightAbsID = 11;
    public static final boolean k_backRightInverted = false;
    public static final boolean k_backRightAbsInverted = false;
    public static final double k_backRightChassisOffset = 0.0; //help
    public static final double k_backRightAbsOffset = 0.000244 * 2 * Math.PI;

    public enum MotorLocation {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    };

    public static final double k_trackWidth = Units.inchesToMeters(19);
    public static final double k_wheelBase = Units.inchesToMeters(19);
    public static final SwerveDriveKinematics k_driveKinematics = new SwerveDriveKinematics(
      new Translation2d(k_wheelBase / 2, -k_trackWidth / 2), //front right
      new Translation2d(k_wheelBase / 2, k_trackWidth / 2),  //front left
      new Translation2d(k_wheelBase / 2, -k_trackWidth / 2), //back right
      new Translation2d(k_wheelBase / 2, k_trackWidth / 2)   //back left
    );

    public static final double k_maxWheelSpeedsMPS = 10;
    public static final double k_maxAngularSpeedRadiansPS = 3 * 2 * Math.PI * 0.25;
    public static final double k_maxAccelerationMPSSquared = 3;
    public static final double k_maxAngularAccelerationMPSSquared = 3;
  }
  public static class OperatorConstants {
    public static final int k_driverControllerPort = 0;
    public static final int k_driveYInverted = 1;
    public static final int k_driveXInverted = -1;
    public static final int k_driveZInverted = 1;
    public static final double k_deadband = 0.3;
  }
}
