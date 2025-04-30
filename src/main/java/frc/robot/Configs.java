package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Configs {
    public static final class SwerveModule {
        public static final SparkMaxConfig frontRightDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backLeftDrivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backRightDrivingConfig = new SparkMaxConfig();

        public static final SparkMaxConfig frontRightTurningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig frontLeftTurningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backLeftTurningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig backRightTurningConfig = new SparkMaxConfig();
    

        static {
            double d_drivingFactor = 0; //TODO
            double d_turningFactor = 2 * Math.PI;
            double d_drivingVelocityFF = 0; //TODO

            frontRightDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false); //check
            frontRightDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor)
                .velocityConversionFactor(d_drivingFactor / 60.0);
            frontRightDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0,0,0) //check
                .velocityFF(d_drivingVelocityFF)
                .outputRange(-1, 1);

            frontLeftDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false); //check
            frontLeftDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor)
                .velocityConversionFactor(d_drivingFactor / 60.0);
            frontLeftDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0,0,0) //check
                .velocityFF(d_drivingVelocityFF)
                .outputRange(-1, 1);

            backRightDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false); //check
            backRightDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor)
                .velocityConversionFactor(d_drivingFactor / 60.0);
            backRightDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0,0,0) //check
                .velocityFF(d_drivingVelocityFF)
                .outputRange(-1, 1);
            
            backLeftDrivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50)
                .inverted(false); //check
            backLeftDrivingConfig.encoder
                .positionConversionFactor(d_drivingFactor)
                .velocityConversionFactor(d_drivingFactor / 60.0);
            backLeftDrivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0,0,0) //check
                .velocityFF(d_drivingVelocityFF)
                .outputRange(-1, 1);


            frontRightTurningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            frontRightTurningConfig.encoder // check all
                .inverted(true) 
                .positionConversionFactor(d_turningFactor)
                .velocityConversionFactor(d_turningFactor / 60.0);
            frontRightTurningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // check
                .pid(0,0,0) // check
                .outputRange(-1, 1)
                //check
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);
            
            frontLeftTurningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            frontLeftTurningConfig.encoder // check all
                .inverted(true) 
                .positionConversionFactor(d_turningFactor)
                .velocityConversionFactor(d_turningFactor / 60.0);
            frontLeftTurningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // check
                .pid(0,0,0) // check
                .outputRange(-1, 1)
                //check
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);

            backRightTurningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            backRightTurningConfig.encoder // check all
                .inverted(true) 
                .positionConversionFactor(d_turningFactor)
                .velocityConversionFactor(d_turningFactor / 60.0);
            backRightTurningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // check
                .pid(0,0,0) // check
                .outputRange(-1, 1)
                //check
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);

            backLeftTurningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            backLeftTurningConfig.encoder // check all
                .inverted(true) 
                .positionConversionFactor(d_turningFactor)
                .velocityConversionFactor(d_turningFactor / 60.0);
            backLeftTurningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // check
                .pid(0,0,0) // check
                .outputRange(-1, 1)
                //check
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, d_turningFactor);
        }
    }
}
