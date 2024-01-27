package frc.robot;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.05;
    public static final boolean isRed = true;


    public static final class Swerve {

        // Spark Max Idle Modes
        public static final CANSparkMax.IdleMode driveIdleMode = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode angleIdleMode = CANSparkMax.IdleMode.kBrake;

        // Max Output Powers
        public static final double drivePower = .5;
        public static final double anglePower = .5;

        // Gyro
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final boolean canCoderInvert = true;
        public static final boolean driveMotorInvert = false;
        /* Motor Inverts */
        public static final boolean angleMotorInvert = true;
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
        // the number of degrees that a single rotation of the turn motor turns the wheel.
        public static final double DegreesPerTurnRotation = 360/angleGearRatio;
        /* Module Gear Ratios */
        public static final double driveGearRatio = (6.75 / 1.0);

        // encoder setup
        // meters per rotation
        public static final double wheelCircumference = 4 * Math.PI;
        public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio );
        public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5);
        public static final double wheelBase = Units.inchesToMeters(26.5);
        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 10;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0;
        public static final double angleKFF = 0;

        /* Drive Motor info  */
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;

        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * wheelCircumference)
                / driveGearRatio;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.04;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 1 / kDriveWheelFreeSpeedRps;
        /** Meters per Second */
        public static final double maxSpeed = 3.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 5.0;
        public static double angleRampRate = 0;

        /* CanCoder Constants */
        public static final CANCoderConfiguration swerveCANcoderConfig = new CANCoderConfiguration();

        public static class Modules {
            /* Module Specific Constants */
            /* Front Left Module - Module 0 */
            public static final class Mod0 {

                public static final int driveMotorID = 1;
                public static final int angleMotorID = 2;
                public static final int canCoderID = 9;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.95); //Rotation2d.fromDegrees(37.7);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1 {
                public static final int driveMotorID = 3;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 10;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(272.54);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 {
                public static final int driveMotorID = 5;
                public static final int angleMotorID = 6;
                public static final int canCoderID = 11;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(31.64);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 7;
                public static final int angleMotorID = 8;
                public static final int canCoderID = 12;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(356.92);
                public static final RevSwerveModuleConstants constants =
                    new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
        }
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}