package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// CHANGE CONVERSION FACTORS, MEASUREMENTS AND SPEEDS

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConsts{
    /* * * MEASUREMENTS * * */
    public static final double wheelDiameter = 4 * 2.5 / 100; // in meters
    public static final double gearRatio = 8.14 / 1;
    public static final double steerGearRatio = 150 / 7;
    public static final double trackWidth = 0.635;
    public static final double wheelBase = 0.635;
    public static final double voltage = 2.0;

    /* Swerve Drive Kinematics */
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      // front left
      new Translation2d(wheelBase / 2, trackWidth / 2),
      // back left
      new Translation2d(-wheelBase / 2, trackWidth / 2),
      // back right
      new Translation2d(-wheelBase / 2, -trackWidth / 2),
      // front right
      new Translation2d(wheelBase / 2, -trackWidth / 2)
    );


    /* * * FRONT LEFT * * */
    public static final int FL_driveMotorPort = 1;
    public static final int FL_turningMotorPort = 5;
    public static final int FL_absoluteEncoderPort = 9;
    public static final double FL_offset = -Math.toRadians(0);

    /* * * BACK LEFT * * */
    public static final int BL_driveMotorPort = 2;
    public static final int BL_turningMotorPort = 6;
    public static final int BL_absoluteEncoderPort = 10;
    public static final double BL_offset = -Math.toRadians(0);

    /* * * BACK RIGHT * * */
    public static final int BR_driveMotorPort = 3;
    public static final int BR_turningMotorPort = 7;
    public static final int BR_absoluteEncoderPort = 11;
    public static final double BR_offset = -Math.toRadians(0);

    /* * * FRONT RIGHT * * */
    public static final int FR_driveMotorPort = 4;
    public static final int FR_turningMotorPort = 8;
    public static final int FR_absoluteEncoderPort = 12;
    public static final double FR_offset = -Math.toRadians(0);

    /* * * CONVERSIONS TO METERS * * */
    public static final double driveEncoderRotationConversion = gearRatio * Math.PI * wheelDiameter;
    public static final double driveEncoderSpeedConversion = driveEncoderRotationConversion / 60;

    /* * * CONVERSIONS TO RADIANS * * */
    public static final double turningEncoderRotationConversion = steerGearRatio * Math.PI; //267475.198; //341.88 * Math.PI;
    public static final double turningEncoderSpeedConversion = turningEncoderRotationConversion / 60; //101604.912; //594.3893293799999;
      // 5676 / 60 * 341.88 * Math.PI

    /* * * SPEEDS * * */
    public static final double maxSpeed_mps = 4.1148;
      // 6380.0 / 60.0 * ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)) *
      // 0.10033 * Math.PI; // 13.5 feet per second = 4.1148 meters per second
    public static final double maxRotation = maxSpeed_mps / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    /* * * PID VALUES * * */
    public static final double kp = 0.2;
    public static final double ki = 0.004;
    public static final double kd = 0.0001;
  }

}
