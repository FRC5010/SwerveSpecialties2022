// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.76835;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.635;


    public static final double drivingP = 0.15315;

    public static final double turningP = 0.046496;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
    SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final double MAX_ACCELERATION_METERS_PER_SEC_PER_SEC = MAX_VELOCITY_METERS_PER_SECOND/1;
    public static final double MAX_ANGULAR_ACCELERTATION_RADIANS_PER_SEC_PER_SEC = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/1;


    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  public static final double ELEVATOR_MAX_RPM = 100;

  public static final double ELEVATOR_MAX_HEIGHT = 10.0;
  public static final double ELEVATOR_MIN_HEIGHT = 0.0;

  public static final double ELEVATOR_kP = 0.0;
  public static final double ELEVATOR_kI = 0.0;
  public static final double ELEVATOR_kD = 0.0;
  public static final double ELEVATOR_kS = 0.0;
  public static final double ELEVATOR_kV = 0.0;
  public static final double ELEVATOR_kA = 0.0;
  public static final double ELEVATOR_kG = 0.0;

  public static final double ELEVATOR_Min = 0.0;
  public static final double ELEVATOR_Max = 10.0;

  //public static final double ELEVATOR_FEEDFORWARD = 0.0;

  public static final double ELEVATOR_SPROKET_RADIUS = 1.0;
  

    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(9.67); //

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(275.19); //

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(180.35-180); //0

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(89.29); //

    public static final int LEFT_ELEVATOR_MOTOR = 13;
    public static final int RIGHT_ELEVATOR_MOTOR = 14;
}
