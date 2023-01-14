// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  
  public static final double MAX_VOLTAGE = 12.0;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        
  SwerveDriveKinematics m_kinematics = Constants.m_kinematics;

  private final Pigeon2 m_pigeon;
  private SwerveModulePosition[] modulePositions; 

//         SwerveDriveKinematics kinematics,
//       Rotation2d gyroAngle,
//       SwerveModulePosition[] modulePositions
  
  
  private SwerveDriveOdometry odometer;
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem(Pigeon2 m_pigeon) {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    this.m_pigeon = m_pigeon;
            
        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            
            Mk4iSwerveModuleHelper.GearRatio.L1,
            
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            
            FRONT_LEFT_MODULE_STEER_MOTOR,
            
            FRONT_LEFT_MODULE_STEER_ENCODER,
            
            FRONT_LEFT_MODULE_STEER_OFFSET
    );
        
        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    //m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    //m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    modulePositions  = new SwerveModulePosition[] {
        new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.m_frontLeftModule.getSteerAngle()))), 
        new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.m_frontRightModule.getSteerAngle()))), 
        new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.m_backLeftModule.getSteerAngle()))), 
        new SwerveModulePosition(0, new Rotation2d(Units.radiansToDegrees(this.m_backRightModule.getSteerAngle())))
      };

      this.odometer = new SwerveDriveOdometry(Constants.m_kinematics, new Rotation2d(0), modulePositions);
  }

  public void zeroGyroscope() {
    m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public Pose2d getPose2d(){
          return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
        odometer.resetPosition(new Rotation2d(), this.modulePositions, pose);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  public void setModuleStates(SwerveModuleState[] setDesiredStates){
        SwerveModuleState[] states = setDesiredStates;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        
        if(states[0].speedMetersPerSecond > 0.0001){
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }else{
                m_frontLeftModule.set(0, Math.toRadians(45*5));
                m_frontRightModule.set(0, Math.toRadians(45*7));
                m_backLeftModule.set(0, Math.toRadians(45*3));
                m_backRightModule.set(0, Math.toRadians(45));              
        }
  }


  @Override
  public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(states);

        odometer.update(getGyroscopeRotation(), this.modulePositions);
  }
}
