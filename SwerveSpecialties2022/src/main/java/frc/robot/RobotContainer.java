// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driver = new XboxController(0);

  private final PigeonIMU pigeonGyro = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(pigeonGyro);

  private SendableChooser<PathPlannerTrajectory> autoPicker = new SendableChooser<>();

  public RobotContainer() {
    autoPicker.addOption("DriveSquareAndTurn", getPathFromPathPlanner("DriveSquareAndTurn"));
    autoPicker.addOption("DriveLine", getPathFromPathPlanner("DriveLine"));

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            drivetrainSubsystem,
            () -> -modifyAxis(driver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driver.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new Button(driver::getBButton)
            .whenPressed(drivetrainSubsystem::zeroGyroscope);
  }

  public PathPlannerTrajectory getPathFromPathPlanner(String name){
    return PathPlanner.loadPath(name, Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  }

  public Command getAutonomousCommand() {
    Trajectory pickedPath = autoPicker.getSelected();

    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY_METERS_PER_SECOND, 3*Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand autoCommand = new SwerveControllerCommand(
      pickedPath, drivetrainSubsystem::getPose2d, Constants.m_kinematics, xController, yController, thetaController, drivetrainSubsystem::setModuleStates, drivetrainSubsystem);

    return autoCommand;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.085);
    value = Math.copySign(value * value, value);

    return value;
  }
}
