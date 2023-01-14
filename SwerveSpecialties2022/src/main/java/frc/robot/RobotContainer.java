// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driver = new XboxController(0);
  //private final XboxController coDriver = new XboxController(1);


  private final Pigeon2 pigeonGyro = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(pigeonGyro);
  //private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(13,185,.65);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(13, 14,  31, .95);
  private SendableChooser<PathPlannerTrajectory> autoPicker = new SendableChooser<>();

  public RobotContainer() {
    //autoPicker.addOption("DriveSquareAndTurn", getPathFromPathPlanner("DriveSquareAndTurn"));
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
    new Button(driver::getAButton)
            .whenPressed(drivetrainSubsystem::zeroGyroscope);
    // TODO StartEndCommand does not work how we want it to work, need to change to a different system

    /* ClimbSubsystem currently not supported, physically removed
    
    new Button(driver::getXButton)
            .whileHeld(new RunCommand(() -> climbSubsystem.climbDown(), climbSubsystem))
            .whenReleased(new InstantCommand(() -> climbSubsystem.climbStop(), climbSubsystem));
    new Button(driver::getYButton)
            .whileHeld(new RunCommand(() -> climbSubsystem.climbUp(), climbSubsystem))
            .whenReleased(new InstantCommand(() -> climbSubsystem.climbStop(), climbSubsystem));
    SmartDashboard.putData("Calibrate Arm", new CalibrateArm(climbSubsystem));    
    */
    //ElevatorSubsystem will take over when we are not using climb
    new Button(driver::getXButton)
            .whileHeld(new RunCommand(() -> elevatorSubsystem.elevatorDown(), elevatorSubsystem))
            .whenReleased(new InstantCommand(() -> elevatorSubsystem.elevatorStop(), elevatorSubsystem));
    new Button(driver::getYButton)
            .whileHeld(new RunCommand(() -> elevatorSubsystem.elevatorUp(), elevatorSubsystem))
            .whenReleased(new InstantCommand(() -> elevatorSubsystem.elevatorStop(), elevatorSubsystem));
  }

  public PathPlannerTrajectory getPathFromPathPlanner(String name){
    return PathPlanner.loadPath(name, Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  }

  public Command getAutonomousCommand() {
    Trajectory pickedPath = getPathFromPathPlanner("DriveLine");//autoPicker.getSelected();

    PIDController xController = new PIDController(Constants.drivingP, 0, 0);
    PIDController yController = new PIDController(Constants.drivingP, 0, 0);

    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.turningP, 0, 0, new TrapezoidProfile.Constraints(Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
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
