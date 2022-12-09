// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorVelocityCommand extends CommandBase {
  private XboxController coDriver;
  private ElevatorSubsystem elevatorSubsystem;
  /** Creates a new ElevatorVelocityCommand. */
  public ElevatorVelocityCommand(ElevatorSubsystem elevatorSubsystem, XboxController coDriver) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.coDriver = coDriver;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftVelocity = coDriver.getLeftY();
    elevatorSubsystem.setTargetVelocity(leftVelocity*Constants.ELEVATOR_MAX_RPM);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
