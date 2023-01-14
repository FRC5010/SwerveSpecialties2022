// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class CalibrateArm extends CommandBase {
  /** Creates a new CalibrateArm. */
  double armPosition = 0;
  ClimbSubsystem climbSubsystem;
  double startPos;
  public CalibrateArm(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.setMotorSpeed(-0.3);
    startPos = climbSubsystem.getClimbEncoderPos();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.climbStop();
    climbSubsystem.zeroArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbSubsystem.getClimbEncoderPos() - startPos < -climbSubsystem.getUpperBound();
  }
}
