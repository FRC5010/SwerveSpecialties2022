// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMaxPIDController PIDController;
  private RelativeEncoder encoder;

  private CANSparkMax motorLeft;
  private CANSparkMax motorRight;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(CANSparkMax motorLeft, CANSparkMax motorRight) {
    this.motorLeft = motorLeft;
    this.motorRight = motorRight;
    
    this.motorLeft.setInverted(false);
    this.motorRight.setInverted(true);

    this.motorRight.follow(this.motorLeft);

    PIDController = motorLeft.getPIDController();
    PIDController.setP(Constants.ELEVATOR_kP);
    PIDController.setI(Constants.ELEVATOR_kI);
    PIDController.setD(Constants.ELEVATOR_kD);

    PIDController.setOutputRange(Constants.ELEVATOR_Min, Constants.ELEVATOR_Max);

    encoder = motorLeft.getEncoder();
    encoder.setPosition(0.0);

  }

  public double getElevatorHeight() {
    double circumference = Constants.ELEVATOR_SPROKET_RADIUS*2*Math.PI;
    return circumference*encoder.getPosition();
  }

  public void setTargetPosition(double point) { // Sets Target to Position in _
    PIDController.setReference(point, CANSparkMax.ControlType.kPosition);
  }

  public void setTargetVelocity(double point) { // Sets Target to Velocity in RPM
    PIDController.setReference(point, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
