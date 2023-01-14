// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private CANSparkMax leftElevator;
  private CANSparkMax rightElevator;
  private double upperBound;
  private RelativeEncoder elevatorEncoder;
  private double power;
  
  public ElevatorSubsystem(int leftMotorID, int rightMotorID, double encoderUpperBound, double power) {
    leftElevator = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    rightElevator = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    this.power = power;
    upperBound = encoderUpperBound;
    elevatorEncoder = leftElevator.getEncoder(Type.kHallSensor, 42);
    leftElevator.setInverted(false);
    rightElevator.setInverted(true);

    rightElevator.follow(leftElevator);
  }

  public void setMotorSpeed(double speed) {
    leftElevator.set(speed);
  }
  public void elevatorStop(){
    leftElevator.set(0);
  }
  public void zeroElevator(){
    elevatorEncoder.setPosition(0);
  }
  public double getUpperBound(){
    return upperBound;
  }

  public void elevatorUp(){
    if(Math.abs(elevatorEncoder.getPosition()) < upperBound){
      setMotorSpeed(power);
    }else{
      elevatorStop();
    }
  }
  public void elevatorDown(){
    if(Math.abs(elevatorEncoder.getPosition()) > 0){
      setMotorSpeed(-power);
    }else{
      elevatorStop();
    }
  }

  public double getElevatorEncoderPos(){
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", this.getElevatorEncoderPos());
    // This method will be called once per scheduler run
  }
}
