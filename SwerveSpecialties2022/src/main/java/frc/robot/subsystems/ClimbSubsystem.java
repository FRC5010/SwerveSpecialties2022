// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRC5010.MotorController5010;
import frc.robot.FRC5010.MotorFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private CANSparkMax climbMotor;
  private RelativeEncoder climbEncoder;
  private double upperBound;
  private double power;
  public ClimbSubsystem(int motorID, double encoderUpperBound, double speed) {
    climbMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    climbMotor.setInverted(true);
    climbEncoder = climbMotor.getEncoder(Type.kHallSensor, 42);
    climbEncoder.setPosition(0);
    upperBound = encoderUpperBound;
    power = speed;
  }

  public void setMotorSpeed(double speed) {
    climbMotor.set(speed);
  }

  public void zeroArm(){
    climbEncoder.setPosition(0);
  }

  public double getUpperBound() {
    return upperBound;
  }

  public void climbStop(){
    climbMotor.set(0);
  }
  public void climbUp(){
    System.out.println(climbEncoder.getPosition());
    if(Math.abs(climbEncoder.getPosition()) < upperBound){
      climbMotor.set(power);
    }else{ 
      climbStop();
    }
  }
  public void climbDown(){
    System.out.println(climbEncoder.getPosition());
    if(climbEncoder.getPosition() > 0){
      climbMotor.set(-power);
    }else{ 
      climbStop();
    }
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb Encoder", this.getClimbEncoderPos());
  }
  public double getClimbEncoderPos(){
    
    return climbEncoder.getPosition();
  }
}
