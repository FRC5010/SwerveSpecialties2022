// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** Add your docs here. */
public class NEO extends CANSparkMax implements MotorController5010{
    public NEO(int port){
        super(port, MotorType.kBrushless);
        this.setSmartCurrentLimit(MotorConstants.CurrentLimits.Neo);
    }
    public NEO(int port, int currentLimit){
        super(port, MotorType.kBrushless);
        this.setSmartCurrentLimit(currentLimit);
    }
    @Override
    public MotorController5010 setSlewRate(double rate) {
        // TODO Auto-generated method stub
        this.setOpenLoopRampRate(rate);
        return this;
    }
    @Override
    public MotorController5010 setFollow(MotorController5010 motor) {
        // TODO Auto-generated method stub
        super.follow((CANSparkMax) motor.getMotor());
        return this;
    }
    @Override
    public MotorController5010 invert(boolean inverted) {
        // TODO Auto-generated method stub
        this.setInverted(inverted);
        return this;
    }
    @Override
    public RelativeEncoder getEncoder(){
        return this.getEncoder();
    }
    @Override
    public RelativeEncoder getEncoder(Type sensorType, int countsPerRev){
        return this.getEncoder(sensorType, countsPerRev);
    }
    @Override
    public MotorController getMotor() {
        // TODO Auto-generated method stub
        return this;
    }
    @Override
    public void factoryDefault() {
        // TODO Auto-generated method stub
        this.restoreFactoryDefaults();
    }
}
