// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Arm extends SubsystemBase {
  private WPI_VictorSPX armMotor = new WPI_VictorSPX(Constants.c_armMotor);
  public static Encoder armEncoder = new Encoder(Constants.c_armEncoderPorts[0], Constants.c_armEncoderPorts[1]);

  //private final ArmFeedforward armFeedForward = new ArmFeedforward(Constants.kStatic, Constants.kGravity, Constants.kVolts);
  private double setpoint = Constants.kMinAngle;

  public Arm() {/*
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));
    getController().setTolerance(Constants.kAngleTolerance);
    armEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    setSetpoint(setpoint); */
  }

  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } */

  public void move(double speed){
    armMotor.set(speed);
  }

  /*
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    armMotor.setVoltage(output + armFeedForward.calculate(setpoint, 0));
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return armMotor.get();
  }

  public void adjustPosition() {
    SmartDashboard.putNumber("Arm setpoint", setpoint);
  }

  public boolean atSetPoint() {
    return false;
  }

  public void stopFeeder() {
    armMotor.set(0);
  } */
}