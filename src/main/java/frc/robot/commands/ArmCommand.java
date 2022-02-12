// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;
//import frc.robot.Robot; 
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  
  double setpoint = 0;
  double armSpeed;
  
  PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  //SmartDashboard.putNumber("Arm Encoder Get Distance", Arm.armEncoder.getDistance());

  public ArmCommand() {
    //pid.disableContinuousInput();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = 5;
    //Arm.armEncoder.reset();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Boolean button_up = RobotContainer.m_setPointUp.get();
    Boolean button_down = RobotContainer.m_setPointDown.get();
    
    SmartDashboard.putBoolean("button_up", button_up);
    SmartDashboard.putBoolean("button_down", button_down);
    System.out.println("up: " + button_up.toString() + "down: " + button_down.toString());
    if(button_up){
      setpoint = setpoint + 5/50;
    }
    if(button_down){
      setpoint = setpoint - 5/50;
    }

    setpoint = MathUtil.clamp(setpoint, 5, 20);

    //double leftInput = RobotContainer.m_xbox.getRawAxis(Constants.c_leftTriggerAxis);
    //double rightInput = RobotContainer.m_xbox.getRawAxis(Constants.c_rightTriggerAxis);
    //double sumInput = -leftInput + rightInput;

    //  if ((Math.abs(Arm.armEncoder.getDistance()) > 0 && Math.abs(Arm.armEncoder.getDistance()) <= 5)
    //   || (Math.abs(Arm.armEncoder.getDistance()) >= 18 && Math.abs(Arm.armEncoder.getDistance()) < 20)) {
    //armSpeed = (Constants.c_armSpeed/5) * sumInput;
    armSpeed = pid.calculate(Arm.armEncoder.getDistance(), setpoint);
    SmartDashboard.putNumber("setpoint", setpoint);
    
    //  } else {
    //    armSpeed = (Constants.c_armSpeed/2) * sumInput; 
    //  } 
    RobotContainer.m_arm.move(-armSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !RobotContainer.m_armDownButton.get() || 
    //Math.abs(Arm.armEncoder.getDistance()) <=0 || Math.abs(Arm.armEncoder.getDistance()) >= 1000;
     
    return false;
  }
}
