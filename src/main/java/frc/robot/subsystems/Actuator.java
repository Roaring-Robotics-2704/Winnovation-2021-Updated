// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuator extends SubsystemBase {
    private Servo actuator = new Servo(0);

  /** Creates a new ExampleSubsystem. */
  public Actuator() {
      actuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void move(double speed) {
    actuator.setSpeed(speed);
  }
}
