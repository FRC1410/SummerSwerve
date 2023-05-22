package org.frc1410.SummerSwerve;

import edu.wpi.first.wpilibj.RobotBase;

public interface Main {
  static void main(String[] args) {
	  RobotBase.startRobot(Robot::new);
  }
}
