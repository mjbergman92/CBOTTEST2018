package org.usfirst.frc3534.RobotBasic.systems;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SystemBase implements SystemInterface {

	private SpeedControllerGroup rightSide = RobotMap.rightSideMotors;
	private SpeedControllerGroup leftSide = RobotMap.leftSideMotors;
	private DifferentialDrive drive;

	private double rightPower, leftPower;

	public Drive() {

		drive = new DifferentialDrive(leftSide, rightSide);
		drive.setSafetyEnabled(true);
		drive.setMaxOutput(1.0);

	}

	@Override
	public void process() {

		if (Robot.teleop && Robot.enabled) {

			SmartDashboard.putNumber("power out", -Robot.oi.getController1().getY(Hand.kLeft));

			if(Robot.oi.getController1().getTriggerAxis(Hand.kRight) >= 0.5){

				drive.arcadeDrive(-Robot.oi.getController1().getY(Hand.kLeft) * 0.6, Robot.oi.getController1().getX(Hand.kLeft) * 0.8);

			}else{

				drive.arcadeDrive(-Robot.oi.getController1().getY(Hand.kLeft), Robot.oi.getController1().getX(Hand.kLeft));

			}

		} else if (Robot.autonomous) {

			drive.tankDrive(leftPower, rightPower);

		}

		// uncomment the following code to test for max velocity
		double velocity;
		 
		if(RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity() >
		RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity()) {
		
		velocity =
		RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity() * 10 * RobotMap.inchesPerCountMultiplier;
		
		}else{
		
		velocity =
		RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity() * 10 * RobotMap.inchesPerCountMultiplier;
		
		}
		
		SmartDashboard.putNumber("Velocity", velocity);

	}

	public void setRightPower(double power) {

		rightPower = power;

	}

	public void setLeftPower(double power) {

		leftPower = power;

	}
}