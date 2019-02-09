package org.usfirst.frc3534.RobotBasic.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SystemBase implements SystemInterface {

	private SpeedControllerGroup rightSide = RobotMap.rightSideMotors;
	private SpeedControllerGroup leftSide = RobotMap.leftSideMotors;
	private DifferentialDrive drive;

	private double rightPower, leftPower;

	private double left_command = 0.0, right_command = 0.0;

	private double last_error, distance_last_error;

	private double KpAim = 0.065;
	private double KdAim = 0.004;
	private double KpDistance = 0.03;
	private double KdDistance = .015;
	private double min_aim_command = 0.005;

	public Drive() {

		drive = new DifferentialDrive(leftSide, rightSide);
		drive.setSafetyEnabled(true);
		drive.setMaxOutput(1.0);

	}

	@Override
	public void process() {

		//Network 
		//Attempt at calling the Network Tables for Limelight and setting it 
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		double tx = table.getEntry("tx").getDouble(0.0);
		SmartDashboard.putNumber("tx", tx);
		double ty = table.getEntry("ty").getDouble(0.0);

		if(Robot.oi.getController1().getAButton()){

			double heading_error = tx;
        	double distance_error = ty;
        	double steering_adjust = 0.0;

			if ( tx > 1.0 ) {

                steering_adjust = KpAim * heading_error + min_aim_command  + (heading_error - last_error) * KdAim ;
	   
			}
       		else if ( tx < -1.0 ) {

                steering_adjust = KpAim * heading_error - min_aim_command + (heading_error - last_error) * KdAim;
			   
			}else{

				steering_adjust = 0.0;
				left_command = 0.0;
				right_command = 0.0;

			}

			last_error = heading_error;

			double distance_adjust = KpDistance * distance_error + KdDistance * (distance_error - distance_last_error);
			
			distance_last_error = distance_error;

        	left_command = steering_adjust + distance_adjust ;
			right_command = -steering_adjust + distance_adjust ;

			drive.tankDrive(left_command, right_command);
			

		}else{

			last_error = 0;
			distance_last_error = 0;

			if (Robot.teleop && Robot.enabled) {

				//uncomment the following code to see the forward joystick to be put to the motors
				//SmartDashboard.putNumber("power out", -Robot.oi.getController1().getY(Hand.kLeft));
	
				if(Robot.oi.getController1().getTriggerAxis(Hand.kRight) >= 0.5){
	
					drive.arcadeDrive(-Robot.oi.getController1().getY(Hand.kLeft) * 0.6, Robot.oi.getController1().getX(Hand.kLeft) * 0.8);
	
				}else{
	
					drive.arcadeDrive(-Robot.oi.getController1().getY(Hand.kLeft), Robot.oi.getController1().getX(Hand.kLeft));
	
				}
	
			} else if (Robot.autonomous) {
	
				drive.tankDrive(leftPower, rightPower);
	
			}

		}

		// uncomment the following code to test for max velocity
		
		double velocity;
		 
		if(RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity() > RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity()) {
		
			velocity = RobotMap.frontLeftMotor.getSensorCollection().getQuadratureVelocity() * 10 * RobotMap.inchesPerCountMultiplier;
		
		}else{
		
			velocity = RobotMap.frontRightMotor.getSensorCollection().getQuadratureVelocity() * 10 * RobotMap.inchesPerCountMultiplier;
		
		}
		
		SmartDashboard.putNumber("Velocity", velocity);
		SmartDashboard.putNumber("Left Distance", RobotMap.frontLeftMotor.getSensorCollection().getQuadraturePosition() * RobotMap.inchesPerCountMultiplier);
		SmartDashboard.putNumber("Right Distance", RobotMap.frontRightMotor.getSensorCollection().getQuadraturePosition() * RobotMap.inchesPerCountMultiplier);

	}

	public void setRightPower(double power) {

		rightPower = power;

	}

	public void setLeftPower(double power) {

		leftPower = power;

	}

	public double getNavxAngle(){

		return RobotMap.navx.getAngle();

	}
}