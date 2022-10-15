// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;
//import com.kauailabs.navx.frc.AHRS;


/** Add your docs here. */

public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;
  
  private double DriveToLineDirection = 1.0;
  private double DriveToLineOffset = 0.0;
  
  private ShuffleboardTab DTLTab = Shuffleboard.getTab("Drive To Line");
  private NetworkTableEntry SwitchDirection = DTLTab.add("Direction", 1).getEntry();
  private NetworkTableEntry DTLDisplacement = DTLTab.add("Displacement", 0.0).getEntry();
  private NetworkTableEntry DTLOffset = DTLTab.add("Displacement", 0.0).getEntry();
  private NetworkTableEntry LeftVelocity = DTLTab.add("Left Native Velocity", 0.0).getEntry();
  private NetworkTableEntry RightVelocity = DTLTab.add("Right Native Velocity", 0.0).getEntry();

    public DriveTrain(){
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.DriveTrainPorts.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.DriveTrainPorts.RightDriveVictorPort);
    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);
    //Configure the sensors of the Talons.
    leftDriveTalon.configFactoryDefault(); leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    
    SmartDashboard.putNumber("Left Native Velocity", leftDriveTalon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Native Velocity", leftDriveTalon.getSelectedSensorVelocity());

    DriveToLineDirection = (int) SwitchDirection.getDouble(1.0);
    DriveToLineOffset = DTLOffset.getDouble(0.0);
    DTLDisplacement.setDouble(getDisplacement());
    LeftVelocity.setDouble(leftDriveTalon.getSelectedSensorVelocity());
    RightVelocity.setDouble(rightDriveTalon.getSelectedSensorVelocity());
    //Controls the robot with joysticks - the sensitivity is reduced 
    tankDrive(RobotContainer.getJoy1().getY()*-0.2, RobotContainer.getJoy2().getY()*-0.2);
    

    
}

public double getDisplacement() {
  return (getTicks() / (Constants.DriveToLineConstants.ticksToMeters));
}


public void resetEncoders() {
  leftDriveTalon.setSelectedSensorPosition(0,0,10);
  rightDriveTalon.setSelectedSensorPosition(0,0,10);
}
public double getDTLOffset() {
  return DriveToLineOffset;
}
public double getDTLDirection() {
  return DriveToLineDirection;
}
public double getTicks() {
  return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
}

public void tankDrive(double leftSpeed, double rightSpeed) {
  rightDriveTalon.set(ControlMode.PercentOutput, rightSpeed);  
  leftDriveTalon.set(ControlMode.PercentOutput, leftSpeed);
}


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
