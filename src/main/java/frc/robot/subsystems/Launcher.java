/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.LauncherConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Launcher extends SubsystemBase implements Loggable {
  private final CANSparkMax m_motorL, m_motorR;
  private final CANEncoder  m_encoderL, m_encoderR;
  private final CANPIDController m_pidVelocity;

  private final Spark msim_motorL, msim_motorR;
  private final Encoder msim_encoderL, msim_encoderR;

  @Log.Graph
  private double m_velocityL, m_velocityR;

  @Config
  private double m_kP, m_kI, m_kD, m_kIz, m_kFF;

  @Config
  private double m_velocitySetpoint;

  /**
   * Creates a new Launcher.
   */
  public Launcher() {
    if (Robot.isReal()) {
      m_motorL = new CANSparkMax(LauncherConstants.kLauncherMotorLeft_id, MotorType.kBrushless);
      m_motorR = new CANSparkMax(LauncherConstants.kLauncherMotorRight_id, MotorType.kBrushless);
      m_motorL.restoreFactoryDefaults();
      m_motorR.restoreFactoryDefaults();

      m_motorL.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
      m_motorR.setClosedLoopRampRate(LauncherConstants.kClosedLoopRampRate);
      m_motorL.setIdleMode(LauncherConstants.kIdleMode);
      m_motorR.setIdleMode(LauncherConstants.kIdleMode);

      m_motorR.follow(m_motorL, true); // Set the output of right motor to opposite of that of the left motor

      m_encoderL = new CANEncoder(m_motorL);
      m_encoderR = new CANEncoder(m_motorR);
      
      // Setup the controller on leader; the follower will get the voltage values from leader.
      m_pidVelocity = new CANPIDController(m_motorL);

      // set PID coefficients
      m_kP = LauncherConstants.kP;
      m_kI = LauncherConstants.kI;
      m_kD = LauncherConstants.kD;
      m_kIz = LauncherConstants.kIz;
      m_kFF = LauncherConstants.kFF;
      m_pidVelocity.setP(m_kP);
      m_pidVelocity.setI(m_kI);
      m_pidVelocity.setD(m_kD);
      m_pidVelocity.setIZone(m_kIz);
      m_pidVelocity.setFF(m_kFF);
      m_pidVelocity.setOutputRange(LauncherConstants.kMinOutput, LauncherConstants.kMaxOutput);

      msim_motorL = msim_motorR = null;
      msim_encoderL = msim_encoderR = null;
    } else {
      // This code is running in Simulation
      msim_motorL = new Spark(4);
      msim_motorR = new Spark(5);
      msim_encoderL = new Encoder(4, 5);
      msim_encoderR = new Encoder(6, 7);

      m_motorL = m_motorR = null;
      m_encoderL = m_encoderR = null;
      m_pidVelocity = null;
    }
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      // This method will be called once per scheduler run
      m_velocityL = m_encoderL.getVelocity();
      m_velocityR = m_encoderR.getVelocity();
      launchAtRPM(m_velocitySetpoint);
    } else {
      m_velocityL = msim_encoderL.getRate();
      m_velocityR = msim_encoderR.getRate();
      launchAtRPM(m_velocitySetpoint);
    }
  }

  @Config(name = "Set kP value", defaultValueNumeric = 1.0)
  public void set_kP(double kP) { m_kP = kP; }
  @Config
  public void set_kI(double kI) { m_kI = kI; }
  @Config
  public void set_kD(double kD) { m_kD = kD; }
  @Config
  public void set_kIz(double kIz) { m_kIz = kIz; }
  @Config
  public void set_kFF(double kFF) { m_kFF = kFF; }
  @Config
  public void set_velocitySetpoint(double setpoint) { m_velocitySetpoint = setpoint; }

  public void launchAtRPM(double rpm) {
    System.out.println("lunchAtRPM: rpm = " + rpm);
    if (Robot.isReal()) {
      m_pidVelocity.setReference(rpm, ControlType.kVelocity);  
    } else {
      double pctSpeed = rpm / LauncherConstants.maxRPM;
      msim_motorL.set(pctSpeed);
      msim_motorR.set(-pctSpeed);
    }
  }

  public void launch(double pctSpeed) {
    if (Robot.isReal()) {
      double setPoint = pctSpeed * LauncherConstants.maxRPM;
      m_pidVelocity.setReference(setPoint, ControlType.kVelocity);  
    } else {
      msim_motorL.set(pctSpeed);
      msim_motorR.set(-pctSpeed);
    }
  }
}
