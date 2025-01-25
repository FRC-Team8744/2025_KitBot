// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTurnPID extends Command {
  double P = 0.01;
  double I,D = 0.0;
    private final CANDriveSubsystem m_drive;
    PIDController m_turnCtrl = new PIDController(P, I, D);
    private double m_output;
    private double m_heading;
    private double m_goalAngle;

  /** Creates a new AutoTurnPID. */
  public AutoTurnPID(double targetAngleDegrees, CANDriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.m_drive = drive;
      addRequirements(m_drive);
  
      this.m_goalAngle = targetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(1.0);
    m_turnCtrl.setSetpoint(m_goalAngle);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_heading = m_drive.getHeading();
    m_output = MathUtil.clamp(m_turnCtrl.calculate(m_heading), -0.4, 0.4);
    // Send PID output to drivebase
    m_drive.driveArcade(0.0, m_output, false);
    // m_drive.tankDriveVolts(m_output, -m_output);

    // Debug information
    SmartDashboard.putNumber("PID setpoint", m_goalAngle);
    SmartDashboard.putNumber("PID output", m_output);
    SmartDashboard.putNumber("PID setpoint error", m_turnCtrl.getPositionError());
    SmartDashboard.putNumber("PID velocity error", m_turnCtrl.getVelocityError());
    SmartDashboard.putNumber("PID measurement", m_heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.driveArcade(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnCtrl.atSetpoint();
  }
}
