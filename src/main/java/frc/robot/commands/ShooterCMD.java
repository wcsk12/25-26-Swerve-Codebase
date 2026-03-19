// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCMD extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final double targetRPM;
  private final CommandXboxController controller;
  private double realRPM;
  private double speed = 0.67; //default speed
  private final double deadzone = DriveConstants.shooterRPMDeadzone;

  private double startTimer;
  private double timer1;
  private double timer2;
  private final boolean useClosedLoop;
  
  /** Creates a new ShooterCMD. */
  public ShooterCMD(ShooterSubsystem shooterSubsystem, double targetRPM, CommandXboxController controller) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetRPM = targetRPM;
    this.controller = controller;
    this.timer1 = 0.0;
    this.useClosedLoop = targetRPM > 500; // treat >500 as an RPM target -> use closed-loop
  }

  // Non-Xbox constructor
  public ShooterCMD(ShooterSubsystem miscSubsystem, double targetRPM) {
    this.shooterSubsystem = miscSubsystem;
    this.targetRPM = targetRPM;
    this.controller = null;
    this.timer1 = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(miscSubsystem);
    this.useClosedLoop = targetRPM > 500;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("shooting started, target RPM: " + targetRPM + " useClosedLoop=" + useClosedLoop);
    timer1 = System.currentTimeMillis() + 1500; // Add 1500 ms delay after shooting is started before
      //beginning to allow the shooting speed to be changed
    if (useClosedLoop) {
      // Command closed-loop controller to the desired RPM. The tuner should have
      // updated the closed-loop gains already; this will use those gains.
      shooterSubsystem.setClosedLoopTargetRPM(targetRPM);
      // allow the normal execute loop to monitor RPM and rumble
      return;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useClosedLoop) {
      realRPM = Math.abs(shooterSubsystem.getMeasuredRPM());
      System.out.println("rpm: " + realRPM + " target: " + targetRPM);
      // give operator a rumble when near target
      if (controller != null) {
        double error = Math.abs(targetRPM - realRPM);
        if (error < 150) controller.setRumble(RumbleType.kBothRumble, 1);
        else controller.setRumble(RumbleType.kBothRumble, 0);
      }
      return;
    }
    // fallback to legacy open-loop incremental control
    realRPM = Math.abs(shooterSubsystem.getShooterRPM());
    timer2 = System.currentTimeMillis();
    if (timer2 - timer1 > 333) {
      if (realRPM < targetRPM - deadzone) {
        speed += 0.01;
      } else if (realRPM > targetRPM + (deadzone * 0.75)) {
        speed -= 0.012;
      }
      timer1 = System.currentTimeMillis();
    }
    if (speed > 1.0) speed = 1.0;
    else if (speed < 0) speed = 0;
    System.out.println("rpm: " + realRPM + " power: " + speed);
    shooterSubsystem.setShooterSpeed(speed);
    if (controller != null) controller.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (useClosedLoop) {
      shooterSubsystem.setClosedLoopTargetRPM(0);
    } else {
      shooterSubsystem.setShooterSpeed(0);
    }
    if (controller != null) controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}