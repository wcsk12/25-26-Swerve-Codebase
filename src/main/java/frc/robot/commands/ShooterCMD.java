// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MiscSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCMD extends Command {
  private final MiscSubsystem miscSubsystem;
  private final double targetRPM;
  private final CommandXboxController controller;
  private double realRPM;
  private double speed = 0.7; //default speed
  private final double deadzone = DriveConstants.shooterRPMDeadzone;
  private double startTimer;
  private double timer1;
  private double timer2;
  
  /** Creates a new ShooterCMD. */
  public ShooterCMD(MiscSubsystem miscSubsystem, double targetRPM, CommandXboxController controller) {
    this.miscSubsystem = miscSubsystem;
    this.targetRPM = targetRPM;
    this.controller = controller;
    this.timer1 = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(miscSubsystem);
  }

  // Non-Xbox constructor
  public ShooterCMD(MiscSubsystem miscSubsystem, double targetRPM) {
    this.miscSubsystem = miscSubsystem;
    this.targetRPM = targetRPM;
    this.controller = null;
    this.timer1 = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(miscSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("shooting started, target RPM: " + targetRPM);
    timer1 = System.currentTimeMillis() + 1500; // Add 1500 ms delay after shooting is started before
      //beginning to allow the shooting speed to be changed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (Robot.limelight_id() == 10 || Robot.limelight_id() == 25){
      if (1.0 > Math.abs(speed)){
      miscSubsystem.setShooterSpeed(Math.pow(0.1, speed));
      }
    }
    else{
      miscSubsystem.setShooterSpeed(speed);
    }*/
    realRPM = Math.abs(miscSubsystem.getShooterRPM());
    /* timer1 resets after every change to lower the number of times the shooter's speed can change every 
       second. timer2 is the real current time. When initializing timer1 it is given an extra delay to
       allow the shooter to speed up before having the speed variable be changed.
    */
    timer2 = System.currentTimeMillis();
    if (timer2 - timer1 > 333) { // If a third of a second has passed since last change, allow speed to change
      if (realRPM < targetRPM - deadzone) { 
        speed += 0.01; // Raise power if rpm is lower than target
      } else if (realRPM > targetRPM + deadzone) {
        speed -= 0.01; // Lower power if rpm is higher than target
      }
      timer1 = System.currentTimeMillis(); // Reset timer1
    }
    if (speed > 1.0) { // Normalize speed values
      speed = 1.0;
    } else if (speed < 0) { // We don't want our shooter to move backwards
      speed = 0;
    }
    System.out.println("rpm: " + realRPM + " power: " + speed);
    miscSubsystem.setShooterSpeed(speed);
    if (controller != null) {
      controller.setRumble(RumbleType.kBothRumble, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    miscSubsystem.setShooterSpeed(0);
    if (controller != null) {
      controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}