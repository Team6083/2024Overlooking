// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveControls;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class Forward extends Command {
  /** Creates a new Forward. */

  Timer time = new Timer();
  Drivebase drivebase;
  public Forward(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(2.5,0,0,true);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time.get()>2.0;
  }
}
