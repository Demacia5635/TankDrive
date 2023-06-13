// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class AngleForLow extends CommandBase {
  /** Creates a new AngleForLow. */
  private final Shooting shooting;
  private int time;
  public AngleForLow(Shooting shooting) {
    this.shooting = shooting;
    addRequirements(shooting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting.setTurnerPower(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooting.getUpperLimitSwitch()) {
      time++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting.setTurnerPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooting.getUpperLimitSwitch() && time > 50;
  }
}
