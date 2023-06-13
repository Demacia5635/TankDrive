// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class MoveCommand extends CommandBase {
  /** Creates a new MoveCommand. */
  private Chassis chassis;
  private DoubleSupplier y;
  public MoveCommand(Chassis chassis, DoubleSupplier y) {
    this.y = y;
    this.chassis = chassis;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = Math.signum(y.getAsDouble())*Constants.MOVE_POWER;
    chassis.setPower(power,power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(y.getAsDouble()) < 0.1;
  }
}
