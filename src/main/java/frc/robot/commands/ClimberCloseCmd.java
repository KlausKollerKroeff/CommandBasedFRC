// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Climber;

/** An example command that uses an example subsystem. */
public class ClimberCloseCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Climber climber;
  private double setPositionClimberPID;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCloseCmd(Climber climber, double setPositionClimberPID) {
    this.setPositionClimberPID = setPositionClimberPID;
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Climber Closing", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setElevatorPIDPosition(this.setPositionClimberPID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Climber Closing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.pidClimberController.atSetpoint();
  }
}
