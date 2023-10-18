// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SetToAngle extends CommandBase {

  private final SwerveSubsystem swerve;
  private final double angle;

  public SetToAngle(SwerveSubsystem swerve, double angle) {
    this.swerve = swerve;
    this.angle = angle;
    addRequirements(swerve);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    //Setting the swerve to desired angle
    swerve.frontLeft.setToAngle(angle);
    swerve.frontRight.setToAngle(angle);
    swerve.backLeft.setToAngle(angle);
    swerve.backRight.setToAngle(angle);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //RUN WITH TIMEOUT
  }
}
