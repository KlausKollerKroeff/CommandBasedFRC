// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase implements IIntake {
  private IntakeConstants intakeConstants = new IntakeConstants();

  double P = intakeConstants.P;
  double I = intakeConstants.I;
  double D = intakeConstants.D;

  public RelativeEncoder encoderIntakePivot;

  public PIDController pidIntakeController = new PIDController(this.P, this.I, this.D);
  CANSparkMax motorIntakePivot = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax motorIntakeRoller = new CANSparkMax(2, MotorType.kBrushless);
  
  @Override
  public double getIntakePivotEncoderDegrees(){
    encoderIntakePivot = motorIntakePivot.getEncoder();
    return (encoderIntakePivot.getPosition()*360);
  }

  public Intake() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Avarage Position", getIntakePivotEncoderDegrees());
  }

  @Override
  public void setIntakeRollerMotors(double intakeRollerSpeed, double velocityController){
    motorIntakeRoller.set(intakeRollerSpeed*velocityController);
  }

  @Override
  public void setIntakePivotPIDPosition(double measurementPivotIntakePID, double setPositionIntakePID){
    motorIntakePivot.set(pidIntakeController.calculate(measurementPivotIntakePID, setPositionIntakePID));
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
