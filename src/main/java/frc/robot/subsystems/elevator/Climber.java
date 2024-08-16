package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase implements IClimber {

  private ClimberConstants climberConstants = new ClimberConstants();
  public RelativeEncoder encoderClimber;
  double P = climberConstants.P;
  double I = climberConstants.I;
  double D = climberConstants.D;
  public PIDController pidClimberController = new PIDController(this.P, this.I, this.D);
  CANSparkMax motorClimber = new CANSparkMax(5, MotorType.kBrushless);
  
  @Override
  public double getClimberEncoder(){
    // Here is converting the encoder values, being 1, the value when the shooter is lift up and 0 when is lift down
    encoderClimber = motorClimber.getEncoder();
    encoderClimber.setPositionConversionFactor(1/4.801);
    return (encoderClimber.getPosition());
  }

  public Climber() {}

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
    SmartDashboard.putNumber("Encoder Avarage Position", getClimberEncoder());
  }

  @Override
  public void setElevatorPIDPosition(double setPositionClimberPID){
    motorClimber.set(pidClimberController.calculate(getClimberEncoder(), setPositionClimberPID));
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
