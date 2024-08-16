package frc.robot.subsystems.elevator;

public interface IClimber {
    public double getClimberEncoder();
    public void setElevatorPIDPosition(double setPositionClimberPID);
}