package frc.robot.subsystems.intake;

public interface IIntake {
    public double getIntakePivotEncoderDegrees();
    public void setIntakeRollerMotors(double intakeRollerSpeed, double velocityController);
    public void setIntakePivotPIDPosition(double measurementPivotIntakePID, double setPositionIntakePID);
}