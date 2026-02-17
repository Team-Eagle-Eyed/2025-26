package main.java.frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class MotorButtonMap {
    private final Joystick driverController;
    private final Joystick operatorController;
    private final Robot robot;
    // Expose drive logic as a Runnable field
    public final Runnable driveWithJoysticks;

    public MotorButtonMap(Joystick driverController, Joystick operatorController, Robot robot) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.robot = robot;
        configureButtonBindings();
        driveWithJoysticks = () -> {
            double l1 = driverController.getRawAxis(2);
            double forward = -driverController.getRawAxis(1);
            double rotation = driverController.getRawAxis(0);
            if (l1 > 0.05) {
                double speed = l1;
                double left = (forward + rotation) * speed;
                double right = (forward - rotation) * speed;
                robot.leftMotor.set(left);
                robot.rightMotor.set(right);
            } else {
                robot.leftMotor.set(0);
                robot.rightMotor.set(0);
            }
        };
    }

    private void configureButtonBindings() {

        // Button 1 (A on Xbox, Trigger on Logitech): Intake Forward
        new JoystickButton(operatorController, 1)
            .whileTrue(() -> robot.IntakeMotor.set(0.5));
        // Button 2 (B on Xbox, Thumb Button 2 on Logitech): Intake Reverse
        new JoystickButton(operatorController, 2)
            .whileTrue(() -> robot.IntakeMotor.set(-0.5));
        // Button 3 (X on Xbox, Thumb Button 3 on Logitech): PreShooter Forward
        new JoystickButton(operatorController, 3)
            .whileTrue(() -> robot.PreShooterMotor.set(0.5));
        // Button 4 (Y on Xbox, Thumb Button 4 on Logitech): PreShooter Reverse
        new JoystickButton(operatorController, 4)
            .whileTrue(() -> robot.PreShooterMotor.set(-0.5));
        // Button 5 (Left Bumper on Xbox, Top Left on Logitech): Shooter Left Forward
        new JoystickButton(operatorController, 5)
            .whileTrue(() -> robot.ShooterLeftMotor.set(0.5));
        // Button 6 (Right Bumper on Xbox, Top Right on Logitech): Shooter Left Reverse
        new JoystickButton(operatorController, 6)
            .whileTrue(() -> robot.ShooterLeftMotor.set(-0.5));
        // Button 7 (Back on Xbox, Base 7 on Logitech): Shooter Right Forward
        new JoystickButton(operatorController, 7)
            .whileTrue(() -> robot.ShooterRightMotor.set(0.5));
        // Button 8 (Start on Xbox, Base 8 on Logitech): Shooter Right Reverse
        new JoystickButton(operatorController, 8)
            .whileTrue(() -> robot.ShooterRightMotor.set(-0.5));
        // Button 9 (Left Stick on Xbox, Base 9 on Logitech): Intake Extender Forward
        new JoystickButton(operatorController, 9)
            .whileTrue(() -> robot.IntakeExtender.set(0.5));
        // Button 10 (Right Stick on Xbox, Base 10 on Logitech): Intake Extender Reverse
        new JoystickButton(operatorController, 10)
            .whileTrue(() -> robot.IntakeExtender.set(-0.5));
        // Button 11 (D-Pad Up on Xbox, Base 11 on Logitech): Agitator Forward
        new JoystickButton(operatorController, 11)
            .whileTrue(() -> robot.AgitatorMotor.set(0.5));
        // Button 12 (D-Pad Down on Xbox, Base 12 on Logitech): Agitator Reverse
        new JoystickButton(operatorController, 12)
            .whileTrue(() -> robot.AgitatorMotor.set(-0.5));
        // Button 13 (D-Pad Left on Xbox, Base 13 on Logitech): Kicker Forward
        new JoystickButton(operatorController, 13)
            .whileTrue(() -> robot.Kicker.set(0.5));
        // Button 14 (D-Pad Right on Xbox, Base 14 on Logitech): Kicker Reverse
        new JoystickButton(operatorController, 14)
            .whileTrue(() -> robot.Kicker.set(-0.5));
    }
}
