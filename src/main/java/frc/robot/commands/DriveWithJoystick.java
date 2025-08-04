package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;


public class DriveWithJoystick extends Command {
  private final SwerveDrivetrain swerve;
  private final Joystick controller;

  private final SlewRateLimiter xspeedLimiter, yspeedLimiter, rotLimiter;
  private final boolean fieldRelative;

  double maxSpeed;

  public DriveWithJoystick(SwerveDrivetrain dt, Joystick cl, boolean fr) {
    swerve = dt;
    controller = cl;
    fieldRelative = fr;

    xspeedLimiter = new SlewRateLimiter(3);
    yspeedLimiter = new SlewRateLimiter(3);
    rotLimiter = new SlewRateLimiter(3);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    maxSpeed = SwerveDrivetrain.kMaxSpeed;

    final var xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(0), 0.02))
            * maxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(1), 0.02))
            * maxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRawAxis(3), 0.02))
            * maxSpeed;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}