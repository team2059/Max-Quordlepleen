package org.team2059.MaxQ.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team2059.MaxQ.Constants;
import org.team2059.MaxQ.subsystems.collector.Collector;
import org.team2059.MaxQ.util.LoggedTunableNumber;

public class CollectorTiltSetpointCmd extends Command {

  private Collector collector;

  private double setpoint;

  private final LoggedTunableNumber kP = new LoggedTunableNumber("CollectorKp", Constants.CollectorConstants.tiltkP);

  private final PIDController controller = new PIDController(kP.get(), 0.0, 0.0);

  public CollectorTiltSetpointCmd(Collector collector, double setpoint) {
    this.collector = collector;
    this.setpoint = setpoint;

    addRequirements(collector);
  }

  @Override
  public void initialize() {
    controller.setTolerance(0.05);
  }

  @Override
  public void execute() {

    if (kP.hasChanged(hashCode())) {
      controller.setPID(kP.get(), 0.0, 0.0);
    }

    double pidOutput = controller.calculate(collector.thruBorePos(), setpoint);

    Logger.recordOutput("CollectorPIDOutput", pidOutput);
    Logger.recordOutput("CollectorPIDSetpoint", setpoint);

    collector.setTiltMotorSpeed(pidOutput);
  }

  @Override
  public void end(boolean interrupted) {
    collector.setTiltMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

}
