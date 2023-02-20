package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class PickUp extends CommandBase {
    private Intake intake;
    private Elevator elevator;

    public PickUp(Intake intake, Elevator elevator) {
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(intake, elevator);
    }

    public void execute() {
        System.out.println("Auto pickup initiated");

        elevator.to(0);
        intake.open();
        elevator.to(ElevatorConstants.maxHeight);
        intake.closeBoth();
    }
    
}
