package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    enum intakeState {
        nothing, cone, cube
    };

    public intakeState State;
    Solenoid SolenoidPCM1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Solenoid SolenoidPCM2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    public void IntakeSubsystem() {
        State = intakeState.nothing;
    }

    public void open() {
        SolenoidPCM1.set(true);
        SolenoidPCM2.set(true);
    }

    public void closeBoth() {
        SolenoidPCM2.set(false);
        SolenoidPCM1.set(false);
    }

    public void closeRight() {
        SolenoidPCM1.set(false);
    }

    public void closeLeft() {
        SolenoidPCM2.set(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}