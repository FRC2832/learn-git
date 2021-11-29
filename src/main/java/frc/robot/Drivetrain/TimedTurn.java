package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Turn the robot 90* left or right
 */
public class TimedTurn extends CommandBase {
    private Drivetrain drive;
    private Timer timer;
    private boolean rightTurn;

    /**
     * Turn the robot 90* based on rightTurn.
     * @param drive Drivetrain subsystem to command
     * @param rightTurn Is the turn right?
     */
    public TimedTurn(Drivetrain drive, boolean rightTurn) {
        //copy inputs to the command
        this.drive = drive;
        this.rightTurn = rightTurn;

        //add this command to the drivetrain subsystem
        addRequirements(drive);

        //initialize the timer
        timer = new Timer();
    }

    @Override
    public void initialize() {
        //start the timer when the command starts
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double turn = 0;

        if(rightTurn == true) {
            //turn right
            turn = -0.322;
        } else {
            //turn left
            turn = 0.322;
        }
        drive.arcadeDrive(0, turn);
    }

    @Override
    public boolean isFinished() {
        //turns always last 1 sec
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        //when stopped, stop the drivetrain
        drive.arcadeDrive(0, 0);
    }
}
