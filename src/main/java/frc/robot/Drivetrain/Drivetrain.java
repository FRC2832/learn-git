package frc.robot.Drivetrain;

import frc.robot.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Make a drivetrain subsystem for our robot
 */
public class Drivetrain extends SubsystemBase {
    // robot parts
    private VictorSP leftDrive;
    private VictorSP rightDrive;
    private DifferentialDrive drive;
    private ADXRS450_Gyro gyro;
    private Encoder leftEncoder;
    private Encoder rightEncoder;

    /**
     * Initialize the Drivetrain components
     * @param sim Simulation object to fill
     */
    public Drivetrain(Simulate sim) {
        //call the register function from SubsystemBase
        super();

        // initialize robot parts and locations where they are
        leftDrive = new VictorSP(1);                //left drive motor is connected to VictorSP on PWM channel 1
        rightDrive = new VictorSP(2);               //right drive motor is connected to VictorSP on PWM channel 2
        drive = new DifferentialDrive(leftDrive, rightDrive);   //combine the 2 motors into a drivetrain
        gyro = new ADXRS450_Gyro();                 //Kit Gyro plugged into the SPI port on the top right of RoboRIO (traditionally we use a Pigeon IMU on a TalonSRX)
        leftEncoder = new Encoder(1, 2);            //Encoder for the left axle plugged into DIO 1 and 2 (traditionally we use the TalonSRX IO port)
        rightEncoder = new Encoder(3, 4);           //Encoder for the right axle plugged into DIO 3 and 4 (traditionally we use the TalonSRX IO port)
        leftEncoder.setDistancePerPulse(0.01);      //set to 1 meter / x pulses, need to measure on robot
        rightEncoder.setDistancePerPulse(0.01);     //set to 1 meter / x pulses, need to measure on robot

        //setup drivetrain simulation (maybe simulation should go to simulationPeriodic in Subsystems)
        sim.RegisterDrivetrain(gyro, leftEncoder, rightEncoder);
    }

    /**
     * Drive the robot with a speed and turn input
     * @param speed How fast to drive (-1 to 1)
     * @param turn How fast to turn (-1 to 1)
     */
    public void arcadeDrive(double speed, double turn) {
        drive.arcadeDrive(speed, turn);
    }

    /**
     * Gets the left drivetrain distance
     * @return Distance traveled in meters, forward is positive
     */
    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    /**
     * Get drivetrain angle.  Will keep incrementing past a circle (360 to 361*).
     * @return Angle the drivetrain is facing in degrees
     */
    public double getAngle() {
        return gyro.getAngle();
    }
}
