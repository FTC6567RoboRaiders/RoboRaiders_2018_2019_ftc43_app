package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.Robot.NostromoBot;


@Autonomous (name="TurningTest", group="Samples")


public class TurningTest extends LinearOpMode {

    NostromoBot robot = new NostromoBot();
    public float degreesToTurn;
    public float currentHeading;
    public float finalHeading;
    public float getHeading = robot.getHeading();

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);             // Initialize the robot
        robot.resetEncoders();                     // Reset the encoder counts
        robot.runWithEncoders();                   // Tell the motors to run with encoders

        telemetry.addData("Status ", "Initialized");
        telemetry.update();

        // Wait for the start button to be pushed
        waitForStart();
        degreesToTurn = 20;
        robot.getHeading();
        currentHeading = getHeading;
        finalHeading = currentHeading + degreesToTurn;


        robot.setDriveMotorPower(0.3, -0.3, 0.3, -0.3);
        while(opModeIsActive() && currentHeading < finalHeading) {
        }
        robot.setDriveMotorPower(0, 0, 0, 0);

    }
}
