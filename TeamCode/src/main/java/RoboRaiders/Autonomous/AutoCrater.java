package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethdos.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethdos.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous(name="Auto: Robot Starts Towards Crater")

public class AutoCrater extends NostromoAutonomousMethods {

    NostromoBot robot = new NostromoBot();
    RoboRaidersPID robotPID = new RoboRaidersPID();
    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this,"Nostormo");
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        rtd.displayRobotTelemetry("Initialized, Waiting For Start");

        waitForStart();

        rtd.displayRobotTelemetry("Started");
        rtd.displayRobotTelemetry("Deploying from Lander","Calling DeployRobot");

        DeployRobot(robot);

        closeRedDepot(robotPID, robot, rtd);

        //moveTest(robotPID, robot);

    }

    }

