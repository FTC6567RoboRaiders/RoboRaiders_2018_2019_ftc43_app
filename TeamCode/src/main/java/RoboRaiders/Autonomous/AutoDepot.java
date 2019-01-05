package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethdos.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethdos.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBot;

@Autonomous(name="Auto: Robot Starts Towards Depot")

public class AutoDepot extends NostromoAutonomousMethods {

    NostromoBot robot = new NostromoBot();
    RoboRaidersPID robotPID = new RoboRaidersPID();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        waitForStart();

        DeployRobot(robot);

        farRedDepot(robotPID, robot);

        //moveTest(robotPID, robot);

    }

    }

