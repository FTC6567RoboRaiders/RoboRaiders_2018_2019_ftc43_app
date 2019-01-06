package RoboRaiders.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.AutonomousMethods.NostromoAutonomousMethods;
import RoboRaiders.Robot.NostromoBot;

@Autonomous(name="Auto: Robot Starts Towards Crater")

public class AutoCrater extends NostromoAutonomousMethods {

    NostromoBot robot = new NostromoBot();
    RoboRaidersPID robotPID = new RoboRaidersPID();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        waitForStart();

        DeployRobot(robot);

        closeRedDepot(robotPID, robot);

        //moveTest(robotPID, robot);

    }

    }

