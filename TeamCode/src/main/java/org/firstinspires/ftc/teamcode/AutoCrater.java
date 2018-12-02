package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import RoboRaiders.AutoOptions.RoboRaidersPID;

@Autonomous(name="Auto: Robot Starts Towards Crater")

public class AutoCrater extends RoboraiderAutonomous {

    ProtoBot robot = new ProtoBot();
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

