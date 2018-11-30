package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import RoboRaiders.AutoOptions.RoboRaidersPID;

@Autonomous(name="Auto: Robot Starts Towards Depot")

public class AutoDepot extends RoboraiderAutonomous {

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

