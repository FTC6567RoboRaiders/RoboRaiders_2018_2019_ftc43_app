package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import RoboRaiders.AutoOptions.RoboRaidersPID;

@Autonomous

public class ProtoAutonomous extends RoboraiderAutonomous {

    ProtoBot robot = new ProtoBot();
    RoboRaidersPID robotPID = new RoboRaidersPID();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);
        waitForStart();


        //closeRedDepot(robotPID,robot);

        DeployRobot(robot);


    }

    }

