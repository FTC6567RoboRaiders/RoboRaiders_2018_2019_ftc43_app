package org.firstinspires.ftc.teamcode;

import android.sax.EndElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.AutoOptions.RoboRaidersPID;



public abstract class RoboraiderAutonomous extends LinearOpMode {

    public double motor_power;

    public void closeRedDepot (RoboRaidersPID robotPID, ProtoBot robot) throws InterruptedException {
        EncoderDrivePID(robotPID, robot,2 );
        Thread.sleep(500);

        imuTurn(robot,30, 1, "left");
        Thread.sleep(500);

        EncoderDrivePID(robotPID,robot,2);
        Thread.sleep(500);

        imuTurn(robot,20,1,"right");
        Thread.sleep(500);

        EncoderDrivePID(robotPID,robot, 2);
        Thread.sleep(500);

        imuTurn(robot, 90, 1, "right");
        Thread.sleep(500);

        EncoderDrivePID(robotPID,robot, 3);
        Thread.sleep(500);

        }

     public void farRedDepot (RoboRaidersPID robotPID, ProtoBot robot) throws InterruptedException {
        EncoderDrivePID( robotPID, robot,1);

        imuTurn(robot,90,1,"left");

        EncoderDrivePID(robotPID,robot,2);

        imuTurn(robot,20, 1, "right");

        EncoderDrivePID(robotPID,robot, 2);

        imuTurn(robot, 90,1, "left");

        EncoderDrivePID(robotPID,robot,4);
     }





    public void EncoderDrivePID(RoboRaidersPID robotPID, ProtoBot robot, double wantedDistance) {
          robot.resetEncoders();
          robot.runWithEncoders();
        while (opModeIsActive() && robot.getSortedEncoderCount() < robot.calculateCOUNTS(wantedDistance)) {
                motor_power =robotPID.pidWithCounts(robot.calculateCOUNTS(wantedDistance), robot.getSortedEncoderCount());
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);
            telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
            telemetry.addData("Target Count", robot.calculateCOUNTS(wantedDistance));
            telemetry.update();
        }
    }
    public void imuTurn(ProtoBot robot, float degrees, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string

        robot.resetIMU(); //resets IMU angle to zero

        robot.getHeading(); //returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
        }
        else if (direction.equals("left")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
        }

        while (robot.getHeading() < (degrees - 20) && opModeIsActive()) { //while the value of getHeading is
            //less then the degree value
            //and while opMode is active continue the while loop

            telemetry.addData("Heading", robot.getHeading()); //feedback of getHeading value
            telemetry.update(); //continuous update
        }

        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }


    public void DeployRobot(ProtoBot robot) throws InterruptedException {

        while (!robot.sensorTouch.isPressed()) {
            robot.setLiftMotorPower(-0.95);


            if (robot.sensorTouch.isPressed()) {

                robot.setLiftMotorPower(0);

                robot.liftClaw.setPosition(robot.liftClawOpen);
            }
        }



        }

  /*  public void DistanceDrivePID() {
        while (opModeIsActive() && robot.getSensorDistance() < Target) {
            motor_power = drivePID.pidWithDistance(robot.getSensorDistance(), Target);
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

            telemetry.addData("Distance Sensor", robot.getSortedEncoderCount());
            telemetry.addData("Target Distance", Target);
            telemetry.update();

        }*/
    }

