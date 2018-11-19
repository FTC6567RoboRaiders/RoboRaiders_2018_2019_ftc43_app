package org.firstinspires.ftc.teamcode;

import android.sax.EndElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.AutoOptions.RoboRaidersPID;
import RoboRaiders.reference.IndieRobot;


public abstract class RoboraiderAutonomous extends LinearOpMode {

    public double motor_power;

    public void farRedDepot (RoboRaidersPID robotPID, ProtoBot robot) throws InterruptedException {
        EncoderDrivePID(robotPID, robot,48 );
        Thread.sleep(500);

        imuTurn (robot, 90, 1, "left");
        Thread.sleep(500);

        EncoderDrivePID(robotPID, robot, 1);
        Thread.sleep(500);

        imuTurn(robot, 45, 1, "left");
        Thread.sleep(500);

        EncoderDrivePID(robotPID, robot, 60);
        Thread.sleep(500);

        }

     public void closeRedDepot (RoboRaidersPID robotPID, ProtoBot robot) throws InterruptedException {
        EncoderDrivePID(robotPID,robot,28);
        Thread.sleep(500);

        encodersMove(robot, 6, .80, "backward");
        Thread.sleep(500);

        imuTurn(robot, 90, .65, "left");
        Thread.sleep(500);

        EncoderDrivePID(robotPID, robot, 34);
         Thread.sleep(500);

        imuTurn(robot, 40, .65, "left");
         Thread.sleep(500);

        EncoderDrivePID(robotPID, robot, 32 );
         Thread.sleep(500);

        imuTurn(robot, 90, .65, "right");
         Thread.sleep(500);

        imuTurn(robot, 90, .65, "right");
         Thread.sleep(500);

        EncoderDrivePID(robotPID, robot, 78);
         Thread.sleep(500);
     }





    public void EncoderDrivePID(RoboRaidersPID robotPID, ProtoBot robot, double wantedDistance) {
          robot.resetEncoders();
          robot.runWithEncoders();
        while (opModeIsActive() && robot.getSortedEncoderCount() < robot.calculateCOUNTS(wantedDistance)) {
                motor_power =robotPID.pidWithCounts(robot.calculateCOUNTS(wantedDistance), robot.getSortedEncoderCount());
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

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



    public void DeployRobot(ProtoBot robot) throws InterruptedException{

        double startDeployTime = System.currentTimeMillis();

        robot.setLiftMotorPower(-0.95);

        while (opModeIsActive() && !robot.sensorTouch.isPressed() && System.currentTimeMillis()-startDeployTime < 7800) {
                            // System.currentTimeMillis()-startDeployTime is the elapsed time (the current time minus the start time)
        }

        robot.setLiftMotorPower(0);

        robot.liftClaw.setPosition(robot.liftClawOpen);

        Thread.sleep(1500);

    }




  /*  public void DistanceDrivePID() {
        while (opModeIsActive() && robot.getSensorDistance() < Target) {
            motor_power = drivePID.pidWithDistance(robot.getSensorDistance(), Target);
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

            telemetry.addData("Distance Sensor", robot.getSortedEncoderCount());
            telemetry.addData("Target Distance", Target);
            telemetry.update();

        }*/
  public void encodersMove(ProtoBot robot, double distance, double power, String direction) { //sets the parameters

      robot.resetEncoders(); //resets encoders
      robot.runWithEncoders(); //sets the mode back to run with encoder

      double COUNTS = robot.calculateCOUNTS(distance); //COUNTS is now equal to the value calculated

      if (direction.equals("forward")) { //if the desired direction is forward

          robot.setDriveMotorPower(power, power, power, power); //start driving forward

          while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
              //still less than the desired count and the opMode has not been stopped

              telemetry.addData("COUNTS", COUNTS);
              telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
              telemetry.update();
          }

          robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
      }
      else if (direction.equals("backward")) { //if the desired direction is backward

          robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

          while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
              //still greater than the desired count and the opMode has not been stopped

              telemetry.addData("COUNTS", COUNTS);
              telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
              telemetry.update();
          }

          robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
      }
      else if (direction.equals("right")) { //if the desired direction is right

          robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

          while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
              //still less than the desired count and the opMode has not been stopped

              telemetry.addData("COUNTS", COUNTS);
              telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
              telemetry.update();
          }

          robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
      }
      else if (direction.equals("left")) { //if the desired direction is left

          robot.setDriveMotorPower(-power, power, power, -power); //start strafing left

          while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
              //still greater than the desired count and the opMode has not been stopped

              telemetry.addData("COUNTS", COUNTS);
              telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
              telemetry.update();
          }

          robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
      }

      robot.runWithoutEncoders(); //sets the mode back to run without encoder
  }
    }


