package RoboRaiders.AutonomousMethdos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.AutonomousMethdos.AutoOptions.RoboRaidersPID;
import RoboRaiders.Robot.NostromoBot;


public abstract class NostromoAutonomousMethods extends LinearOpMode {

    public double motor_power;

    public void
    farRedDepot (RoboRaidersPID robotPID, NostromoBot robot) throws InterruptedException {
        EncoderDrivePID(robotPID, robot,50 );
        Thread.sleep(500);

        imuTurn(robot, 90, .25, "right");
        Thread.sleep(500);

        DeployTeamMarker(robot);



        encodersMove(robot, 6, .9, "backward");
        Thread.sleep(500);


        imuTurn(robot, 60, .25, "left");
        Thread.sleep(500);

        encodersMove(robot, 30, .9, "backward");
        Thread.sleep(500);

        imuTurn(robot, 105, .35, "right");  // was 100
        Thread.sleep(250);

        encodersMove(robot, 15, .9, "backward");

        }

     public void closeRedDepot (RoboRaidersPID robotPID, NostromoBot robot) throws InterruptedException {
        //DeployRobot(robot);

        EncoderDrivePID(robot,28);
        Thread.sleep(200);

        encodersMove(robot, 3, 1, "backward");
        Thread.sleep(200);

        imuTurn(robot, 100, .35, "left");
        Thread.sleep(100);

        EncoderDrivePID(robot, 41);   // was 40 now 41
        Thread.sleep(100);

        encodersMove(robot, 3.0,0.25,"forward"); // was 1
        Thread.sleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55
        Thread.sleep(250);

        EncoderDrivePID(robot,36 ); // was 39
        Thread.sleep(100);

        encodersMove(robot, 1,0.25,"forward");
        Thread.sleep(100);

        robot.collectionOff();


        imuTurn(robot, 105, .35, "right");  // was 100
         Thread.sleep(250);

        DeployTeamMarker(robot);

        imuTurn(robot, 90, .35, "right"); // was 85, now at 90
        Thread.sleep(200);

        EncoderDrivePID(robot, 68);  // was 78 inches
        Thread.sleep(250);
     }
     public void farBlueDepot (RoboRaidersPID robotPID, NostromoBot robot) throws InterruptedException {
         EncoderDrivePID(robotPID,robot,28);
         Thread.sleep(500);

         encodersMove(robot, 6, .80, "backward");
         Thread.sleep(500);

         imuTurn(robot, 90, .25, "left");
         Thread.sleep(500);

         EncoderDrivePID(robotPID, robot, 35);
         Thread.sleep(500);

         imuTurn(robot, 40, .25, "left");
         Thread.sleep(500);

         EncoderDrivePID(robotPID, robot, 34 );
         Thread.sleep(500);

         imuTurn(robot, 90, .25, "right");
         Thread.sleep(500);

         imuTurn(robot, 90, .30, "right");
         Thread.sleep(500);

         EncoderDrivePID(robotPID, robot, 78);
         Thread.sleep(500);
     }
     public void closeBlueDepot (RoboRaidersPID robotPID, NostromoBot robot) throws InterruptedException {
         EncoderDrivePID(robotPID, robot,48 );
         Thread.sleep(500);

         imuTurn (robot, 90, .25, "left");
         Thread.sleep(500);

         EncoderDrivePID(robotPID, robot, 1);
         Thread.sleep(500);

         imuTurn(robot, 45, .25, "left");
         Thread.sleep(500);

         EncoderDrivePID(robotPID, robot, 60);
         Thread.sleep(500);
     }
    public void moveTest (RoboRaidersPID robotPID, NostromoBot robot) throws InterruptedException {
        EncoderDrivePID(robotPID, robot, 48);
    }
    public void moveDepotFromCraterStart (NostromoBot robot) throws  InterruptedException {
        EncoderDrivePID(robot,28);
        Thread.sleep(200);

        encodersMove(robot, 3, 1, "backward");
        Thread.sleep(200);

        imuTurn(robot, 100, .35, "left");
        Thread.sleep(100);

        EncoderDrivePID(robot, 41);   // was 40 now 41
        Thread.sleep(100);

        encodersMove(robot, 3.0,0.25,"forward"); // was 1
        Thread.sleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55
        Thread.sleep(250);

        EncoderDrivePID(robot,36 ); // was 39
        Thread.sleep(100);

        encodersMove(robot, 1,0.25,"forward");
        Thread.sleep(100);

        robot.collectionOff();

        imuTurn(robot, 105, .35, "right");
        Thread.sleep(250);
    }

    public void moveDepotFromDepotStart (NostromoBot robot) throws InterruptedException {
        EncoderDrivePID(robot,50 );
        Thread.sleep(500);

        imuTurn(robot, 90, .25, "right");
        Thread.sleep(500);
    }

    public void parkFromCraterStart (NostromoBot robot) throws InterruptedException {
        imuTurn(robot, 90, .35, "right");
        Thread.sleep(200);

        EncoderDrivePID(robot, 68);
        Thread.sleep(250);
    }

    public void parkFromDepotStart (NostromoBot robot) throws InterruptedException {
        encodersMove(robot, 6, .9, "backward");
        Thread.sleep(500);


        imuTurn(robot, 60, .25, "left");
        Thread.sleep(500);

        encodersMove(robot, 30, .9, "backward");
        Thread.sleep(500);

        imuTurn(robot, 35, .25, "right");
        Thread.sleep(500);

        encodersMove(robot, 15, .9, "backward");
    }





    public void EncoderDrivePID(RoboRaidersPID robotPID, NostromoBot robot, double wantedDistance) {
        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about

        double EncoderCount = robot.calculateCOUNTS(wantedDistance);
        double currentEncoderCount = robot.getSortedEncoderCount();
        while (opModeIsActive() &&
                (currentEncoderCount <= EncoderCount - 30.0 || currentEncoderCount >= EncoderCount + 30.0)) {

            motor_power =robotPID.pidWithCounts(EncoderCount, robot.getSortedEncoderCount());
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);
            currentEncoderCount = robot.getSortedEncoderCount();


        }
    }
   /* public void imuTurnWithPID (NostromoBot robot, float degrees, String direction ) {
        robot.resetIMU();
        float finalHeading = robot.getHeading() + degrees;

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
        }
        else if (direction.equals("left")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
        }

        while (robot.getHeading() < (finalHeading - 20) && opModeIsActive()) { //while the value of getHeading is
            //less then the degree value
            //and while opMode is active continue the while loop

            telemetry.addData("Heading", robot.getHeading()); //feedback of getHeading value
            telemetry.update(); //continuous update
        }

        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }
    }*/
    public void EncoderDrivePID(NostromoBot robot, double wantedDistance) {
    //    robot.resetEncoders();
    //    robot.runWithEncoders();
        RoboRaidersPID pidClass = new RoboRaidersPID();   // create new pidClass

        EncoderDrivePID(pidClass,robot,wantedDistance);

    }
    public void imuTurn(NostromoBot robot, float degrees, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        robot.resetIMU();
        float finalHeading = robot.getHeading() + degrees;

       // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
        }
        else if (direction.equals("left")) { //if the desired direction is left

            robot.setDriveMotorPower(-power, power, -power, power); //the robot will turn left
        }

        while (robot.getHeading() < (finalHeading - 20) && opModeIsActive()) { //while the value of getHeading is
            //less then the degree value
            //and while opMode is active continue the while loop

            telemetry.addData("Heading", robot.getHeading()); //feedback of getHeading value
            telemetry.update(); //continuous update
        }

        robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stops robot
    }



    public void DeployRobot(NostromoBot robot) throws InterruptedException{

        double startDeployTime = System.currentTimeMillis();

        robot.setLiftMotorPower(-0.95);
        while (opModeIsActive() && System.currentTimeMillis()-startDeployTime < 6500 && !robot.sensorTouch.isPressed()){

        }

        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime >= 6500 && System.currentTimeMillis() - startDeployTime < 10000 && !robot.sensorTouch.isPressed()){
            robot.setLiftMotorPower(-.45);
        }


        //while (opModeIsActive() && !robot.sensorTouch.isPressed() && System.currentTimeMillis()-startDeployTime < 7800) {
                            // System.currentTimeMillis()-startDeployTime is the elapsed time (the current time minus the start time)
        //}

        robot.setLiftMotorPower(0);

        //Thread.sleep (250);

        //encodersMove(robot, 1.5, .5, "backward");

        Thread.sleep(500);

        robot.liftClaw.setPosition(robot.liftClawOpen);

        Thread.sleep(1200);

        robot.collectionIn();

    }

    public void DeployTeamMarker(NostromoBot robot) throws InterruptedException{

        robot.markerDrop.setPosition(robot.markerDropDown);

        Thread.sleep(500);

        robot.markerDrop.setPosition(robot.markerDropUp);

        Thread.sleep(500);

    }




  /*  public void DistanceDrivePID() {
        while (opModeIsActive() && robot.getSensorDistance() < Target) {
            motor_power = drivePID.pidWithDistance(robot.getSensorDistance(), Target);
            robot.setDriveMotorPower(motor_power, motor_power, motor_power, motor_power);

            telemetry.addData("Distance Sensor", robot.getSortedEncoderCount());
            telemetry.addData("Target Distance", Target);
            telemetry.update();

        }*/
  public void encodersMove(NostromoBot robot, double distance, double power, String direction) { //sets the parameters

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


