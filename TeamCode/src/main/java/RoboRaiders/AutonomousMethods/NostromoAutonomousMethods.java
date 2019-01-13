package RoboRaiders.AutonomousMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import RoboRaiders.AutonomousMethods.AutoOptions.RoboRaidersPID;
import RoboRaiders.Robot.NostromoBot;
import RoboRaiders.Robot.RobotTelemetryDisplay;


public abstract class NostromoAutonomousMethods extends LinearOpMode {

    public double motor_power;

    RobotTelemetryDisplay rtd = new RobotTelemetryDisplay(this, "Nostormo");

    public void farRedDepot(RoboRaidersPID robotPID, NostromoBot robot) {
        EncoderDrivePID(robotPID, robot, 50);

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(50));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        //DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        encodersMove(robot, 6, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);


        imuTurn(robot, 60, .25, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        encodersMove(robot, 30, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(30));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        encodersMove(robot, 15, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(15));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(15));

    }

    public void closeRedDepot(RoboRaidersPID robotPID, NostromoBot robot) {
        //DeployRobot(robot);

        EncoderDrivePID(robot, 28);

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(28));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        encodersMove(robot, 3, 1, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        imuTurn(robot, 100, .35, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To  Be Turned", String.valueOf(100));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(100);

        EncoderDrivePID(robot, 41);   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(36));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        robot.collectionOff();
        rtd.displayRobotTelemetry("Turning Off Collection");

        imuTurn(robot, 105, .35, "right");  // was 100

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        // DeployTeamMarker(robot);
        rtd.displayRobotTelemetry("Deploying Team Marker");

        imuTurn(robot, 90, .35, "right"); // was 85, now at 90

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(200);

        EncoderDrivePID(robot, 68);  // was 78 inches

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(250);
    }

    public void farBlueDepot(RoboRaidersPID robotPID, NostromoBot robot) {
        EncoderDrivePID(robotPID, robot, 28);
        robotSleep(500);

        encodersMove(robot, 6, .80, "backward");
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 35);
        robotSleep(500);

        imuTurn(robot, 40, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 34);
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");
        robotSleep(500);

        imuTurn(robot, 90, .30, "right");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 78);
        robotSleep(500);
    }

    public void closeBlueDepot(RoboRaidersPID robotPID, NostromoBot robot) {
        EncoderDrivePID(robotPID, robot, 48);
        robotSleep(500);

        imuTurn(robot, 90, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 1);
        robotSleep(500);

        imuTurn(robot, 45, .25, "left");
        robotSleep(500);

        EncoderDrivePID(robotPID, robot, 60);
        robotSleep(500);
    }

    public void moveTest(RoboRaidersPID robotPID, NostromoBot robot) {
        EncoderDrivePID(robotPID, robot, 48);
    }

    public void moveDepotFromCraterStart(NostromoBot robot) {
        EncoderDrivePID(robot, 28);//this is the mineral thing

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Drving Forward", String.valueOf(28));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        encodersMove(robot, 3, 1, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Moving Backward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(200);

        imuTurn(robot, 100, .35, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned");
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(100);

        EncoderDrivePID(robot, 41);   // was 40 now 41

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Moving Forward", String.valueOf(41));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 3.0, 0.25, "forward"); // was 1

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(3));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        imuTurn(robot, 60, .35, "left");  // was 55

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);

        EncoderDrivePID(robot, 36); // was 39

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward");
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        encodersMove(robot, 1, 0.25, "forward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(1));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(100);

        robot.collectionOff();
        rtd.displayRobotTelemetry("Turning off Collection");

        imuTurn(robot, 105, .35, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(105));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(250);
    }

    public void moveDepotFromDepotStart(NostromoBot robot) {
        EncoderDrivePID(robot, 6);//also mineral knocking was 50


        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(50));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 90, .25, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be turned");
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);
    }

    public void parkFromCraterStart(NostromoBot robot) {
        imuTurn(robot, 90, .35, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(90));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(200);

        EncoderDrivePID(robot, 68);

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Forward", String.valueOf(68));
        rtd.displayRobotTelemetry("Encoder Counts", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(250);
    }

    public void parkFromDepotStart(NostromoBot robot) {
        encodersMove(robot, 6, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving backward", String.valueOf(6));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);


        imuTurn(robot, 60, .25, "left");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(60));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        encodersMove(robot, 30, .9, "backward");

        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(30));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
        robotSleep(500);

        imuTurn(robot, 35, .25, "right");

        rtd.displayRobotTelemetry("Turning");
        rtd.displayRobotTelemetry("Degrees To Be Turned", String.valueOf(35));
        rtd.displayRobotTelemetry("Angle", String.valueOf(robot.getHeading()));
        robotSleep(500);

        encodersMove(robot, 15, .9, "backward");
        rtd.displayRobotTelemetry("Moving");
        rtd.displayRobotTelemetry("Driving Backward", String.valueOf(15));
        rtd.displayRobotTelemetry("Encoder Count", String.valueOf(robot.getSortedEncoderCount()));
    }


    public void EncoderDrivePID(RoboRaidersPID robotPID, NostromoBot robot, double wantedDistance) {
        robot.resetEncoders();
        robot.runWithEncoders();
        robotPID.initialize();   // re-initialized the pid variables that we care about

        double EncoderCount = robot.calculateCOUNTS(wantedDistance);
        double currentEncoderCount = robot.getSortedEncoderCount();
        while (opModeIsActive() &&
                (currentEncoderCount <= EncoderCount - 30.0 || currentEncoderCount >= EncoderCount + 30.0)) {

            motor_power = robotPID.pidWithCounts(EncoderCount, robot.getSortedEncoderCount());
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

        EncoderDrivePID(pidClass, robot, wantedDistance);

    }

    public void imuTurn(NostromoBot robot, float degrees, double power, String direction) { //gets hardware from
        //Robot and defines degrees as a
        //float, power as a double, and direction as a string
        robot.resetIMU();
        float finalHeading = robot.getHeading() + degrees;

        // robot.getHeading(); returns the current heading of the IMU

        if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, power, -power); //the robot will turn right
        } else if (direction.equals("left")) { //if the desired direction is left

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


    public void DeployRobot(NostromoBot robot) {

        double startDeployTime = System.currentTimeMillis();

        robot.setLiftMotorPower(-0.95);
        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime < 1500 && !robot.sensorTouch.isPressed()) {

        }

        while (opModeIsActive() && System.currentTimeMillis() - startDeployTime >= 1500 && System.currentTimeMillis() - startDeployTime < 5000 && !robot.sensorTouch.isPressed()) {
            robot.setLiftMotorPower(-.45);
        }


        //while (opModeIsActive() && !robot.sensorTouch.isPressed() && System.currentTimeMillis()-startDeployTime < 7800) {
        // System.currentTimeMillis()-startDeployTime is the elapsed time (the current time minus the start time)
        //}

        robot.setLiftMotorPower(0);

        //robotSleep (250);

        //encodersMove(robot, 1.5, .5, "backward");

        robotSleep(500);

        robot.liftClaw.setPosition(robot.liftClawOpen);

        robotSleep(1200);

    }

   /* public void DeployTeamMarker(NostromoBot robot) throws InterruptedException{

        robot.markerDrop.setPosition(robot.markerDropDown);

        robotSleep(500);

        robot.markerDrop.setPosition(robot.markerDropUp);

        robotSleep(500);

    }*/

    /**
     * Will detect the location of the gold mineral
     *
     * @param robot - the robot to work with
     */
    public void samplingMinerals(NostromoBot robot) {

        encodersMove(robot, 12, .8, "forward");
        robotSleep(200);

        int goldLocation = detectGoldMineral(robot);

        switch (goldLocation) {
            case 1:
                mineralLeft(robot);
                break;
            case 2:
                mineralCenter(robot);
                break;
            case 3:
                mineralRight(robot);
                break;

        }
    }

    /**
     * will detect the gold mineral location
     *
     * @return the location of the gold mineral
     */
    public int detectGoldMineral(NostromoBot robot) {
        int goldPostion = -1;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            robot.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (robot.tfod != null) {
                robot.tfod.activate();
            }
            CameraDevice.getInstance().setFlashTorchMode(true);

            while (opModeIsActive()) {
                if (robot.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = robot.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // Is the robot seeing 2 minerals
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            // Did the robot just see two silver minerals?
                            if (silverMineral2X != -1) {

                                // Yes, indicate the gold mineral is on the left
                                goldPostion = 1;
                            }
                            // The robot saw a gold and silver mineral
                            else {

                                // Is the gold mineral to the left of the silver mineral?
                                if (goldMineralX < silverMineral1X) {

                                    // Yes, indicate the gold mineral is in the center
                                    goldPostion = 2;
                                } else {

                                    // No, the gold mineral is on the right
                                    goldPostion = 3;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (robot.tfod != null) {
            robot.tfod.shutdown();
        }
        return goldPostion;
    }

    public void mineralLeft(NostromoBot robot) {

        encodersMove(robot, 16.9, 1, "left");
        robotSleep(200);

        EncoderDrivePID(robot, 30);
        robotSleep(200);


        imuTurn(robot, 50, .35, "right");
        robotSleep(200);

    }

    public void mineralRight(NostromoBot robot) {


        encodersMove(robot, 16.9, 1, "right");
        robotSleep(200);

        EncoderDrivePID(robot, 30);
        robotSleep(200);


        encodersMove(robot, 16.9, 1, "left");

    }

    public void mineralCenter(NostromoBot robot)  {

        EncoderDrivePID(robot, 30);
        robotSleep(200);

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
        } else if (direction.equals("backward")) { //if the desired direction is backward

            robot.setDriveMotorPower(-power, -power, -power, -power); //start driving backward

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still greater than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0, 0, 0, 0); //stop the robot
        } else if (direction.equals("right")) { //if the desired direction is right

            robot.setDriveMotorPower(power, -power, -power, power); //start strafing right

            while (robot.getSortedEncoderCount() < COUNTS && opModeIsActive()) { //while the current count is
                //still less than the desired count and the opMode has not been stopped

                telemetry.addData("COUNTS", COUNTS);
                telemetry.addData("Encoder Count", robot.getSortedEncoderCount());
                telemetry.update();
            }

            robot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0); //stop the robot
        } else if (direction.equals("left")) { //if the desired direction is left

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

    /**
     * make the robot sleep (wait)
     *
     * @param timeToSleep time in milliseconds
     */
    public void robotSleep(int timeToSleep) {
        try {
            Thread.sleep(timeToSleep);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
