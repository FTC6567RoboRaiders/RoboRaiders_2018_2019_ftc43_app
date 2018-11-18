package RoboRaiders.AutoOptions;

public class RoboRaidersPID {
    public double Kp = 0.004;
    public double Ki = 0.0;
    public double Kd = 0.005;
    public double error;
    public double integral;
    public double derivative;
    public double previous_error;
    public double power;

    private double currentTime;
    private double previous_time = 0;
    private double timeChange;




    public double pidWithCounts(double Target, double Sensor) {

        currentTime = System.currentTimeMillis();
        timeChange = (currentTime - previous_time);

        error = (Target) - (Sensor);
        integral = integral + (error * timeChange);

        if (error == 0) {
            integral = 0;
        }
        if (Math.abs(error) > 100) {
            integral = 0;
        }
        derivative = (error - previous_error) / timeChange;
        previous_error = error;
        power = Kp * error + Ki * integral + Kd * derivative;

        previous_time = System.currentTimeMillis();
        return power;
    }


     public double pidWithDistance (double Sensor, double Target){
         currentTime = System.currentTimeMillis();
         timeChange = (currentTime - previous_time);

         error = (Sensor) - (Target);
         integral = integral + (error * timeChange);

         if (error == 0) {
             integral = 0;
         }
         if (Math.abs(error) > 100) {
             integral = 0;
         }
         derivative = (error - previous_error) / timeChange;
         previous_error = error;
         power = Kp * error + Ki * integral + Kd * derivative;


         return power;
     }

 }
