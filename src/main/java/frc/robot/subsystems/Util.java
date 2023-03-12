package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Util {

    /**
     * Utility function of auto aimming
     */
    public static class AimUtil {

        /**
         * Return the closest april tag id base on alliance color and absolute pose of robot
         * @return id
         */

        public static int getClosestID(Pose2d pose) {
            double currY = pose.getY();
            int id = -1;
            if(DriverStation.getAlliance().equals(Alliance.Red)) {
                if(Math.abs(currY - 4.4) < 1) {
                    id = 3;
                } else if(Math.abs(currY - 2.7) < 1) {
                    id = 2;
                } else if(Math.abs(currY - 1) < 1) {
                    id = 1;
                }
            } else {
                if(Math.abs(currY - 4.4) < 1) {
                    id = 6;
                } else if(Math.abs(currY - 2.7) < 1) {
                    id = 7;
                } else if(Math.abs(currY - 1) < 1) {
                    id = 8;
                }
            }
            return id;
        }

        /**
         * Get mid target y bases on current pose
         * @param pose
         * @return target y
         */

        public static double getMidTargetY(Pose2d pose){
            int id = getClosestID(pose);
            double tar = pose.getY();

            switch(id) {
                case 1:
                    tar = 1.07;
                    break;
                case 2:
                    tar = 2.72;
                    break;  
                case 3:
                    tar = 4.42;
                    break;  
                case 6:
                    tar = 4.42;
                    break;  
                case 7:  
                    tar = 2.72;
                    break;    
                case 8:  
                    tar = 1.07;
                    break; 
            }

            return tar;
        }
        
        /**
         * Get right target y bases on current pose
         * @param pose
         * @return target y
         */

        public static double getRightTargetY(Pose2d pose) {
            int id = getClosestID(pose);
            double tar = pose.getY();

            switch (id) {
                case 1:
                    tar = 0.4;
                    break;
                case 2:
                    tar = 2.15;
                    break;
                case 3:
                    tar = 3.85;
                    break;
                case 6:
                    tar = 5.0;
                    break;
                case 7:
                    tar = 3.25;
                    break;
                case 8:
                    tar = 1.5;
                    break;
            }

            return tar;
        }

        /**
         * Get left target y bases on current pose
         * @param pose
         * @return target y
         */

        public static double getLeftTargetY(Pose2d pose) {
            int id = getClosestID(pose);
            double tar = pose.getY();

            switch (id) {
                case 1:
                    tar = 1.5;
                    break;
                case 2:
                    tar = 3.25;
                    break;
                case 3:
                    tar = 5.0;
                    break;
                case 6:
                    tar = 3.85;
                    break;
                case 7:
                    tar = 2.15;
                    break;
                case 8:
                    tar = 0.4;
                    break;
            }
            
            return tar;
        }
    }

    /**
     * A filter for botpose from limelight.
     */

    public class BotPoseFilter {
        MedianFilter[] filter = new MedianFilter[6];

        public BotPoseFilter() {
            for(int i = 0; i < 6; ++i) {
                filter[i] = new MedianFilter(33);
            }
        }

        /**
         * Given raw data from limelight, return more stable pose of robot.
         * @param botpose
         * @return
         */

        public double[] calculate(double[] botpose) {
            double[] res = new double[6];
            for(int i = 0; i < 6; ++i) {
                res[i] = filter[i].calculate(botpose[i]);
            }
            return res;
        }
    }
}
