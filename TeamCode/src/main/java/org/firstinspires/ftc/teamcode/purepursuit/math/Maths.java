/*
 * Copyright (c) 2026 Vikram Janakiraman. All rights reserved.
 */
package org.firstinspires.ftc.teamcode.purepursuit.math;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Class to store helpful math methods
 */
public class Maths {
    public static double normalizeAngle(double angle) {
        angle = AngleUnit.normalizeRadians(angle);

        return angle;
    }
    public static Pose minus(Pose end, Pose start) {
        return new Pose(end.x - start.x,
                            end.y - start.y,
                                end.h - start.h);
    }
    public static Vector rotated(Vector vect, double angle) {
        double newX = vect.x * Math.cos(angle) - vect.y * Math.sin(angle);
        double newY = vect.x * Math.sin(angle) + vect.y * Math.cos(angle);
        return new Vector(newX, newY);
    }
    public static double dist(Vector end, Vector start) {
        return Math.hypot(end.x - start.x, end.y - start.y);
    }
//    public static double dist(Vector2d end, Vector2d start) {
//        return Math.hypot(end.x - start.x, end.y - start.y);
//    }

    /**
     *Currently most used Maths function, finds distance between two points, as Poses
     */
    public static double dist(Pose end, Pose start) {
        return Math.hypot(end.x - start.x, end.y - start.y);
    }
    public static PoseVelocity2d scaleUp(PoseVelocity2d powers, double maxPower) {
        double x = powers.linearVel.x;
        double y = powers.linearVel.y;
        double h = powers.angVel;

        // largest magnitude component (x, y, or angVel)
        double maxComponent = Math.max(Math.max(Math.abs(x), Math.abs(y)), Math.abs(h));

        // if all components are zero, nothing to scale
        if (maxComponent == 0) {
            return powers;
        }

        // scale so that maxComponent becomes exactly maxPower
        double scale = maxPower / maxComponent;

        if (maxPower > 0) {
            return new PoseVelocity2d(
                    new Vector2d(x * scale, y * scale),
                    h * scale
            );
        }
        else return powers;
    }

    public static double clamp(double input, double leftBound, double rightBound) {
        if (leftBound < rightBound) {
            if (input < leftBound) input = leftBound;
            else if (input > rightBound) input = rightBound;

            return input;
        }
        return input;
    }
    public static Vector midpoint(Vector start, Vector end) {
        return new Vector((start.x + end.x)/2, (start.y + end.y) / 2);
    }
    public static List<Double> createDoubleList(int length) {
        double[] array = new double[length];
        List<Double> list = new ArrayList<>();
        int inc = 0;
        for (double dub: array) {
            array[inc] = inc;
            inc += 1;
        }
        for (double dub: array) {
            list.add(dub);
        }
        return list;
    }
    public static double[] listToArray(List<Double> list) {
        double[] array = new double[list.size()];
        int increment = 0;
        for (double dub: list) {
            array[increment] = list.get(increment);
        }
        return array;
    }
    public static List<Double> arrayToList(double[] array) {
        List<Double> list = new ArrayList<>();
        int increment = 0;
        for (double dub: array) {
            list.add(dub);
        }
        return list;
    }
    /**
     * Reverses the s of the elements of a list of points
     * @param poses
     */
    public static List<Pose> reversePoseList(List<Pose> poses) {
        int inc = poses.size() - 1;
        List<Pose> reversed = new ArrayList<>();
        for (Pose pose: poses) {
            reversed.add(poses.get(inc));
            inc -= 1;
        }
        return reversed;
    }

//    /**
//     * Does the same thing as the above function, but with a list of segments
//     * @param segments
//     */
//    public static List<PurePursuitPathSegment> reverseSegmentList(List<PurePursuitPathSegment> segments) {
//        int inc = segments.size() - 1;
//        List<PurePursuitPathSegment> reversed = new ArrayList<>();
//        for (PurePursuitPathSegment segment: segments) {
//            reversed.add(segments.get(inc));
//            inc -= 1;
//        }
//        return reversed;
//    }
    public static double angle(Vector vector1, Vector vector2) {
        // Calculate the dot product of the two vectors
        double dotProduct = vector1.x * vector2.x + vector1.y * vector2.y;

        // Calculate the magnitudes of the vectors
        double magnitude1 = Math.sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
        double magnitude2 = Math.sqrt(vector2.x * vector2.x + vector2.y * vector2.y);

        // Calculate the cosine of the angle between the vectors
        double cosAngle = dotProduct / (magnitude1 * magnitude2);

        // Calculate the angle in radians using the arccosine function

        return Math.acos(cosAngle);
    }

    public static double angleAtan(Vector vector1, Vector vector2) {
        // Calculate the angle in radians using the atan2 function
        double angleRad = Math.atan2(vector2.y, vector2.x) - Math.atan2(vector1.y, vector1.x);

        // Ensure the angle is within the range [0, 2π]
        angleRad = angleRad >= 0 ? angleRad : angleRad + 2 * Math.PI;

        // Convert the angle to degrees
        return Math.toDegrees(angleRad);
    }
    public static int getNum(Bezier bezier, List<Bezier> beziers) {
        int counter = 0;
        for (Bezier curve: beziers) {
            if (bezier == curve) {
                break;
            }
            counter ++;
        }
        return counter;
    }
    public static double cosine(Vector vector1, Vector vector2) {
        // Calculate the dot product of the two vectors
        double dotProduct = vector1.x * vector2.x + vector1.y * vector2.y;

        // Calculate the magnitudes of the vectors
        double magnitude1 = Math.sqrt(vector1.x * vector1.x + vector1.y * vector1.y);
        double magnitude2 = Math.sqrt(vector2.x * vector2.x + vector2.y * vector2.y);

        // Calculate the cosine of the angle between the vectors
        return dotProduct / (magnitude1 * magnitude2);
    }
    public static double avg(double... numbers) {
        double counter = 0;
        for (double dub: numbers) {
            counter += dub;
        }
        return counter / numbers.length;
    }
    public static double[] solveQuad(double a, double b, double c) {
        double discriminant = Math.sqrt(b * b - 4 * a * c);
        double x1 = (-b + discriminant)/2*a;
        double x2 = (-b - discriminant)/2*a;
        double[] roots = new double[]{2};
        roots[0] = x1;
        roots[1] = x2;
        return roots;
    }
    public static int round(double num) {
        int counter = 0;
        double decimal;
        double usableNum;
        int negative = 1;

        if (num > 0) {
            usableNum = num;
        }
        else if (num < 0) {
            usableNum = -1*num;
            negative = -1;
        }
        else return 0;

        while (counter < usableNum) {
            counter ++;
        }

        decimal = usableNum - counter;

        if (decimal >= 0.5) {
            return (int) (usableNum + 1) * negative;
        }
        else return (int) (usableNum - 1) * negative;
    }
}
