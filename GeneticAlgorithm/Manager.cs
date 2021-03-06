﻿using System;
using System.Collections.Generic;
using System.Linq;
using Helper;
using WorkEnv;
using GeneticAlgorithm;
using PathGenerator;
using System.Diagnostics;

namespace RoboDraw
{
    class Manager
    {
        public static Manipulator Manip;
        public static Obstacle[] Obstacles;
        public static Point Goal;
        public static List<Point> Path;
        public static List<Point[]> Joints;
        public static Tree Tree;
        public static List<Attractor> Attractors;
        public static List<Tree.Node> Buffer = new List<Tree.Node>();

        public static Dictionary<string, bool> States;

        public static void Execute()
        {
            States = new Dictionary<string, bool>();
            States.Add("Obstacles", false);
            States.Add("Goal", false);
            States.Add("Path", false);
            States.Add("Joints", false);
            States.Add("Tree", false);
            States.Add("Attractors", false);

            //manipulator
            Manip = new Manipulator
            {
                Links = new double[] { 2, 2, 2, 2 },
                q = Misc.ToRad(new double[] { 0, 0, 0, 0 }),
                Base = Point.Zero
            };

            double MaxStep = 1;
            Manip.StepRanges = new double[Manip.Links.Length, 2];
            for (int i = 0; i < Manip.Links.Length; i++)
            {
                Manip.StepRanges[i, 0] = -MaxStep;
                Manip.StepRanges[i, 1] = MaxStep;
            }

            //obstacles
            Obstacles = new Obstacle[2];
            Point[] obst_data = new Point[360];
            for (int i = 0; i < obst_data.Length; i++)
            {
                obst_data[i] = new Point
                (
                    Math.Cos(i * Math.PI / 180) + 0,
                    Math.Sin(i * Math.PI / 180) + 1.5
                );
            }
            Obstacles[0] = new Obstacle(obst_data);
            for (int i = 0; i < obst_data.Length; i++)
            {
                obst_data[i] = new Point
                (
                    Math.Cos(i * Math.PI / 180) - 2.2,
                    Math.Sin(i * Math.PI / 180) + 0
                );
            }
            Obstacles[1] = new Obstacle(obst_data);
            States["Obstacles"] = true;

            //goal
            Goal = new Point(-2 * Math.Sqrt(2) - 2, 0);
            States["Goal"] = true;

            //initialize genetic algorithm parameters
            Algorithm.Initialize(Manip, Obstacles);

            IKP Solver = new IKP(0.02, Manip.Links.Length, 10, 0.2, 50);

            Attractors = new List<Attractor>();

            Random rng = new Random();
            double work_radius = Manip.Links.Sum(), x, y_plus, y_minus, y;

            //adding main attractor
            Point AttrPoint = Goal;

            double AttrWeight = Manip.DistanceTo(Goal);

            Point[] AttrArea = new Point[180];
            double r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
            for (int i = 0; i < AttrArea.Length; i++)
            {
                AttrArea[i] = new Point
                (
                    r * Math.Cos(i * Math.PI / (AttrArea.Length / 2)) + AttrPoint.x,
                    r * Math.Sin(i * Math.PI / (AttrArea.Length / 2)) + AttrPoint.y
                );
            }

            Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));

            //adding ancillary attractors
            while (Attractors.Count < 1000)
            {
                //work_radius = Manip.Links.Sum();
                x = Manip.Base.x + rng.NextDouble() * 2 * work_radius - work_radius;
                y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                y_minus = -y_plus;
                y = Manip.Base.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;
                work_radius -= Manip.Links.Sum() / 2000;

                Point p = new Point(x, y);
                bool collision = false;
                foreach (var obst in Obstacles)
                {
                    if (obst.Contains(p))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    AttrPoint = p;

                    AttrWeight = Manip.DistanceTo(p) + Goal.DistanceTo(p);

                    AttrArea = new Point[180];
                    r = 0.05 * Math.Pow(AttrWeight / Manip.DistanceTo(Goal), 4);
                    for (int i = 0; i < AttrArea.Length; i++)
                    {
                        AttrArea[i] = new Point
                        (
                            r * Math.Cos(i * Math.PI / (AttrArea.Length / 2)) + AttrPoint.x,
                            r * Math.Sin(i * Math.PI / (AttrArea.Length / 2)) + AttrPoint.y
                        );
                    }

                    Attractors.Add(new Attractor(AttrPoint, AttrWeight, AttrArea, r));
                }
            }
            States["Attractors"] = true;

            Generator.Solver = Solver;
            Generator.RRT(new Random(), ref Tree, Goal, Manip, Obstacles, 15000, 0.04);
            //Tree.RectifyWhole();

            Tree.Node start = Tree.Min(Goal), node_curr = start;
            List<Point> path = new List<Point>();
            List<double[]> configs = new List<double[]>();
            for (int i = start.Layer; i >= 0; i--)
            {
                if (i == 0)
                {
                    int a;
                    a = 2;
                }

                if (node_curr.Layer == i)
                {
                    path.Add(node_curr.p);

                    configs.Add(node_curr.q);
                    if (node_curr.Parent != null)
                    {
                        int pointsNum = node_curr.Layer - node_curr.Parent.Layer - 1;
                        if (pointsNum > 0)
                        {
                            Tree.Node[] nodes = Tree.Discretize(node_curr, node_curr.Parent, pointsNum);
                            foreach (var node in nodes)
                            {
                                configs.Add(node.q);
                            }
                        }
                    }

                    node_curr = node_curr.Parent;
                }
            }
            path.Reverse();
            configs.Reverse();
            States["Tree"] = true;

            Path = path;
            States["Path"] = true;

            Joints = new List<Point[]>();
            for (int i = 0; i < configs.Count; i++)
            {
                if (configs[i] == null)
                    break;

                if (i == 0)
                {
                    int a;
                    a = 0;
                }
                Manip.q = configs[i];
                Joints.Add(Manip.Joints);
            }
            States["Joints"] = true;
        }
    }
}
