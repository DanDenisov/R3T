using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Helper;
using WorkEnv;
using GeneticAlgorithm;

namespace PathGenerator
{
    class Generator
    {
        public static Manipulator Agent;
        public static Obstacle[] Obstacles;
        public static IKP Solver;

        public static Tree RRT(Random rng, Point goal, Manipulator manip, Obstacle[] obstacles, int k, double d)
        {
            Agent = new Manipulator(manip);
            Obstacles = obstacles;
            //for (int i = 0; i < Algorithm.Agent.StepRanges.GetLength(0); i++)
            //{
            //    for (int j = 0; j < Algorithm.Agent.StepRanges.GetLength(1); j++)
            //    {
            //        Algorithm.Agent.StepRanges[i, j] *= 2;
            //    }
            //}

            Tree tree = new Tree(new Tree.Node(null, Agent.GripperPos, Agent.q));

            for (int i = 0; i < k; i++)
            {
                if ((i + 1) % 1000 == 0)
                {
                    tree.RectifyWhole();
                }

                double work_radius;
                double x;
                double y_plus, y_minus;
                double y;

                bool GoalConvergence = false;
                if (!GoalConvergence)
                {
                    work_radius = Agent.Links.Sum();
                    x = Agent.Base.x + rng.NextDouble() * 2 * work_radius - work_radius;
                    y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                    y_minus = -y_plus;
                    y = Agent.Base.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;
                    if ((i + 1) % 800 == 0)
                        GoalConvergence = true;
                }
                else
                {
                    work_radius = 1;
                    x = goal.x + rng.NextDouble() * 2 * work_radius - work_radius;
                    y_plus = Math.Sqrt(work_radius * work_radius - x * x);
                    y_minus = -y_plus;
                    y = goal.y + (rng.NextDouble() * 2 * (y_plus - y_minus) - (y_plus - y_minus)) / 2;
                    if ((i + 1) % 100 == 0)
                        GoalConvergence = false;
                }

                Point p = new Point(x, y);
                Tree.Node min_node = tree.Min(p);

                Vector v = new Vector(min_node.p, p);
                Point p_n = min_node.p + v.Normalized * d;
                bool collision = false;
                foreach (var obst in Obstacles)
                {
                    if (obst.Contains(p_n))
                    {
                        collision = true;
                        break;
                    }
                }

                if (!collision)
                {
                    Algorithm.Agent.q = min_node.q;
                    var res = Solver.Execute(p_n);
                    if (res.Item1 && !res.Item4.Contains(true))
                    {
                        tree.AddNode(new Tree.Node(min_node, p_n, Algorithm.Agent.q.Zip(res.Item3, (t, s) => { return t + s; }).ToArray()));
                    }
                }
            }
            
            return tree;
        }
    }
}
