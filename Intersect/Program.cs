using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace Intersect
{

    public struct Tri
    {
        public Vector3 v0, v1, v2, normal;
        //plane equation: plane = normal * X + d
        public float d;

        public Tri(float[] elems)
        {
            if( elems.Length != 9 )
            {
                throw new ArgumentException("Elements length must be 9");
            }
            v0 = new Vector3(elems[0], elems[1], elems[2]);
            v1 = new Vector3(elems[3], elems[4], elems[5]);
            v2 = new Vector3(elems[6], elems[7], elems[8]);
            Vector3 edge1_1 = v1 - v0;
            Vector3 edge2_0 = v2 - v0;

            normal = Vector3.Cross(edge1_1, edge2_0);

            d = Vector3.Dot(
               Vector3.Negate(normal),
               v0);

        }
    }

    public enum DistanceType { ZERO, NON_ZERO_SAME_SIGN, NON_ZERO_DIFFERENT_SIGN}

    public static class TriTriOverlap
    {

        public static DistanceType CheckDistances(Tri tri1, Tri tri2)
        {
            Vector3 edge2_1 = tri2.v1 - tri2.v0;
            Vector3 edge2_2 = tri2.v2 - tri2.v0;

            Vector3 normal2 = Vector3.Cross(edge2_1, edge2_2);
            float d2 = Vector3.Dot(
                Vector3.Negate(normal2),
                tri2.v0);

            //plane equation: p2 = normal2 * X + d2
            float distance1 = Vector3.Dot(normal2, tri1.v0) + d2;
            float distance2 = Vector3.Dot(normal2, tri1.v1) + d2;
            float distance3 = Vector3.Dot(normal2, tri1.v2) + d2;

            if (Math.Abs(distance1) > float.Epsilon
                && Math.Abs(distance2) > float.Epsilon
                && Math.Abs(distance3) > float.Epsilon)
            {
                if ((distance1 < 0 && distance2 < 0 && distance3 < 0)
                    ||
                    (distance1 > 0 && distance2 > 0 && distance3 > 0))
                {
                    return DistanceType.NON_ZERO_SAME_SIGN;
                }
                else
                {
                    return DistanceType.NON_ZERO_DIFFERENT_SIGN;

                }
            }
            else
            {
                return DistanceType.ZERO;
            }
        }

        public static bool IsCoplanarIntersect(Tri tri1, Tri tri2)
        {
            return true;
        }

        public static bool IsTriTriIntersect(Tri tri1, Tri tri2)
        {
            var distanceTri1_2 = CheckDistances(tri1, tri2);
            if (distanceTri1_2 == DistanceType.NON_ZERO_SAME_SIGN)
            {
                return false;
            }

            var distanceTri2_1 = CheckDistances(tri2, tri1);
            if (distanceTri2_1 == DistanceType.NON_ZERO_SAME_SIGN)
            {
                return false;
            }

            if( distanceTri1_2 == DistanceType.ZERO || distanceTri2_1 == DistanceType.ZERO )
            {
                return IsCoplanarIntersect(tri1, tri2);
            }





            return true;
        }
    }
    class Program
    {
        static void Main(string[] args)
        {
            Tri tri1 = new Tri( new float[9] { 0, 10, 20, 60, 60, 60, 10, 10, 60 } );
            Tri tri2 = new Tri( new float[9] { 0, 0, 20, 60, -60, 60, 10, -10, 60 } );
            var isIntersect = TriTriOverlap.IsTriTriIntersect(tri1, tri2);
            Console.WriteLine("isIntersect: " + isIntersect);

            tri1 = new Tri(new float[9] { 0, 10, 20, 60, 60, 60, 10, 10, 60 });
            tri2 = new Tri(new float[9] { 0, 20, 20, 60, -60, 60, 10, -10, 60 });
            isIntersect = TriTriOverlap.IsTriTriIntersect(tri1, tri2);
            Console.WriteLine("isIntersect: " + isIntersect);

            //            var triPoints_modelSpace = [
            //0, 10, 20, 60, 60, 60, 10, 10, 60,
            //0, 20, 20, 60, -60, 60, 10, -10, 60]

        }
    }
}
