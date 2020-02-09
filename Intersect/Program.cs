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
        public static float ElemByIndex(this Vector3 vertex3, int index)
        {
            switch( index )
            {
                case 0:
                    return vertex3.X;
                case 1:
                    return vertex3.Y;
                case 2:
                    return vertex3.Z;
                default:
                    throw new ArgumentException("Invalid index");
            }
            
        }

        public static DistanceType CheckDistances(Tri tri1, Tri tri2)
        {
            //plane equation: p2 = normal2 * X + d2
            float distance1 = Vector3.Dot(tri2.normal, tri1.v0) + tri2.d;
            float distance2 = Vector3.Dot(tri2.normal, tri1.v1) + tri2.d;
            float distance3 = Vector3.Dot(tri2.normal, tri1.v2) + tri2.d;

            if (Math.Abs(distance1) > float.Epsilon
                && Math.Abs(distance2) > float.Epsilon
                && Math.Abs(distance3) > float.Epsilon)
            {
                if ((distance1 < 0 && distance2 < 0 && distance3 < 0)
                    ||
                    (distance1 > 0 && distance2 > 0 && distance3 > 0))
                
                    return DistanceType.NON_ZERO_SAME_SIGN;
                
                else
                    return DistanceType.NON_ZERO_DIFFERENT_SIGN;
            }
            else
                return DistanceType.ZERO;
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

            var L_intersectLine = Vector3.Cross(tri1.normal, tri2.normal);
            var max = Math.Abs(L_intersectLine.X);
            var index = 0;
            var bb = Math.Abs(L_intersectLine.Y);
            var cc = Math.Abs(L_intersectLine.Z);
            if(bb > max) { max = bb; index = 1; }
            if(cc > max) { max = cc; index = 2; }

            float vp0, vp1, vp2, up0, up1, up2;
            
            vp0 = tri1.v0.ElemByIndex(index);
            vp1 = tri1.v1.ElemByIndex(index);
            vp2 = tri1.v2.ElemByIndex(index); 

            up0 = tri2.v0.ElemByIndex(index);
            up1 = tri2.v1.ElemByIndex(index);
            up2 = tri2.v2.ElemByIndex(index);

            // compute interval for triangle 1 
            float a = 0, b = 0, c = 0, x0 = 0, x1 = 0;
            float dv0 = Vector3.Dot(tri2.normal, tri1.v0) + tri2.d;
            float dv1 = Vector3.Dot(tri2.normal, tri1.v1) + tri2.d;
            float dv2 = Vector3.Dot(tri2.normal, tri1.v2) + tri2.d;
            float dv0dv1 = dv0 * dv1;
            float dv0dv2 = dv0 * dv2;

            if (ComputeIntervals(vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, ref a, ref b, ref c, ref x0, ref x1))
            {
                return false;
                //return TriTriCoplanar(n1, v0, v1, v2, u0, u1, u2);
            }


            // compute interval for triangle 1 
            float du0 = Vector3.Dot(tri1.normal, tri2.v0) + tri1.d;
            float du1 = Vector3.Dot(tri1.normal, tri2.v1) + tri1.d;
            float du2 = Vector3.Dot(tri1.normal, tri2.v2) + tri1.d;
            float du0du1 = du0 * du1;
            float du0du2 = du0 * du2;

            // compute interval for triangle 2 
            float d = 0, e = 0, f = 0, y0 = 0, y1 = 0;
            if (ComputeIntervals(up0, up1, up2, du0, du1, du2, du0du1, du0du2, ref d, ref e, ref f, ref y0, ref y1))
            {
                return false;
                //return TriTriCoplanar(n1, v0, v1, v2, u0, u1, u2);
            }

            float xx, yy, xxyy, tmp;
            xx = x0 * x1;
            yy = y0 * y1;
            xxyy = xx * yy;

            tmp = a * xxyy;

            Vector2 isect1 = Vector2.Zero, isect2 = Vector2.Zero;


            isect1.X = tmp + b * x1 * yy;
            isect1.Y = tmp + c * x0 * yy;

            tmp = d * xxyy;
            isect2.X = tmp + e * xx * y1;
            isect2.Y = tmp + f * xx * y0;

            Sort(isect1);
            Sort(isect2);

            return !(isect1.Y < isect2.X || isect2.Y < isect1.X);

        }

        private static bool ComputeIntervals(float VV0, float VV1, float VV2,
                               float D0, float D1, float D2, float D0D1, float D0D2,
                               ref float A, ref float B, ref float C, ref float X0, ref float X1)
        {
            if (D0D1 > 0.0f)
            {
                // here we know that D0D2<=0.0 
                // that is D0, D1 are on the same side, D2 on the other or on the plane 
                A = VV2; B = (VV0 - VV2) * D2; C = (VV1 - VV2) * D2; X0 = D2 - D0; X1 = D2 - D1;
            }
            else if (D0D2 > 0.0f)
            {
                // here we know that d0d1<=0.0 
                A = VV1; B = (VV0 - VV1) * D1; C = (VV2 - VV1) * D1; X0 = D1 - D0; X1 = D1 - D2;
            }
            else if (D1 * D2 > 0.0f || D0 != 0.0f)
            {
                // here we know that d0d1<=0.0 or that D0!=0.0 
                A = VV0; B = (VV1 - VV0) * D0; C = (VV2 - VV0) * D0; X0 = D0 - D1; X1 = D0 - D2;
            }
            else if (D1 != 0.0f)
            {
                A = VV1; B = (VV0 - VV1) * D1; C = (VV2 - VV1) * D1; X0 = D1 - D0; X1 = D1 - D2;
            }
            else if (D2 != 0.0f)
            {
                A = VV2; B = (VV0 - VV2) * D2; C = (VV1 - VV2) * D2; X0 = D2 - D0; X1 = D2 - D1;
            }
            else
            {
                return true;
            }

            return false;
        }

        private static bool TriTriCoplanar(Vector3 N, Vector3 v0, Vector3 v1, Vector3 v2, Vector3 u0, Vector3 u1, Vector3 u2)
        {
            float[] A = new float[3];
            short i0, i1;

            // first project onto an axis-aligned plane, that maximizes the area
            // of the triangles, compute indices: i0,i1. 
            A[0] = Math.Abs(N.ElemByIndex(0));
            A[1] = Math.Abs(N.ElemByIndex(1));
            A[2] = Math.Abs(N.ElemByIndex(2));
            if (A[0] > A[1])
            {
                if (A[0] > A[2])
                {
                    // A[0] is greatest
                    i0 = 1;
                    i1 = 2;
                }
                else
                {
                    // A[2] is greatest
                    i0 = 0;
                    i1 = 1;
                }
            }
            else
            {
                if (A[2] > A[1])
                {
                    // A[2] is greatest 
                    i0 = 0;
                    i1 = 1;
                }
                else
                {
                    // A[1] is greatest 
                    i0 = 0;
                    i1 = 2;
                }
            }

            // test all edges of triangle 1 against the edges of triangle 2 
            if (EdgeAgainstTriEdges(v0, v1, u0, u1, u2, i0, i1)) { return true; }
            if (EdgeAgainstTriEdges(v1, v2, u0, u1, u2, i0, i1)) { return true; }
            if (EdgeAgainstTriEdges(v2, v0, u0, u1, u2, i0, i1)) { return true; }

            // finally, test if tri1 is totally contained in tri2 or vice versa 
            if (PointInTri(v0, u0, u1, u2, i0, i1)) { return true; }
            if (PointInTri(u0, v0, v1, v2, i0, i1)) { return true; }

            return false;
        }

        private static void Sort(Vector2 v)
        {
            if (v.X > v.Y)
            {
                float c;
                c = v.X;
                v.X = v.Y;
                v.Y = c;
            }
        }

        /// <summary>
        /// This edge to edge test is based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202 
        /// </summary>
        private static bool EdgeEdgeTest(Vector3 v0, Vector3 v1, Vector3 u0, Vector3 u1, int i0, int i1)
        {
            float Ax, Ay, Bx, By, Cx, Cy, e, d, f;
            Ax = v1.ElemByIndex(i0) - v0.ElemByIndex(i0);
            Ay = v1.ElemByIndex(i1) - v0.ElemByIndex(i1);

            Bx = u0.ElemByIndex(i0) - u1.ElemByIndex(i0);
            By = u0.ElemByIndex(i1) - u1.ElemByIndex(i1);
            Cx = v0.ElemByIndex(i0) - u0.ElemByIndex(i0);
            Cy = v0.ElemByIndex(i1) - u0.ElemByIndex(i1);
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f))
            {
                e = Ax * Cy - Ay * Cx;
                if (f > 0)
                {
                    if (e >= 0 && e <= f) { return true; }
                }
                else
                {
                    if (e <= 0 && e >= f) { return true; }
                }
            }

            return false;
        }

        private static bool EdgeAgainstTriEdges(Vector3 v0, Vector3 v1, Vector3 u0, Vector3 u1, Vector3 u2, short i0, short i1)
        {
            // test edge u0,u1 against v0,v1
            if (EdgeEdgeTest(v0, v1, u0, u1, i0, i1)) { return true; }

            // test edge u1,u2 against v0,v1 
            if (EdgeEdgeTest(v0, v1, u1, u2, i0, i1)) { return true; }

            // test edge u2,u1 against v0,v1 
            if (EdgeEdgeTest(v0, v1, u2, u0, i0, i1)) { return true; }

            return false;
        }

        private static bool PointInTri(Vector3 v0, Vector3 u0, Vector3 u1, Vector3 u2, short i0, short i1)
        {
            float a, b, c, d0, d1, d2;

            // is T1 completly inside T2?
            // check if v0 is inside tri(u0,u1,u2)
            a = u1.ElemByIndex(i1) - u0.ElemByIndex(i1);
            b = -(u1.ElemByIndex(i0) - u0.ElemByIndex(i0));
            c = -a * u0.ElemByIndex(i0) - b * u0.ElemByIndex(i1);
            d0 = a * v0.ElemByIndex(i0) + b * v0.ElemByIndex(i1) + c;

            a = u2.ElemByIndex(i1) - u1.ElemByIndex(i1);
            b = -(u2.ElemByIndex(i0) - u1.ElemByIndex(i0));
            c = -a * u1.ElemByIndex(i0) - b * u1.ElemByIndex(i1);
            d1 = a * v0.ElemByIndex(i0) + b * v0.ElemByIndex(i1) + c;

            a = u0.ElemByIndex(i1) - u2.ElemByIndex(i1);
            b = -(u0.ElemByIndex(i0) - u2.ElemByIndex(i0));
            c = -a * u2.ElemByIndex(i0) - b * u2.ElemByIndex(i1);
            d2 = a * v0.ElemByIndex(i0) + b * v0.ElemByIndex(i1) + c;

            if (d0 * d1 > 0.0f)
            {
                if (d0 * d2 > 0.0f) { return true; }
            }

            return false;
        }


        //project vector1 onto vector2
        public static Vector3 ProjectVector(Vector3 vector1, Vector3 vector2)
        {
            //var scalar = Cartesian3.dot(a, b) / Cartesian3.dot(b, b);
            //return Cartesian3.multiplyByScalar(b, scalar, result);
            var scalar = Vector3.Dot(vector1, vector2) / Vector3.Dot(vector2, vector2);
            return Vector3.Multiply(vector2, scalar);
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

            tri1 = new Tri(new float[9] { -20, -20, 0, 20, 20, 20, -10, -10, 20, });
            tri2 = new Tri(new float[9] { 20,-30,0, 30,30,10, -15,-15,25 });
            isIntersect = TriTriOverlap.IsTriTriIntersect(tri1, tri2);
            Console.WriteLine("isIntersect: " + isIntersect);


            //            var triPoints_modelSpace = [
            //0, 10, 20, 60, 60, 60, 10, 10, 60,
            //0, 20, 20, 60, -60, 60, 10, -10, 60]

        }
    }
}
