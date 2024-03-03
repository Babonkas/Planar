using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{

	public static class PlanarMath
	{
		public static readonly float negligibleAmt = 0.0005f;

		public static float Clamp(float value, float min, float max)
		{
			if(min == max)
			{
				return min;
			}
			if(min > max)
			{
				throw new ArgumentOutOfRangeException("Min is greater than max.");
			}
			if(value < min)
			{
				return min;
			}
			if (value > max)
			{
				return max;
			}
			return value;
		}
		public static int Clamp(int value, int min, int max)
		{
			if (min == max)
			{
				return min;
			}
			if (min > max)
			{
				throw new ArgumentOutOfRangeException("Min is greater than max.");
			}
			if (value < min)
			{
				return min;
			}
			if (value > max)
			{
				return min;
			}
			return value;
		}




		//distance from 0
		public static float LengthSquared(PlanarVector v)
		{
			return v.X * v.X + v.Y * v.Y;
		}
		public static float Length(PlanarVector v)
		{
			return MathF.Sqrt(v.X*v.X + v.Y * v.Y);
		}

		//distance between 2 vectors
		public static float Distance(PlanarVector a, PlanarVector b)
		{
			float dx = a.X - b.X;
			float dy = a.Y - b.Y;
			return MathF.Sqrt(dx*dx + dy * dy);
		}
		public static float DistanceSquared(PlanarVector a, PlanarVector b)
		{
			float dx = a.X - b.X;
			float dy = a.Y - b.Y;
			return dx * dx + dy * dy;
		}

		//same direction, magnitude of 1
		public static PlanarVector Normalize(PlanarVector v)
		{
			float len = PlanarMath.Length(v);
			return new PlanarVector(v.X / len, v.Y / len);
		}

		//projections
		public static float Dot(PlanarVector a, PlanarVector b)
		{
			return a.X * b.X + a.Y * b.Y;
		}

		//determines which side of line are points on
		public static float Cross(PlanarVector a, PlanarVector b)
		{
			return a.X * b.Y - a.Y * b.X;
		}

		public static bool NearlyEqual(float a, float b)
		{
			return MathF.Abs(a - b) < PlanarMath.negligibleAmt;
		}
		public static bool NearlyEqual(PlanarVector a, PlanarVector b)
		{
			return PlanarMath.DistanceSquared(a, b) < PlanarMath.negligibleAmt * PlanarMath.negligibleAmt;
		}

		public static T GetItem<T>(List<T> array, int index)
		{
			if(index >= array.Count)
			{
				return array[index % array.Count];
			}
			else if(index < 0)
			{
				return array[index % array.Count + array.Count];
			}
			else
			{
				return array[index];
			}
		}
	}
}
