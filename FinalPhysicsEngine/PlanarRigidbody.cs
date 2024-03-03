using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public sealed class PlanarRigidbody
	{
		public PlanarCollider Collider
		{
			get;
			private set;
		}
		private PlanarVector force = PlanarVector.Zero;

		public readonly float gravityMultiplyer;
		public readonly float Mass;
		public readonly float InvMass;
		public readonly float Density;
		public readonly float Inertia;
		public readonly float InvInertia;
		public readonly float Restitution;
		public readonly bool IsStatic;

		public PlanarTransform PlanarTransform
		{
			get;
			private set;
		}

		public PlanarRigidbody(PlanarCollider collider, float density, float restitution, float gravityMultiplier, bool isStatic) 
		{
			Collider = collider;

			restitution = PlanarMath.Clamp(restitution, 0f, 1f);


			Mass = 0.0f;
			Inertia = 0.0f;

			if (!isStatic)
			{
				Mass = collider.Area * density;
				InvMass = Mass > 0f ? 1f / Mass : 0f;
				Inertia = CalculateInertia();
			}

			InvInertia = Inertia > 0f ? 1f / Inertia : 0f;
			Density = density;
			Restitution = restitution;
			gravityMultiplyer = gravityMultiplier;
			IsStatic = isStatic;

		}
		private float CalculateInertia()
		{
			if (Collider is PlanarBoxCollider boxCollider)
			{
				return (1f / 12f) * Mass * (boxCollider.Width * boxCollider.Width + boxCollider.Height * boxCollider.Height);
			}
			else if (Collider is PlanarCircleCollider circleCollider)
			{
				return (1f / 2f) * Mass * circleCollider.Radius * circleCollider.Radius;
			}
			else if (Collider is PlanarPolygonCollider polygonCollider)
			{
				PlanarVector center = PlanarVector.Zero;
				float area = 0;
				float inertia = 0;

				PlanarVector[] vertices = polygonCollider.Vertices;

				//int count = polygonCollider.VerticesCount;
				int prev = vertices.Length - 1;

				for (int i = 0; i < vertices.Length; i++)
				{
					PlanarVector a = vertices[prev];
					PlanarVector b = vertices[i];

					float area_step = PlanarMath.Cross(a, b) / 2;
					PlanarVector center_step = (a + b) / 3;
					float inertia_step = area_step * (PlanarMath.Dot(a, a) + PlanarMath.Dot(b, b) + PlanarMath.Dot(a, b)) / 6;

					center = (center * area + center_step * area_step) / (area + area_step);
					area += area_step;
					inertia += inertia_step;

					prev = i;
				}

				inertia *= Density;
				inertia -= Mass * PlanarMath.Dot(center, center);

				return inertia;
			}
			else
			{
				throw new Exception("Not valid shape type");
			}
		}

		public PlanarVector Position
		{
			get;
			set;
		}

		public PlanarVector LinearVelocity
		{
			get;
			internal set;
		}
		public float Angle
		{
			get;
			internal set;
		}
		public float AngularVelocity
		{
			get;
			internal set;
		}


		internal void Step(float time, PlanarVector gravity, int iterations)
		{
			
			PlanarTransform = new PlanarTransform(Position, Angle);

			Collider.ComputeAABB(PlanarTransform);

			if (IsStatic)
			{
				return;
			}

			time /= (float)iterations;
			//force = mass * acc
			//acc = force / mass



			PlanarVector acceleration = force / Mass;
			LinearVelocity += acceleration * time;


			LinearVelocity += gravity * time * gravityMultiplyer;
			//Console.WriteLine(LinearVelocity);

			Position += LinearVelocity * time;

			Angle += AngularVelocity * time;

			force = PlanarVector.Zero;
			//transformUpdateRequired = true;
			//aabbUpdateRequired = true;

		}
		public void Move(PlanarVector amount)
		{
			Position += amount;
		}
		public void MoveTo(PlanarVector position)
		{
			Position = position;
		}

		public void Rotate(float amount)
		{
			Angle += amount;
		}

		public void RotateTo(float angle)
		{
			Angle = angle;
		}

		public void AddForce(PlanarVector amount)
		{
			force = amount;

		}

	}
}
