using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Drawing;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PlanarPhysicsEngine
{
	public sealed class PlanarWorld
	{
		public static readonly float MinBodySize = 0.01f * 0.01f;
		public static readonly float MaxBodySize = 641f * 64f;

		public static readonly float MinDensity = 0.2f;
		public static readonly float MaxDensity = 21.4f;

		public static readonly int MinIterations = 1;
		public static readonly int MaxIterations = 128;

		private PlanarVector gravity;
		private List<PlanarRigidbody> bodyList;
		private List<(int, int)> contactPairs;

		//public List<PlanarVector> contactPointsList
		//{
		//	get;
		//	private set;
		//}

		PlanarVector[] contactList;
		PlanarVector[] impulseList;
		PlanarVector[] raList;
		PlanarVector[] rbList;
		PlanarVector[] frictionImpulseList;
		float[] jList;


		public int BodyCount
		{
			get { return bodyList.Count; }
		}

		public PlanarWorld()
		{
			contactList = new PlanarVector[2];
			impulseList = new PlanarVector[2];
			raList = new PlanarVector[2];
			rbList = new PlanarVector[2];
			frictionImpulseList = new PlanarVector[2];
			jList = new float[2];

			// 
			gravity = new PlanarVector(0f, -9.81f);
			bodyList = new List<PlanarRigidbody>();
			contactPairs = new List<(int, int)>();
			//contactPointsList = new List<PlanarVector>();

		}

		public void AddBody(PlanarRigidbody body)
		{
			bodyList.Add(body);
		}
		public bool RemoveBody(PlanarRigidbody body)
		{
			return bodyList.Remove(body);
		}
		public bool GetBody(int index, out PlanarRigidbody body)
		{
			body = null;
			if(index < 0 || index >= bodyList.Count)
			{
				return false;
			}
			body = bodyList[index];
			return true;
		}

		public void Step(float time, int totalIteration)
		{

			totalIteration = PlanarMath.Clamp(totalIteration, PlanarWorld.MinIterations, PlanarWorld.MaxIterations);
			//contactPointsList.Clear();
			for(int currentIteration = 0; currentIteration < totalIteration; currentIteration++)
			{

				contactPairs.Clear();
				StepBodies(time, totalIteration);
				//contactPointsList.Clear();
				BroadPhase();
				NarrowPhase();
				
				
			}

		}
		private void BroadPhase()
		{
			for (int i = 0; i < bodyList.Count - 1; i++)
			{
				PlanarRigidbody bodyA = bodyList[i];
				PlanarAABB bodyA_aabb = bodyA.Collider.AABB;

				for (int j = i + 1; j < bodyList.Count; j++)
				{
					PlanarRigidbody bodyB = bodyList[j];
					PlanarAABB bodyB_aabb = bodyB.Collider.AABB;

					//Console.WriteLine(bodyA.ShapeType + " : " + bodyB.ShapeType);

					if (bodyA.IsStatic && bodyB.IsStatic)
					{
						continue;
					}
					//Console.WriteLine(bodyA.ShapeType + " : " + bodyB.ShapeType);
					if (!Collisions.IntersectAABBs(bodyA_aabb, bodyB_aabb))
					{
						continue;
					}
					//Console.WriteLine(bodyA.ShapeType + " : " + bodyB.ShapeType);
					//Console.WriteLine("COLLID");
					contactPairs.Add((i, j));
				}
			}
		}

		private void NarrowPhase()
		{
			//contactManifoldList.Clear();
			for (int i = 0; i < contactPairs.Count; i++)
			{
				(int, int) pair = contactPairs[i];
				PlanarRigidbody bodyA = bodyList[pair.Item1];
				PlanarRigidbody bodyB = bodyList[pair.Item2];
				
				//Console.WriteLine("TEST");
				if (Collisions.Collide(bodyA, bodyB, out PlanarVector normal, out float depth))
				{
					//Console.WriteLine("TEST");
					SeperateBodies(bodyA, bodyB, normal * depth);
					Collisions.FindContactPoints(bodyA, bodyB, out PlanarVector contact1, out PlanarVector contact2, out int contactCount);
					PlanarManifold contact = new PlanarManifold(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);


					ResolveCollisionWithRotationAndFriction(in contact);
					//contactManifoldList.Add(contact);

#if false
					if (!contactPointsList.Contains(contact1))
					{
						contactPointsList.Add(contact1);

						if(contactCount > 1)
						{
							if (!contactPointsList.Contains(contact2))
							{
								contactPointsList.Add(contact2);
							}
						}
					}
#endif
				}


			}
			
		}

		public void StepBodies(float time, int totalIterations)
		{
			for (int i = 0; i < bodyList.Count; i++)
			{
				bodyList[i].Step(time, gravity, totalIterations);

			}
		}


		private void SeperateBodies(PlanarRigidbody bodyA, PlanarRigidbody bodyB, PlanarVector mtv)
		{
			if (bodyA.IsStatic)
			{
				bodyB.Move(mtv);
			}
			else if (bodyB.IsStatic)
			{
				bodyA.Move(-mtv);
			}
			else
			{
				bodyA.Move(-mtv / 2f);
				bodyB.Move(mtv / 2f);
			}

		}

		public void ResolveCollisionBasic(in PlanarManifold contact)
		{
			PlanarRigidbody bodyA = contact.BodyA;
			PlanarRigidbody bodyB = contact.BodyB;
			PlanarVector normal = contact.Normal;
			float depth = contact.Depth;

			PlanarVector relativeVelocity = bodyB.LinearVelocity - bodyA.LinearVelocity;

			if(PlanarMath.Dot(relativeVelocity, normal) > 0f)
			{
				return;
			}

			float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

			float j = -(1 + e) * PlanarMath.Dot(relativeVelocity, normal);

			j /= bodyA.InvMass +bodyB.InvMass;

			PlanarVector impulse = j * normal;


			bodyA.LinearVelocity -= impulse * bodyA.InvMass;
			bodyB.LinearVelocity += impulse * bodyB.InvMass;
		}
		public void ResolveCollisionWithRotation(in PlanarManifold contact)
		{
			PlanarRigidbody bodyA = contact.BodyA;
			PlanarRigidbody bodyB = contact.BodyB;
			PlanarVector normal = contact.Normal;

			PlanarVector contact1 = contact.Contact1;
			PlanarVector contact2 = contact.Contact2;
			int contactCount = contact.ContactCount;

			float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

			contactList[0] = contact1;
			contactList[1] = contact2;

			for (int i = 0; i < contactCount; i++)
			{
				impulseList[i] = PlanarVector.Zero;
				raList[i] = PlanarVector.Zero;
				rbList[i] = PlanarVector.Zero;
			}



			for (int i = 0; i < contactCount; i++)
			{
				PlanarVector ra = contactList[i] - bodyA.Position;
				PlanarVector rb = contactList[i] - bodyB.Position;

				raList[i] = ra;
				rbList[i] = rb;

				PlanarVector raPerp = new PlanarVector(-ra.Y, ra.X);
				PlanarVector rbPerp = new PlanarVector(-rb.Y, rb.X);

				PlanarVector angularLineaarVelocityA = raPerp * bodyA.AngularVelocity;
				PlanarVector angularLineaarVelocityB = rbPerp * bodyB.AngularVelocity;


				PlanarVector relativeVelocity =
					(bodyB.LinearVelocity + angularLineaarVelocityB) -
					(bodyA.LinearVelocity + angularLineaarVelocityA);

				float contactVelocityMag = PlanarMath.Dot(relativeVelocity, normal);


				if (contactVelocityMag > 0f)
				{
					continue;
				}

				float raPerpDotN = PlanarMath.Dot(raPerp, normal);
				float rbPerpDotN = PlanarMath.Dot(rbPerp, normal);

				float denom = bodyA.InvMass + bodyB.InvMass +
					(raPerpDotN * raPerpDotN) * bodyA.InvInertia +
					(rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

				float j = -(1 + e) * contactVelocityMag;
				j /= denom;
				j /= (float)contactCount;

				PlanarVector impulse = j * normal;
				impulseList[i] = impulse;
			}

			for (int i = 0; i < contactCount; i++)
			{
				PlanarVector impulse = impulseList[i];
				PlanarVector ra = raList[i];
				PlanarVector rb = rbList[i];


				bodyA.LinearVelocity += -impulse * bodyA.InvMass;
				bodyA.AngularVelocity += -PlanarMath.Cross(ra, impulse) * bodyA.InvInertia;
				bodyB.LinearVelocity += impulse * bodyB.InvMass;
				bodyB.AngularVelocity += PlanarMath.Cross(rb, impulse) * bodyB.InvInertia;
			}
		}
		public void ResolveCollisionWithRotationAndFriction(in PlanarManifold contact)
		{
			PlanarRigidbody bodyA = contact.BodyA;
			PlanarRigidbody bodyB = contact.BodyB;
			PlanarVector normal = contact.Normal;

			PlanarVector contact1 = contact.Contact1;
			PlanarVector contact2 = contact.Contact2;
			int contactCount = contact.ContactCount;

			float e = MathF.Min(bodyA.Restitution, bodyB.Restitution);

			float sf = (bodyA.Collider.StaticFriction + bodyB.Collider.StaticFriction) * 0.5f;
			float df = (bodyA.Collider.DynamicFriction + bodyB.Collider.DynamicFriction) * 0.5f;

			contactList[0] = contact1;
			contactList[1] = contact2;

			for(int i = 0; i < contactCount; i++)
			{
				impulseList[i] = PlanarVector.Zero;
				raList[i] = PlanarVector.Zero;
				rbList[i] = PlanarVector.Zero;
				frictionImpulseList[i] = PlanarVector.Zero;
				jList[i] = 0;
			}



			for (int i = 0; i < contactCount; i++)
			{
				PlanarVector ra = contactList[i] - bodyA.Position;
				PlanarVector rb = contactList[i] - bodyB.Position;

				raList[i] = ra;
				rbList[i] = rb;

				PlanarVector raPerp = new PlanarVector(-ra.Y, ra.X);
				PlanarVector rbPerp = new PlanarVector(-rb.Y, rb.X);

				PlanarVector angularLineaarVelocityA = raPerp * bodyA.AngularVelocity; 
				PlanarVector angularLineaarVelocityB = rbPerp * bodyB.AngularVelocity;


				PlanarVector relativeVelocity = 
					(bodyB.LinearVelocity + angularLineaarVelocityB) - 
					(bodyA.LinearVelocity + angularLineaarVelocityA);

				float contactVelocityMag = PlanarMath.Dot(relativeVelocity, normal);


				if (contactVelocityMag > 0f)
				{
					continue;
				}

				float raPerpDotN = PlanarMath.Dot(raPerp, normal);
				float rbPerpDotN = PlanarMath.Dot(rbPerp, normal);

				float denom = bodyA.InvMass + bodyB.InvMass + 
					(raPerpDotN * raPerpDotN) * bodyA.InvInertia + 
					(rbPerpDotN * rbPerpDotN) * bodyB.InvInertia;

				float j = -(1 + e) * contactVelocityMag;
				j /= denom;
				j /= (float)contactCount;

				jList[i] = j;

				PlanarVector impulse = j * normal;
				impulseList[i] = impulse;
			}

			for(int i = 0; i < contactCount; i++)
			{
				PlanarVector impulse = impulseList[i];
				PlanarVector ra = raList[i];
				PlanarVector rb = rbList[i];


				bodyA.LinearVelocity += -impulse * bodyA.InvMass;
				bodyA.AngularVelocity += -PlanarMath.Cross(ra, impulse) * bodyA.InvInertia;
				bodyB.LinearVelocity += impulse * bodyB.InvMass;
				bodyB.AngularVelocity += PlanarMath.Cross(rb, impulse) * bodyB.InvInertia;
			}








			for (int i = 0; i < contactCount; i++)
			{
				PlanarVector ra = contactList[i] - bodyA.Position;
				PlanarVector rb = contactList[i] - bodyB.Position;

				raList[i] = ra;
				rbList[i] = rb;

				PlanarVector raPerp = new PlanarVector(-ra.Y, ra.X);
				PlanarVector rbPerp = new PlanarVector(-rb.Y, rb.X);

				PlanarVector angularLineaarVelocityA = raPerp * bodyA.AngularVelocity;
				PlanarVector angularLineaarVelocityB = rbPerp * bodyB.AngularVelocity;


				PlanarVector relativeVelocity =
					(bodyB.LinearVelocity + angularLineaarVelocityB) -
					(bodyA.LinearVelocity + angularLineaarVelocityA);

				PlanarVector tangent = relativeVelocity - PlanarMath.Dot(relativeVelocity, normal) * normal;

				if(PlanarMath.NearlyEqual(tangent, PlanarVector.Zero))
				{
					continue;
				}
				else
				{
					tangent = PlanarMath.Normalize(tangent);
				}


				float raPerpDotT = PlanarMath.Dot(raPerp, tangent);
				float rbPerpDotT = PlanarMath.Dot(rbPerp, tangent);

				float denom = bodyA.InvMass + bodyB.InvMass +
					(raPerpDotT * raPerpDotT) * bodyA.InvInertia +
					(rbPerpDotT * rbPerpDotT) * bodyB.InvInertia;

				float jt = -PlanarMath.Dot(relativeVelocity, tangent);
				jt /= denom;
				jt /= (float)contactCount;

				PlanarVector frictionImpulse;

				float j = jList[i];

				if (MathF.Abs(jt) <= j * sf)
				{
					frictionImpulse = jt * tangent;

				}
				else{
					frictionImpulse = -j * tangent * df;
				}

				 
				frictionImpulseList[i] = frictionImpulse;
			}

			for (int i = 0; i < contactCount; i++)
			{
				PlanarVector frictionImpulse = frictionImpulseList[i];
				PlanarVector ra = raList[i];
				PlanarVector rb = rbList[i];


				bodyA.LinearVelocity += -frictionImpulse * bodyA.InvMass;
				bodyA.AngularVelocity += -PlanarMath.Cross(ra, frictionImpulse) * bodyA.InvInertia;
				bodyB.LinearVelocity += frictionImpulse * bodyB.InvMass;
				bodyB.AngularVelocity += PlanarMath.Cross(rb, frictionImpulse) * bodyB.InvInertia;
			}
		}
		
	}
}
