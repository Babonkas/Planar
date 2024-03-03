using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Flat;
using Flat.Graphics;
using Flat.Input;
using PlanarPhysicsEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using System.Collections;
using System.Linq;

namespace PlanarPhysicsTester
{
	public class Game1 : Game
	{

		private GraphicsDeviceManager graphics;
		private Screen screen;
		private Sprites sprites;
		private Shapes shapes;
		private Camera camera;
		private SpriteBatch _spriteBatch;
		private SpriteFont fontConsolas18;


		private PlanarWorld world;
		
		

		private List<PlanarEntity> entityList;
		private List<PlanarEntity> entityRemovalList;
		private PlanarCollider character;

		private Stopwatch watch;

		private double totalWorldStepTime = 0d;
		private int totalBodyCount = 0;
		private int totalSampleCount = 0;
		private Stopwatch sampleTimer = new Stopwatch();

		private string worldStepTimeString = string.Empty;
		private string bodyCountString = string.Empty;


		//Creator


		public Game1()
		{
			

			this.graphics = new GraphicsDeviceManager(this);
			this.graphics.SynchronizeWithVerticalRetrace = true;

			this.Content.RootDirectory = "Content";
			this.IsMouseVisible = true;
			this.IsFixedTimeStep = true;

			const double UpdatesPerSecond = 60d;
			this.TargetElapsedTime = TimeSpan.FromTicks((long)Math.Round((double)TimeSpan.TicksPerSecond / UpdatesPerSecond));
		}

		protected override void Initialize()
		{
			this.Window.Position = new Point(10, 40);
			// TODO: Add your initialization logic here

			FlatUtil.SetRelativeBackBufferSize(this.graphics, 0.85f);

			this.screen = new Screen(this, 1280, 768);
			this.sprites = new Sprites(this);
			this.shapes = new Shapes(this);
			this.camera = new Camera(this.screen);
			this.camera.Zoom = 20;

			this.camera.GetExtents(out float left, out float right, out float bottom, out float top);


			this.entityList = new List<PlanarEntity>();
			this.entityRemovalList = new List<PlanarEntity>();

			this.world = new PlanarWorld();


			float padding = MathF.Abs(right - left) * 0.1f;

			PlanarRigidbody ground = new PlanarRigidbody(new PlanarBoxCollider(50f, 2f, 0.6f, 0.4f), 1f, 0.5f, 1f, true);
			ground.MoveTo(new PlanarVector(0f, -10f));
			world.AddBody(ground);
			entityList.Add(new PlanarEntity(ground));


			//PlanarRigidbody staticBox = new PlanarRigidbody(new PlanarBoxCollider(9f, 9f, 0.6f, 0.4f), 1f, 0.5f, 1f, true);
			//staticBox.MoveTo(new PlanarVector(0f, 5.5f));
			//staticBox.RotateTo(MathF.PI/4);
			//world.AddBody(staticBox);
			//entityList.Add(new PlanarEntity(staticBox));

			PlanarRigidbody circle = new PlanarRigidbody(new PlanarCircleCollider(1, 0.0f, 0.0f), 15f, 0.25f, 1f, false);
			circle.MoveTo(new PlanarVector(-20f, 20f));
			world.AddBody(circle);
			entityList.Add(new PlanarEntity(circle));

			PlanarRigidbody polygonStatic = new PlanarRigidbody(new PlanarPolygonCollider(
				new PlanarVector[] { 
					new PlanarVector(-3f, 2f), 
					new PlanarVector(3f, 0), 
					new PlanarVector(-3f, 0f) }, 
				0.6f, 0.4f), 15, 0.0f, 1f, true);
			polygonStatic.MoveTo(new PlanarVector(-20f, 10f));
			world.AddBody(polygonStatic);
			entityList.Add(new PlanarEntity(polygonStatic));

			PlanarRigidbody box = new PlanarRigidbody(new PlanarBoxCollider(5f, 1f, 0.4f, 0.6f), 1f, 1f, 1f, true);
			box.MoveTo(new PlanarVector(-8f, -8.5f));
			world.AddBody(box);
			entityList.Add(new PlanarEntity(box));

			//PlanarRigidbody staticCircle = new PlanarRigidbody(new PlanarCircleCollider(2, 0.6f, 0.4f), 1f, 0.5f, 1f, true);
			////circle.MoveTo(new PlanarVector(2f, 20f));
			//world.AddBody(staticCircle);
			//entityList.Add(new PlanarEntity(staticCircle));






			this.watch = new Stopwatch();

			this.sampleTimer.Start();

			base.Initialize();
		}
		
		

		protected override void LoadContent()
		{
			this.fontConsolas18 = this.Content.Load<SpriteFont>("Consolas18");
			_spriteBatch = new SpriteBatch(GraphicsDevice);

			// TODO: use this.Content to load your game content here
		}

		protected override void Update(GameTime gameTime)
		{
			FlatKeyboard keyboard = FlatKeyboard.Instance;
			FlatMouse mouse = FlatMouse.Instance;

			keyboard.Update();
			mouse.Update();


			
			if (mouse.IsLeftMouseButtonPressed())
			{
				
			}


			if (keyboard.IsKeyAvailable)
			{

				if (keyboard.IsKeyClicked(Keys.Escape))
				{
					this.Exit();
				}
				if (keyboard.IsKeyDown(Keys.A))
				{
					this.camera.IncZoom();

					//character.Move(new PlanarVector(-1, 0));
				}
				if (keyboard.IsKeyDown(Keys.Z))
				{
					this.camera.DecZoom();
				}


			}

			if(this.sampleTimer.Elapsed.TotalSeconds > 1d)
			{
				this.bodyCountString = "BodyCount: " + Math.Round(this.totalBodyCount / (double)this.totalSampleCount, 4).ToString();
				this.worldStepTimeString = "StepTime" + Math.Round(this.totalWorldStepTime / (double)totalSampleCount, 4).ToString();
				this.totalBodyCount = 0;
				this.totalWorldStepTime = 0d;
				this.totalSampleCount = 0;
				this.sampleTimer.Restart();
			}


			this.watch.Restart();
			this.world.Step(FlatUtil.GetElapsedTimeInSeconds(gameTime), 36);
			this.watch.Stop();

			this.totalWorldStepTime += this.watch.Elapsed.TotalMilliseconds;
			this.totalBodyCount += this.world.BodyCount;
			this.totalSampleCount++;

			 
			this.camera.GetExtents(out _, out _, out float viewBottom, out _);

			

			for (int i = 0; i < this.entityList.Count; i++)
			{
				PlanarEntity entity = this.entityList[i];
				PlanarRigidbody body = entity.Body;

				if (body.IsStatic)
				{
					continue;
				}

				PlanarAABB box = body.Collider.AABB;

				if (box.Max.Y < viewBottom)
				{
					this.entityRemovalList.Add(entity);

				}
			}
			//Console.WriteLine(entityRemovalList.Count);
			for(int i = 0; i < this.entityRemovalList.Count; i++)
			{
				//Console.WriteLine("removing");
				PlanarEntity entity = this.entityRemovalList[i];
				this.world.RemoveBody(entity.Body);
				this.entityList.Remove(entity);
			}

			// TODO: Add your update logic here
			this.entityRemovalList.Clear();
			base.Update(gameTime);
		}
		protected override void Draw(GameTime gameTime)
		{
			// TODO: Add your drawing code here

			this.screen.Set();
			this.GraphicsDevice.Clear(new Color(50, 60, 70));



			this.shapes.Begin(this.camera);

		

			for(int i = 0; i < this.entityList.Count; i++)
			{
				this.entityList[i].Draw(this.shapes);
			}

			Vector2 bodyCountStringSize = this.fontConsolas18.MeasureString(this.bodyCountString);
			Vector2 worldStepStringSize = this.fontConsolas18.MeasureString(this.worldStepTimeString);

			this.sprites.Begin();
			this.sprites.DrawString(this.fontConsolas18, this.bodyCountString, new Vector2(0, 0), Color.White);
			this.sprites.DrawString(this.fontConsolas18, this.worldStepTimeString, new Vector2(0, bodyCountStringSize.Y), Color.White);
			this.sprites.End();

#if false
			List<PlanarVector> contactPoints = world.contactPointsList;
			Console.WriteLine(contactPoints.Count);
			for(int i = 0; i < contactPoints.Count; i++)
			{
				shapes.DrawBox(PlanarConverter.ToVector2(contactPoints[i]), 0.5f, 0.5f, Color.Red);
			}
#endif
			this.shapes.End();

			this.screen.Unset();
			this.screen.Present(this.sprites);

			
			base.Draw(gameTime);
		}
	}
}