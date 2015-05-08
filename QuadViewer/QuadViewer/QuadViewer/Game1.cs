using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using Quadrotor_Control;


namespace QuadViewer
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Microsoft.Xna.Framework.Game
    {
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;

        Model quad;
        float aspectRatio;
        float rotVar = 0;

        bool fullScreenToggled = false;

        int rotationOrder;

        // Set the position of the camera in world space, for our view matrix.
        Vector3 cameraPosition = new Vector3(0.0f, 300.0f, 600.0f);
        Vector3 modelCenter;

        SpriteFont font;
        Vector2 fontPos;

        CommunicationControl comm;
        List<String> ports;

        int height = 1280;
        int width = 800;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";


            graphics.PreferredBackBufferHeight = height;
            graphics.PreferredBackBufferWidth = width;
        }

        /// <summary>
        /// Allows the game to perform any initialization it needs to before starting to run.
        /// This is where it can query for any required services and load any non-graphic
        /// related content.  Calling base.Initialize will enumerate through any components
        /// and initialize them as well.
        /// </summary>
        protected override void Initialize()
        {
            // TODO: Add your initialization logic here
            comm = new CommunicationControl();
            ports = new List<String>();

            foreach (String port in Bluetooth.GetPortNames())
            {
                ports.Add(port);
            }

            this.graphics.ToggleFullScreen();

            base.Initialize();
        }

        /// <summary>
        /// LoadContent will be called once per game and is the place to load
        /// all of your content.
        /// </summary>
        protected override void LoadContent()
        {
            // Create a new SpriteBatch, which can be used to draw textures.
            spriteBatch = new SpriteBatch(GraphicsDevice);

            // TODO: use this.Content to load your game content here
            font = Content.Load<SpriteFont>("Arial");

            quad = Content.Load<Model>("quadcopter");

            aspectRatio = graphics.GraphicsDevice.Viewport.AspectRatio;

            fontPos = new Vector2(graphics.GraphicsDevice.Viewport.Width / 2,
                graphics.GraphicsDevice.Viewport.Height / 2);

        }

        /// <summary>
        /// UnloadContent will be called once per game and is the place to unload
        /// all content.
        /// </summary>
        protected override void UnloadContent()
        {
            // TODO: Unload any non ContentManager content here
        }

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input, and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            // Allows the game to exit
            if (Keyboard.GetState().IsKeyDown(Keys.Escape))
                this.Exit();

            if (!fullScreenToggled && Keyboard.GetState().IsKeyDown(Keys.F4))
            {
                graphics.ToggleFullScreen();
                fullScreenToggled = true;
            }
            if (fullScreenToggled && Keyboard.GetState().IsKeyUp(Keys.F4))
            {
                fullScreenToggled = false;
            }

            if (!comm.IsOpen)
            {
                for (int i = 0; i < ports.Count; i++)
                {
                    if (Keyboard.GetState().IsKeyDown(Keys.D1 + i))
                    {
                        try
                        {
                            comm.OpenPort(ports[i], null);

                        }
                        catch (Exception ex)
                        {
                            System.Windows.Forms.MessageBox.Show(String.Format("{0}: {1}", ex.GetType().Name, ex.Message));
                            comm.Close();
                        }
                    }
                }
            }
            else
            {
                for (int i=0; i<6; i++)
                {
                    if (Keyboard.GetState().IsKeyDown(Keys.D0 + i))
                        rotationOrder = i;
                }
            }
            


            // TODO: Add your update logic here

            base.Update(gameTime);
        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            if (comm.IsOpen)
                GraphicsDevice.Clear(Color.DarkGray);
            else
                GraphicsDevice.Clear(Color.Black);

            spriteBatch.Begin();


            GraphicsDevice.BlendState = BlendState.Opaque;
            GraphicsDevice.DepthStencilState = DepthStencilState.Default;
            GraphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;


            // TODO: Add your drawing code here
            if (!comm.IsOpen)
            {
                for (int i = 0; i < ports.Count; i++ )
                {
                    string portName = ports[i];
                    Vector2 FontOrigin = font.MeasureString(portName) / 2;
                    // Draw the string
                    spriteBatch.DrawString(font, portName, new Vector2(fontPos.X, 100+i * 50), Color.LightGreen,
                        0, FontOrigin, 1.0f, SpriteEffects.None, 0.5f);
                }
            }
            else 
            {

                rotVar+=.01f;
                            // Copy any parent transforms.
                Matrix[] transforms = new Matrix[quad.Bones.Count];
                quad.CopyAbsoluteBoneTransformsTo(transforms);

               
                Matrix angleRot = Matrix.CreateFromAxisAngle(Vector3.UnitY, comm.rot.Y);
                angleRot *= Matrix.CreateFromAxisAngle(Vector3.UnitZ, comm.rot.Z);
                angleRot *= Matrix.CreateFromAxisAngle(Vector3.UnitX, comm.rot.X);



                Vector3 meshColor = Vector3.Zero;
                // Draw the model. A model can have multiple meshes, so loop.
                foreach (ModelMesh mesh in quad.Meshes)
                {
                    if (mesh == quad.Meshes[1]) meshColor = Color.Black.ToVector3();
                    if (mesh == quad.Meshes[0]) meshColor = Color.OldLace.ToVector3();
                    if (mesh == quad.Meshes[2]) meshColor = Color.Red.ToVector3();

                    // This is where the mesh orientation is set, as well 
                    // as our camera and projection.
                    foreach (BasicEffect effect in mesh.Effects)
                    {
                        effect.SpecularColor = meshColor;
                        effect.EmissiveColor = meshColor;
                        effect.EnableDefaultLighting();
                        effect.World = transforms[mesh.ParentBone.Index] *
                            Matrix.CreateTranslation(modelCenter) * 
                            angleRot *
                            Matrix.CreateTranslation(comm.pos*50);
                        effect.View = Matrix.CreateLookAt(cameraPosition,
                            Vector3.Zero, Vector3.Up);
                        effect.Projection = Matrix.CreatePerspectiveFieldOfView(
                            MathHelper.ToRadians(45.0f), aspectRatio,
                            1.0f, 10000.0f);
                    }
                    // Draw the mesh, using the effects set above.
                    //if (quad.Meshes[2]!=mesh)
                        mesh.Draw();
                }
            }

            spriteBatch.End();
            base.Draw(gameTime);
        }

        public void Exit()
        {
            comm.Close();
            base.Exit();
        }
    }
}
