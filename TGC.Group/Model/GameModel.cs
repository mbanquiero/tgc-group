using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using System.Collections.Generic;
using System.Drawing;
using TGC.Core.Camara;
using TGC.Core.Direct3D;
using TGC.Core.Example;
using TGC.Core.Input;
using TGC.Core.Mathematica;
using TGC.Core.SceneLoader;
using TGC.Core.Textures;
using TGC.Examples.Bullet.Physics;
using TGC.Group.Camara;
using TGC.Group.Terrain;

namespace TGC.Group.Model
{
    /// <summary>
    ///     Ejemplo para implementar el TP.
    ///     Inicialmente puede ser renombrado o copiado para hacer más ejemplos chicos, en el caso de copiar para que se
    ///     ejecute el nuevo ejemplo deben cambiar el modelo que instancia GameForm <see cref="Form.GameForm.InitGraphics()" />
    ///     line 97.
    /// </summary>

    public struct Level
    {
        public string name;
        public string heighmap;
        public string colormap;
        public string vegmap;
        public float scaleXZ;
        public float scaleY;
        public TGCVector2 start_pt;
        public TGCVector2 end_pt;
        public int nivelPiso;
    };

    public class GameModel : TGCExample
    {

        public float myElapsedTime = 0;
        //Terreno
        public TgcSimpleTerrain terrain;
        // vegetacion       
        public TgcMesh poste;
        public TgcMesh arbol;
        public TgcMesh pino;
        public List<TGCVector2> lst_arboles = new List<TGCVector2>();
        public List<TGCVector2> lst_pinos = new List<TGCVector2>();
        public TgcSkyBox skyBox;
        public bool []keys = new bool[256];

        // niveles
        public Level[] niveles = new Level[3];
        public int level = 0;

        //Fisica
        private Physics physicsExample = null;
        public bool fps_camara = true;

        // camara
        public float []cam_dist =      { 4.3f, 8.9f, 2.5f, 2.5f };
        public float []cam_altura =    { 2.0f, 4.5f, 1.0f, 1.0f };
        public float []cam_desf_lat =  { 0.3f, 0.3f, 1.5f, -1.5f};
        public int num_cam = 0;
        public TgcFpsCamera cam_fps;
        public TgcRotationalCamera cam_rot;
        public TGCVector3 LookAt = new TGCVector3();
        public TGCVector3 LookFrom = new TGCVector3();
        public TGCVector3 desiredLookAt = new TGCVector3();
        public TGCVector3 desiredLookFrom = new TGCVector3();
        public bool camFlag = true;

        public Sprite sprite;
        public TgcTexture tx_intro , tx_niveles , tx_paso , tx_timer , tx_troncos, tx_refresh,tx_menu;
        public Microsoft.DirectX.Direct3D.Font font;

        public int state = 0;
        public float timer_llegada = 0;
        public float timer_level = 0;
        public float timer_2 = 0;

        /// <summary>
        ///     Constructor del juego.
        /// </summary>
        /// <param name="mediaDir">Ruta donde esta la carpeta con los assets</param>
        /// <param name="shadersDir">Ruta donde esta la carpeta con los shaders</param>
        public GameModel(string mediaDir, string shadersDir) : base(mediaDir, shadersDir)
        {
            Category = Game.Default.Category;
            Name = Game.Default.Name;
            Description = Game.Default.Description;
        }

        public override void Init()
        {
            //Device de DirectX para crear primitivas.
            var d3dDevice = D3DDevice.Instance.Device;

            // texturas
            sprite = new Sprite(d3dDevice);
            tx_intro = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "pan_intro.png");
            tx_niveles = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "pan_niveles.png");
            tx_paso = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "pan_paso.png");
            tx_timer = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "timer.png");
            tx_troncos = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "troncos.png");
            tx_refresh = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "refresh.png");
            tx_menu = TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "menu.png");

            // Fonts
            font = new Microsoft.DirectX.Direct3D.Font(d3dDevice, 40, 0, FontWeight.ExtraBold, 0, false, CharacterSet.Default,
                    Precision.Default, FontQuality.Default, PitchAndFamily.DefaultPitch, "Comic Sans MS");
            font.PreloadGlyphs('0', '9');
            font.PreloadGlyphs('a', 'z');
            font.PreloadGlyphs('A', 'Z');


            //Utilizando esta propiedad puedo activar el update/render a intervalos constantes.
            FixedTickEnable = false;
            //Se puede configurar el tiempo en estas propiedades TimeBetweenUpdates y TimeBetweenRenders, por defecto esta puedo en 1F / FPS_60 es a lo minimo que deberia correr el TP.
            //De no estar a gusto como se ejecuta el metodo Tick (el que maneja el GameLoop) el mismo es virtual con lo cual pueden sobrescribirlo.

            // camaras
            cam_fps = new TgcFpsCamera(Input);
            cam_rot = new TgcRotationalCamera(new TGCVector3(0, 0, 0), 9, Input);
            Camera = fps_camara ? (Core.Camara.TgcCamera)cam_fps : (Core.Camara.TgcCamera)cam_rot;

            // niveles 
            niveles[0] = new Level();
            niveles[0].name = "Level 1";
            niveles[0].heighmap = "Heightmap5.jpg";
            niveles[0].colormap = "grass2.jpg";
            niveles[0].vegmap = "";
            niveles[0].scaleY = 8 * 1f / 256f;
            niveles[0].scaleXZ = 0.5f;
            niveles[0].start_pt = new TGCVector2(36, -94);
            niveles[0].nivelPiso = 2;
            niveles[0].end_pt = new TGCVector2(-30, 75);
            //niveles[0].start_pt = new TGCVector2(-30, 68);

            niveles[1] = new Level();
            niveles[1].name = "Level 2";
            niveles[1].heighmap = "test3.jpg";
            niveles[1].colormap = "grass2.jpg";
            niveles[1].vegmap = "vegMap.png";
            niveles[1].scaleY = -6 * 1f / 256f;
            niveles[1].scaleXZ = 1f;
            niveles[1].start_pt = new TGCVector2(57 - 64, 67 - 64);
            niveles[1].end_pt = new TGCVector2(84 - 64, 44 - 64);
            niveles[1].nivelPiso = 0;



            niveles[2] = new Level();
            niveles[2].name = "Level 3";
            niveles[2].heighmap = "Heightmap6.jpg";
            niveles[2].colormap = "colormap6.jpg";
            niveles[2].vegmap = "";
            niveles[2].scaleY = 60 * 1f / 256f;
            niveles[2].scaleXZ = 1;
            niveles[2].start_pt = new TGCVector2(85-256, 95-256);
            niveles[2].nivelPiso = 20;

            level = 0;

            //Assets varios
            terrain = new TgcSimpleTerrain();
            var loader = new TgcSceneLoader();
            arbol = loader.loadSceneFromFile(MediaDir + "ArbolSelvatico2\\ArbolSelvatico2-TgcScene.xml").Meshes[0];
            pino = loader.loadSceneFromFile(MediaDir + "SketchUp\\arbol2-TgcScene.xml").Meshes[0];
            poste = loader.loadSceneFromFile(MediaDir + "PosteDeLuz\\Poste de luz-TgcScene.xml").Meshes[0];

            skyBox = new TgcSkyBox();
            skyBox.Center = TGCVector3.Empty;
            skyBox.Size = new TGCVector3(10000, 10000, 10000);
            var texturesPath = MediaDir + "Texturas\\Skybox\\";
            skyBox.setFaceTexture(TgcSkyBox.SkyFaces.Up, texturesPath + "skybox2.png");
            skyBox.Init();



        }

        public void loadLevel()
        {

            terrain.create(MediaDir + "Heighmaps\\" + niveles[level].heighmap,
                    niveles[level].scaleXZ, 
                    niveles[level].scaleY, 
                    TGCVector3.Empty,           // posicion
                    MediaDir + "Heighmaps\\" + niveles[level].colormap,
                    MediaDir + "Heighmaps\\" + "grass3.jpg",
                    MediaDir + "Heighmaps\\" + "stones.jpg",
                    MediaDir + "Heighmaps\\" + "tierra.jpg",
                    niveles[level].nivelPiso);

            lst_arboles.Clear();
            lst_pinos.Clear();
            if (niveles[level].vegmap!="")
            {
                var bitmap = (Bitmap)Image.FromFile(MediaDir + "Heighmaps\\"+ niveles[level].vegmap);
                var width = bitmap.Size.Width;
                var height = bitmap.Size.Height;
                var flag = false;
                for (var i = 0; i < width; i++)
                {
                    for (var j = 0; j < height; j++)
                    {
                        //(j, i) invertido para primero barrer filas y despues columnas
                        var pixel = bitmap.GetPixel(j, i);
                        if (pixel.R == 255 && pixel.G == 0 && pixel.B == 255)
                        {
                            if (flag)
                                lst_arboles.Add(new TGCVector2(i - width / 2, j - height / 2));
                            else
                                lst_pinos.Add(new TGCVector2(i - width / 2, j - height / 2));
                            flag = !flag;
                        }

                    }
                }
                bitmap.Dispose();
            }


            // cargo la fisica
            if (physicsExample != null)
            {
                physicsExample.Dispose();
            }

            physicsExample = new Physics(this);
            var x = niveles[level].start_pt.X;
            var z = niveles[level].start_pt.Y;
            var y = terrain.height(x, z) + 0.1f;
            physicsExample.pos_camion = new TGCVector3(x, y, z);
            x = niveles[level].end_pt.X;
            z = niveles[level].end_pt.Y;
            y = terrain.height(x, z);
            physicsExample.pos_final = new TGCVector3(x, y, z);

            physicsExample.SetTriangleDataVB(terrain.getData());
            physicsExample.Init(MediaDir);


        }

        /// <summary>
        ///     Se llama en cada frame.
        ///     Se debe escribir toda la lógica de computo del modelo, así como también verificar entradas del usuario y reacciones
        ///     ante ellas.
        /// </summary>
        public override void Update()
        {
            PreUpdate();
            
            switch(state)
            {
                case 0:
                    // pantalla intro
                    {
                        // tx_intro.Width = 1024 por el tema de las potencias de 2. 
                        float ex = (float)D3DDevice.Instance.Width / (float)835;
                        float ey = (float)D3DDevice.Instance.Height / (float)469;
                        Rectangle btn_next = new Rectangle((int)(685 * ex), (int)(375 * ey), (int)(110 * ex), (int)(50 * ey));
                        if (btn_next.Contains((int)Input.Xpos, (int)Input.Ypos))
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Hand;
                            if (Input.buttonDown(0))
                                state = 1;
                        }
                        else
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Default;
                        }
                    }
                    break;

                case 1:
                    // pantalla nivel
                    {
                        float ex = (float)D3DDevice.Instance.Width / (float)832;
                        float ey = (float)D3DDevice.Instance.Height / (float)465;
                        Rectangle[] btn_level = {
                                new Rectangle((int)(191 * ex), (int)(114 * ey), (int)(65 * ex), (int)(65 * ey)),
                                new Rectangle((int)(291 * ex), (int)(114 * ey), (int)(65 * ex), (int)(65 * ey)),
                                new Rectangle((int)(388 * ex), (int)(114 * ey), (int)(65 * ex), (int)(65 * ey)) };

                        bool cursor_default = true;
                        for (int i=0;i<3 && cursor_default;++i)
                        if (btn_level[i].Contains((int)Input.Xpos, (int)Input.Ypos))
                        {
                            cursor_default = false;
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Hand;
                            if (Input.buttonDown(0))
                            {
                                state = 10;
                                level = i;
                                // cargo el nivel
                                loadLevel();

                            }
                        }

                        if(cursor_default)
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Default;

                    }
                    break;

                case 2:
                    // pantalla llegada
                    timer_2 += ElapsedTime;
                    if (timer_2>2)
                    {
                        // paso al siguiente nivel
                        state = 10;
                        level = (level + 1) % 3;
                        timer_level = 0;
                        loadLevel();
                    }
                    break;



                default:
                    // game
                    {
                        //RenderImage(tx_refresh, W - 150, H - 100, 60, 60);
                        //RenderImage(tx_menu, 10, H - 100, 60, 60);
                        int W = D3DDevice.Instance.Width;
                        int H = D3DDevice.Instance.Height;
                        //todo ESCALAS
                        float ex = 1;
                        float ey = 1;
                        Rectangle btn_refresh = new Rectangle((int)(W-150*ex), (int)(H-100*ex), 
                                (int)(60 * ex), (int)(60 * ey));
                        if (btn_refresh.Contains((int)Input.Xpos, (int)Input.Ypos))
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Hand;
                            if (Input.buttonDown(0))
                            {
                                // recargo el nivel
                                loadLevel();
                            }
                        }
                        else
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Default;
                        }

                        Rectangle btn_menu = new Rectangle((int)(10 * ex), (int)(H - 100 * ex),
                                (int)(60 * ex), (int)(60 * ey));
                        if (btn_menu.Contains((int)Input.Xpos, (int)Input.Ypos))
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Hand;
                            if (Input.buttonDown(0))
                            {
                                // vuelvo al menu
                                state = 1;
                            }
                        }
                        else
                        {
                            System.Windows.Forms.Cursor.Current = System.Windows.Forms.Cursors.Default;
                        }


                        const float fps = 1.0f / 60.0f;
                        myElapsedTime += ElapsedTime;
                        timer_level += myElapsedTime;

                        if (Input.keyDown(Key.F))
                        {
                            keys['F'] = true;
                        }
                        else
                        {
                            if (keys['F'])
                                fps_camara = !fps_camara;
                            keys['F'] = false;
                        }
                        if (Input.keyDown(Key.M))
                        {
                            keys['M'] = true;
                        }
                        else
                        {
                            if (keys['M'])
                            {
                                level = (level + 1) % 3;
                                loadLevel();
                            }
                            keys['M'] = false;
                        }

                        if (myElapsedTime >= fps)
                        {
                            myElapsedTime -= fps;
                            physicsExample.Update(Input);



                            Camera = fps_camara ? (Core.Camara.TgcCamera)cam_fps : (Core.Camara.TgcCamera)cam_rot;

                            if (fps_camara)
                            {
                                /*
                                if (Input.keyDown(Key.A))
                                {
                                    cam_dist += 0.1f;
                                }
                                else
                                if (Input.keyDown(Key.S))
                                {
                                    cam_dist -= 0.1f;
                                }

                                if (Input.keyDown(Key.W))
                                {
                                    cam_altura += 0.1f;
                                }
                                else
                                if (Input.keyDown(Key.Z))
                                {
                                    cam_altura -= 0.1f;
                                }

                                if (Input.keyDown(Key.D))
                                {
                                    cam_desf_lat += 0.1f;
                                }
                                else
                                if (Input.keyDown(Key.F))
                                {
                                    cam_desf_lat -= 0.1f;
                                }*/

                                if (Input.keyDown(Key.C))
                                {
                                    num_cam = (num_cam+1)%4;
                                }


                                var pos = new TGCVector3(physicsExample.carChassis.CenterOfMassPosition);
                                BulletSharp.Math.Matrix tr;
                                physicsExample.carChassis.GetWorldTransform(out tr);
                                TGCMatrix m_tr = new TGCMatrix(tr);
                                TGCVector3 vel = TGCVector3.TransformNormal(new TGCVector3(0, 0, 1), m_tr);
                                vel.Y = 0;
                                vel.Normalize();
                                TGCVector3 tan = TGCVector3.TransformNormal(new TGCVector3(1, 0, 0), m_tr);
                                tan.Y = 0;
                                tan.Normalize();
                                vel = vel + tan * cam_desf_lat[num_cam];

                                desiredLookAt = pos + vel * cam_dist[num_cam];
                                desiredLookFrom = pos - vel * cam_dist[num_cam] + new TGCVector3(0, cam_altura[num_cam], 0);


                                var t = camFlag ? 1 : (desiredLookFrom - LookFrom).Length() * 0.01f;
                                camFlag = false;
                                //t = 1;
                                LookFrom = desiredLookFrom * t + LookFrom * (1 - t);
                                LookAt = desiredLookAt * t + LookAt * (1 - t);
                                /*
                                LookFrom = new TGCVector3(physicsExample.camera.CenterOfMassPosition);
                                LookAt = LookFrom + new TGCVector3(physicsExample.camera.Orientation.Axis);
                                */
                                Camera.SetCamera(LookFrom, LookAt);

                                // chequeo si llego al final
                                if (physicsExample.flag_llegada)
                                {
                                    timer_llegada += ElapsedTime;
                                    if(timer_llegada>0.5f)
                                    {
                                        // pantalla de paso de nivel
                                        state = 2;
                                        timer_2 = 0;
                                    }
                                }
                                else
                                {
                                    timer_llegada = 0;
                                }
                            }
                        }
                    }
                    break;
            }
            PostUpdate();

        }

        /// <summary>
        ///     Se llama cada vez que hay que refrescar la pantalla.
        ///     Escribir aquí todo el código referido al renderizado.
        ///     Borrar todo lo que no haga falta.
        /// </summary>
        public override void Render()
        {

            //Inicio el render de la escena, para ejemplos simples. Cuando tenemos postprocesado o shaders es mejor realizar las operaciones según nuestra conveniencia.
            PreRender();

            switch(state)
            {
                case 0:
                    RenderPantalla(tx_intro);
                    break;
                case 1:
                    RenderPantalla(tx_niveles);
                    break;
                case 2:
                    RenderPantalla(tx_paso);
                    break;
                default:
                    RenderScene();
                    break;
            }

            //Finaliza el render y presenta en pantalla, al igual que el preRender se debe para casos puntuales es mejor utilizar a mano las operaciones de EndScene y PresentScene
            PostRender();
        }

        public void RenderScene()
        {
            int W = D3DDevice.Instance.Width;
            int H = D3DDevice.Instance.Height;

            // debug position
            var x = physicsExample.vehicle.RigidBody.CenterOfMassPosition.X;
            var y = physicsExample.vehicle.RigidBody.CenterOfMassPosition.Y;
            var z = physicsExample.vehicle.RigidBody.CenterOfMassPosition.Z;
            //DrawText.drawText("X=" + x + "Y= " + y + " Z=" + z, 0, 20, Color.OrangeRed);

            sprite.Begin(SpriteFlags.AlphaBlend);
            sprite.Transform = TGCMatrix.Identity;

            font.DrawText(sprite, "" + physicsExample.cant_troncos_ok,
                new Rectangle(90, 10, 0, 0), DrawTextFormat.NoClip | DrawTextFormat.Top | DrawTextFormat.Left, Color.Beige);
            if (physicsExample.flag_llegada)
            {
                int left = 1 + (int)((0.5f - timer_llegada) * 10);
                font.DrawText(sprite, "Descargando. Restan " + left + " s",
                    new Rectangle(0, 0, W, H), DrawTextFormat.NoClip | DrawTextFormat.VerticalCenter | DrawTextFormat.Center, Color.Beige);
            }

            int time = (int)(timer_level/100);
            font.DrawText(sprite, "" + time,
                          new Rectangle(W-50, 10, 0, 0), DrawTextFormat.NoClip | DrawTextFormat.Top | DrawTextFormat.Right, Color.Beige);
            sprite.End();
            RenderImage(tx_timer, W - 150, 10, 60, 60);
            RenderImage(tx_troncos, 10, 10, 90, 45);

            RenderImage(tx_refresh, W - 150, H-100, 60, 60);
            RenderImage(tx_menu, 10, H - 100, 60, 60);


            //if (doRender)
            {
                skyBox.Render();
                terrain.Render();
                physicsExample.Render();

                var rnd = new System.Random(1);
                D3DDevice.Instance.Device.SetSamplerState(0, SamplerStageStates.AddressU, (int)TextureAddress.Clamp);
                D3DDevice.Instance.Device.SetSamplerState(0, SamplerStageStates.AddressV, (int)TextureAddress.Clamp);

                foreach (var pos in lst_arboles)
                {
                    float dk = rnd.Next(-15, 15) / 1000.0f;
                    TGCMatrix S = TGCMatrix.Scaling(0.1f + dk, 0.1f + dk, 0.1f + dk);
                    float X = pos.X;
                    float Z = pos.Y;
                    float Y = terrain.height(X, Z);
                    TGCVector3 p = new TGCVector3(X, Y, Z);
                    arbol.Transform = S * TGCMatrix.Translation(p);
                    arbol.Render();

                    /*
                    arbol.updateBoundingBox();

                    //Solo mostrar la malla si colisiona contra el Frustum
                    var r = TgcCollisionUtils.classifyFrustumAABB(Frustum, arbol.BoundingBox);
                    if (r != TgcCollisionUtils.FrustumResult.OUTSIDE)
                    {
                        arbol.Render();
                    }*/
                }

                D3DDevice.Instance.Device.SetSamplerState(0, SamplerStageStates.AddressU, (int)TextureAddress.Wrap);
                D3DDevice.Instance.Device.SetSamplerState(0, SamplerStageStates.AddressV, (int)TextureAddress.Wrap);
                pino.AlphaBlendEnable = true;
                foreach (var pos in lst_pinos)
                {
                    float dk = rnd.Next(-15, 15) / 1000.0f;
                    TGCMatrix S = TGCMatrix.Scaling(0.06f + dk, 0.06f + dk, 0.06f + dk);
                    float X = pos.X;
                    float Z = pos.Y;
                    float Y = terrain.height(X, Z) + 2;
                    TGCVector3 p = new TGCVector3(X, Y, Z);
                    pino.Transform = S * TGCMatrix.Translation(p);
                    pino.Render();
                }

                {
                    TGCMatrix S = TGCMatrix.Scaling(0.06f, 0.06f, 0.06f);
                    float X = niveles[level].end_pt.X;
                    float Z = niveles[level].end_pt.Y;
                    float Y = terrain.height(X, Z) + 1;
                    TGCVector3 p = new TGCVector3(X, Y, Z);
                    poste.Transform = S * TGCMatrix.Translation(p);
                    poste.Render();
                }

            }

        }

        public void RenderPantalla(TgcTexture tx)
        {
            sprite.Begin(SpriteFlags.AlphaBlend);
            // ajusto para que ocupe toda la pantalla
            float ex = (float)D3DDevice.Instance.Width / (float)tx.Width;
            float ey = (float)D3DDevice.Instance.Height / (float)tx.Height;
            sprite.Transform = TGCMatrix.Transformation2D(new TGCVector2(0,0), 0, new TGCVector2(ex,ey),new TGCVector2(0,0),0, new TGCVector2(0,0));

            sprite.Draw(tx.D3dTexture, new Rectangle(0,0,0,0)
                , new TGCVector3(0,0, 0), new TGCVector3(0, 0, 0), 
                        Color.FromArgb(255, 255, 255, 255));
            sprite.End();

            // debug position
            /*var x = Input.Xpos;
            var y = Input.Ypos;
            DrawText.drawText("X=" + x + "Y= "+y, 0, 20, Color.OrangeRed);
            */


        }

        public void RenderImage(TgcTexture tx , int x , int y , int dx , int dy)
        {
            sprite.Begin(SpriteFlags.AlphaBlend);
            float ex = (float)dx / (float)tx.Width;
            float ey = (float)dy / (float)tx.Height;
            sprite.Transform = TGCMatrix.Transformation2D(new TGCVector2(0, 0), 0, new TGCVector2(ex, ey), new TGCVector2(0, 0), 0, new TGCVector2(x, y));

            sprite.Draw(tx.D3dTexture, new Rectangle(0, 0, 0, 0)
                , new TGCVector3(0, 0, 0), new TGCVector3(0, 0, 0),
                        Color.FromArgb(255, 255, 255, 255));
            sprite.End();
        }


        /// <summary>
        ///     Se llama cuando termina la ejecución del ejemplo.
        ///     Hacer Dispose() de todos los objetos creados.
        ///     Es muy importante liberar los recursos, sobretodo los gráficos ya que quedan bloqueados en el device de video.
        /// </summary>
        public override void Dispose()
        {
            terrain.Dispose();
            physicsExample.Dispose();
        }
    }
}