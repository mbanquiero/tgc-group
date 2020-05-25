using Microsoft.DirectX.Direct3D;
using System;
using System.Drawing;
using TGC.Core.Direct3D;
using TGC.Core.Mathematica;
using TGC.Core.SceneLoader;
using TGC.Core.Shaders;
using TGC.Core.Textures;

namespace TGC.Group.Terrain
{
    /// <summary>
    ///     Permite crear la malla de un terreno en base a una textura de Heightmap
    /// </summary>
    public class TgcSimpleTerrain : IRenderObject
    {
        protected Effect effect;

        protected string technique;
        private Texture terrainTexture;
        private Texture terrainTexture2;
        private Texture terrainTexture3;
        private Texture colorMapTexture;
        private bool invertY;
        private int nivelPiso = 0;

        private int totalVertices;

        private VertexBuffer vbTerrain;
        private CustomVertex.PositionNormalTextured[] data;
        public float m_scaleXZ = 1;
        public float m_scaleY = 1;

        public float min_h = 256;
        public float max_h = 0;


        private float time = 0.0f;


        public static readonly VertexElement[] PositionNormalTextured_VertexElements =
        {
            new VertexElement(0, 0, DeclarationType.Float3, DeclarationMethod.Default, DeclarationUsage.Position, 0),
            new VertexElement(0, 12, DeclarationType.Float3, DeclarationMethod.Default, DeclarationUsage.Normal, 0),
            new VertexElement(0, 24, DeclarationType.Float2, DeclarationMethod.Default, DeclarationUsage.TextureCoordinate, 0),
            VertexElement.VertexDeclarationEnd
        };

        public VertexDeclaration VdecPositionNormalTextured { get; private set; }

        public TgcSimpleTerrain()
        {
            Enabled = true;
            AlphaBlendEnable = false;

            //Shader
            //effect = TGCShaders.Instance.VariosShader;
            //technique = TGCShaders.T_POSITION_TEXTURED;

            effect = TGCShaders.Instance.LoadEffect(TGCShaders.Instance.CommonShadersPath + "terrain.fx");
            technique = "RenderTerrain";

            VdecPositionNormalTextured = new VertexDeclaration(D3DDevice.Instance.Device, PositionNormalTextured_VertexElements);




        }

        /// <summary>
        ///     Devuelve la informacion de Custom Vertex Buffer del HeightMap cargado
        /// </summary>
        /// <returns>Custom Vertex Buffer de tipo PositionTextured</returns>
        public CustomVertex.PositionTextured[] getData()
        {

            CustomVertex.PositionTextured[] data_aux = new CustomVertex.PositionTextured[totalVertices];
            for (var i = 0; i < totalVertices; i++)
                data_aux[i] = new CustomVertex.PositionTextured(data[i].Position, data[i].Tu, data[i].Tv);
            return data_aux;
        }

        /// <summary>
        ///     Valor de Y para cada par (X,Z) del Heightmap
        /// </summary>
        public int[,] HeightmapData { get; private set; }

        /// <summary>
        ///     Indica si la malla esta habilitada para ser renderizada
        /// </summary>
        public bool Enabled { get; set; }

        /// <summary>
        ///     Centro del terreno
        /// </summary>
        public TGCVector3 Center { get; private set; }

        /// <summary>
        ///     Shader del mesh
        /// </summary>
        public Effect Effect
        {
            get { return effect; }
            set { effect = value; }
        }

        /// <summary>
        ///     Technique que se va a utilizar en el effect.
        ///     Cada vez que se llama a Render() se carga este Technique (pisando lo que el shader ya tenia seteado)
        /// </summary>
        public string Technique
        {
            get { return technique; }
            set { technique = value; }
        }

        public TGCVector3 Position
        {
            get { return Center; }
        }

        /// <summary>
        ///     Habilita el renderizado con AlphaBlending para los modelos
        ///     con textura o colores por vértice de canal Alpha.
        ///     Por default está deshabilitado.
        /// </summary>
        public bool AlphaBlendEnable { get; set; }

        /// <summary>
        ///     Renderiza el terreno
        /// </summary>
        public void Render()
        {
            float elapsedTime = 1.0f / 60.0f;
            if (!Enabled)
                return;

            //Textura
            effect.SetValue("texDiffuseMap", terrainTexture);
            effect.SetValue("texDiffuseMap2", terrainTexture2);
            effect.SetValue("texDiffuseMap3", terrainTexture3);
            effect.SetValue("texColorMap", colorMapTexture);

            effect.SetValue("time", time += elapsedTime);
            effect.SetValue("hmin", min_h * m_scaleY);
            effect.SetValue("hmax", max_h * m_scaleY);

            TexturesManager.Instance.clear(1);

            TGCShaders.Instance.SetShaderMatrix(effect, TGCMatrix.Identity);
            D3DDevice.Instance.Device.VertexDeclaration = VdecPositionNormalTextured;
            effect.Technique = technique;
            D3DDevice.Instance.Device.SetStreamSource(0, vbTerrain, 0);

            //Render con shader
            effect.Begin(0);
            effect.BeginPass(0);
            D3DDevice.Instance.Device.DrawPrimitives(PrimitiveType.TriangleList, 0, totalVertices / 3);
            effect.EndPass();
            effect.End();
        }

        /// <summary>
        ///     Libera los recursos del Terreno
        /// </summary>
        public void Dispose()
        {
            if (vbTerrain != null)
            {
                vbTerrain.Dispose();
            }

            if (terrainTexture != null)
            {
                terrainTexture.Dispose();
            }
            if (terrainTexture2 != null)
            {
                terrainTexture2.Dispose();
            }
            if (terrainTexture3 != null)
            {
                terrainTexture3.Dispose();
            }
        }


        public float height(float x , float z)
        {
            var width = HeightmapData.GetLength(0);
            var length = HeightmapData.GetLength(1);


            int i = (int)Math.Round(x / m_scaleXZ + width / 2.0f);
            int j = (int)Math.Round(z / m_scaleXZ + length / 2.0f);

            if (i < 0)
                i = 0;
            else
            if (i >= width)
                i = width - 1;

            if (j < 0)
                j = 0;
            else
            if (j >= length)
                j = length - 1;

            return HeightmapData[i,j] * m_scaleY;
        }

        /// <summary>
        ///     Crea la malla de un terreno en base a un Heightmap
        /// </summary>
        /// <param name="heightmapPath">Imagen de Heightmap</param>
        /// <param name="scaleXZ">Escala para los ejes X y Z</param>
        /// <param name="scaleY">Escala para el eje Y</param>
        /// <param name="center">Centro de la malla del terreno</param>
        public void loadHeightmap(string heightmapPath, float scaleXZ, float scaleY, TGCVector3 center)
        {
            Center = center;

            m_scaleXZ = scaleXZ;
            m_scaleY = scaleY;

            float tx_scale = 1;// 50f;

            //Dispose de VertexBuffer anterior, si habia
            if (vbTerrain != null && !vbTerrain.Disposed)
            {
                vbTerrain.Dispose();
            }



            //cargar heightmap
            HeightmapData = loadHeightMap(D3DDevice.Instance.Device, heightmapPath);
            var width = HeightmapData.GetLength(0);
            var length = HeightmapData.GetLength(1);

            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < length; j++)
                {

                    if (invertY)
                        HeightmapData[i, j] = 256 - HeightmapData[i, j];

                    if (HeightmapData[i, j] * scaleY < nivelPiso)
                    {
                        float s = (nivelPiso - HeightmapData[i, j] * scaleY) / nivelPiso;
                        HeightmapData[i, j] = (int)( (nivelPiso - s) / scaleY);
                    }


                    if (HeightmapData[i, j] > max_h)
                        max_h = HeightmapData[i, j];
                    if (HeightmapData[i, j] < min_h)
                        min_h = HeightmapData[i, j];
                }
            }
            
            //Crear vertexBuffer
            totalVertices = 2 * 3 * (HeightmapData.GetLength(0) - 1) * (HeightmapData.GetLength(1) - 1);
            vbTerrain = new VertexBuffer(typeof(CustomVertex.PositionNormalTextured), totalVertices,
                D3DDevice.Instance.Device,
                Usage.Dynamic | Usage.WriteOnly, CustomVertex.PositionNormalTextured.Format, Pool.Default);

            //Cargar vertices
            var dataIdx = 0;
            data = new CustomVertex.PositionNormalTextured[totalVertices];

            center.X = center.X * scaleXZ - width / 2 * scaleXZ;
            center.Y = center.Y * scaleY;
            center.Z = center.Z * scaleXZ - length / 2 * scaleXZ;


            TGCVector3 [,]N = new TGCVector3[width,length];
            for (var i = 0; i < width - 1; i++)
            {
                for (var j = 0; j < length - 1; j++)
                {
                    var v1 = new TGCVector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j] * scaleY,
                        center.Z + j * scaleXZ);
                    var v2 = new TGCVector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j + 1] * scaleY,
                        center.Z + (j + 1) * scaleXZ);
                    var v3 = new TGCVector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j] * scaleY,
                        center.Z + j * scaleXZ);
                    N[i,j] = TGCVector3.Normalize(TGCVector3.Cross(v2 - v1, v3 - v1));
                }
            }


            for (var i = 0; i < width - 1; i++)
            {
                for (var j = 0; j < length - 1; j++)
                {
                    //Vertices
                    var v1 = new TGCVector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j] * scaleY,
                        center.Z + j * scaleXZ);
                    var v2 = new TGCVector3(center.X + i * scaleXZ, center.Y + HeightmapData[i, j + 1] * scaleY,
                        center.Z + (j + 1) * scaleXZ);
                    var v3 = new TGCVector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j] * scaleY,
                        center.Z + j * scaleXZ);
                    var v4 = new TGCVector3(center.X + (i + 1) * scaleXZ, center.Y + HeightmapData[i + 1, j + 1] * scaleY,
                        center.Z + (j + 1) * scaleXZ);

                    //Coordendas de textura
                    var t1 = new TGCVector2( (float)i / (float)width, (float)j / (float)length) * tx_scale;
                    var t2 = new TGCVector2((float)i / (float)width, (float)(j + 1) / (float)length) * tx_scale;
                    var t3 = new TGCVector2((float)(i + 1) / (float)width, (float)j / (float)length) * tx_scale;
                    var t4 = new TGCVector2((float)(i + 1) / (float)width, (float)(j + 1) / (float)length) * tx_scale;

                    //Cargar triangulo 1
                    data[dataIdx] = new CustomVertex.PositionNormalTextured(v1, N[i,j], t1.X, t1.Y);
                    data[dataIdx + 1] = new CustomVertex.PositionNormalTextured(v2,N[i, j+1], t2.X, t2.Y);
                    data[dataIdx + 2] = new CustomVertex.PositionNormalTextured(v4, N[i+1, j+1], t4.X, t4.Y);

                    //Cargar triangulo 2
                    data[dataIdx + 3] = new CustomVertex.PositionNormalTextured(v1, N[i, j], t1.X, t1.Y);
                    data[dataIdx + 4] = new CustomVertex.PositionNormalTextured(v4, N[i+1, j+1], t4.X, t4.Y);
                    data[dataIdx + 5] = new CustomVertex.PositionNormalTextured(v3, N[i+1, j], t3.X, t3.Y);

                    dataIdx += 6;
                }
            }

            vbTerrain.SetData(data, 0, LockFlags.None);
        }



        public Texture createTexture(string path)
        {
            var b = (Bitmap)Image.FromFile(path);
            b.RotateFlip(RotateFlipType.Rotate90FlipX);
            return Texture.FromBitmap(D3DDevice.Instance.Device, b, Usage.AutoGenerateMipMap, Pool.Managed);
        }

        public void create(string heightMap , float scaleXZ, float scaleY, TGCVector3 center,
            string colorMap, string diffuseMap , string diffuseMap2, string diffuseMap3 , int pnivelPiso=0)
        {
            // cargo el heightmap
            nivelPiso = pnivelPiso;
            invertY = scaleY <0;
            loadHeightmap(heightMap,scaleXZ,FastMath.Abs(scaleY),center);
            // textura con el color Map
            colorMapTexture  = createTexture(colorMap) ;
            // textura diffuse
            terrainTexture = createTexture(diffuseMap);
            terrainTexture2 = createTexture(diffuseMap2);
            terrainTexture3 = createTexture(diffuseMap3);

        }


        /// <summary>
        ///     Carga la textura del terreno
        /// </summary>
        public void loadTexture(string path)
        {
            // DEPRECADA!!
            //Dispose textura anterior, si habia
            if (terrainTexture != null && !terrainTexture.Disposed)
            {
                terrainTexture.Dispose();
            }

            //Rotar e invertir textura
            var b = (Bitmap)Image.FromFile(path);
            b.RotateFlip(RotateFlipType.Rotate90FlipX);
            terrainTexture = Texture.FromBitmap(D3DDevice.Instance.Device, b, Usage.AutoGenerateMipMap, Pool.Managed);
        }

        /// <summary>
        ///     Carga los valores del Heightmap en una matriz
        /// </summary>
        protected int[,] loadHeightMap(Device d3dDevice, string path)
        {
            var bitmap = (Bitmap)Image.FromFile(path);
            var width = bitmap.Size.Width;
            var height = bitmap.Size.Height;
            var heightmap = new int[width, height];
            for (var i = 0; i < width; i++)
            {
                for (var j = 0; j < height; j++)
                {
                    //(j, i) invertido para primero barrer filas y despues columnas
                    var pixel = bitmap.GetPixel(j, i);
                    var intensity = pixel.R * 0.299f + pixel.G * 0.587f + pixel.B * 0.114f;
                    heightmap[i, j] = (int)intensity;
                }
            }

            bitmap.Dispose();
            return heightmap;
        }
    }
}