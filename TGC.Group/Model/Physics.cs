using BulletSharp;
using Microsoft.DirectX.Direct3D;
using Microsoft.DirectX.DirectInput;
using TGC.Core.BulletPhysics;
using TGC.Core.Direct3D;
using TGC.Core.Input;
using TGC.Core.Mathematica;
using TGC.Core.Textures;
using TGC.Core.SceneLoader;
using System.Drawing;
using TGC.Core.Shaders;
using TGC.Group.Model;
using TGC.Core.Geometry;

namespace TGC.Examples.Bullet.Physics
{
    internal class Physics
    {
        //Configuracion de la Simulacion Fisica
        private DiscreteDynamicsWorld dynamicsWorld;

        private CollisionDispatcher dispatcher;
        private DefaultCollisionConfiguration collisionConfiguration;
        private SequentialImpulseConstraintSolver constraintSolver;
        private BroadphaseInterface overlappingPairCache;

        //Datos del los triangulos del VertexBuffer
        private CustomVertex.PositionTextured[] triangleDataVB;

        public RigidBody carChassis;
        public RigidBody[] rgTronco = new RigidBody[40];
        public int cant_troncos;
        public int cant_troncos_ok;
        public RigidBody[] rgObstaculos = new RigidBody[20];
        public TGCMatrix[] ob_tr = new TGCMatrix[100];

        public int cant_obstaculos;
        public CollisionShape[] cs_camion = new CollisionShape[100];
        public TGCMatrix[] cs_tr = new TGCMatrix[100];
        public int cant_cs_camion = 0;


        private TgcScene currentScene;
        private TgcMesh camion;
        private TGCMatrix camion_tr;
        public TgcMesh tronco;
        private TGCMatrix tronco_tr;


        public TGCBox bb = new TGCBox();
        public TgcMesh rueda;
        public TgcMesh boxMesh;

        const int rightIndex = 0;
        const int upIndex = 1;
        const int forwardIndex = 2;
        BulletSharp.Math.Vector3 wheelDirectionCS0 = new BulletSharp.Math.Vector3(0, -1, 0);
        BulletSharp.Math.Vector3 wheelAxleCS = new BulletSharp.Math.Vector3(-1, 0, 0);

        // btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
        // notice that for higher-quality slow-moving vehicles, another approach might be better
        // implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts

        float gEngineForce = 0.0f;
        float gBreakingForce = 0.0f;

        const float maxEngineForce = 2000.0f;
        const float maxBreakingForce = 400.0f;

        float gVehicleSteering = 0;
        const float steeringIncrement = 5.0f;
        const float steeringClamp = 0.7f;
        public float wheelRadius = 0.15f;         // 0,15
        public float wheelWidth = 0.1f;
        public float connectionHeight = 0.0f;       // 0.18f;            // altura de la rueda
        const float wheelFriction = 1000;
        const float suspensionStiffness = 20.0f * 2.0f;
        const float suspensionDamping = 2.3f * 3.0f;
        const float suspensionCompression = 4.4f *3.0f;
        const float rollInfluence = 0.1f;//1.0f;
        const float suspensionRestLength = 0.15f;
        public TGCVector3 camion_size = new TGCVector3();

        public RaycastVehicle vehicle;

        public float[] wi_rotation = new float[32];         // corrije el bug de bullet en la rotacion de la rueda
        public float[] wi_delta_rotation = new float[32];
        public TGCVector3[] wi_position = new TGCVector3[32];
        public TGCVector3[] wi_contact_pt = new TGCVector3[32];
        public bool[] wi_is_in_contact = new bool[32];
        public bool flag_wi = false;

        // pos inicial del camion
        public TGCVector3 pos_camion;
        public TGCVector3 pos_final;

        public GameModel game;
        public bool flag_llegada = false;


        public Physics(GameModel p_game)
        {
            game = p_game;
        }


        RigidBody LocalCreateRigidBody(float mass, BulletSharp.Math.Matrix startTransform, CollisionShape shape)
        {
            bool isDynamic = (mass != 0.0f);

            BulletSharp.Math.Vector3 localInertia = BulletSharp.Math.Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

            RigidBody body;
            using (var rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia))
            {
                body = new RigidBody(rbInfo);
            }

            dynamicsWorld.AddRigidBody(body);

            return body;
        }


        public RigidBody CreateCylinder(TGCVector3 dimensions, TGCVector3 position, float mass)
        {
            //Creamos el Shape de un Cilindro
            var cylinderShape = new CylinderShape(dimensions.X, dimensions.Y, dimensions.Z);

            //Armamos la matrix asociada al Cilindro y el estado de movimiento de la misma.
            var cylinderTransform = TGCMatrix.RotationX(-3.14f / 2.0f);

            cylinderTransform.Origin = position;
            var cylinderMotionState = new DefaultMotionState(cylinderTransform.ToBulletMatrix());

            //Calculamos el momento de inercia
            var cylinderLocalInertia = cylinderShape.CalculateLocalInertia(mass);
            var cylinderInfo = new RigidBodyConstructionInfo(mass, cylinderMotionState, cylinderShape, cylinderLocalInertia);

            //Creamos el cuerpo rigido a partir del de la informacion de cuerpo rigido.
            RigidBody cylinderBody = new RigidBody(cylinderInfo);
            return cylinderBody;
        }


        public void SetTriangleDataVB(CustomVertex.PositionTextured[] newTriangleData)
        {
            triangleDataVB = newTriangleData;
        }

        public void Init(string MediaDir)
        {
            //Creamos el mundo fisico por defecto.
            collisionConfiguration = new DefaultCollisionConfiguration();
            dispatcher = new CollisionDispatcher(collisionConfiguration);
            GImpactCollisionAlgorithm.RegisterAlgorithm(dispatcher);
            constraintSolver = new SequentialImpulseConstraintSolver();
            overlappingPairCache = new DbvtBroadphase(); //AxisSweep3(new BsVector3(-5000f, -5000f, -5000f), new BsVector3(5000f, 5000f, 5000f), 8192);
            dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, overlappingPairCache, constraintSolver, collisionConfiguration);
            dynamicsWorld.Gravity = new TGCVector3(0, -10f, 0).ToBulletVector3();

            //Creamos el terreno
            var meshRigidBody = BulletRigidBodyFactory.Instance.CreateSurfaceFromHeighMap(triangleDataVB);
            meshRigidBody.UserIndex = 0;
            meshRigidBody.Friction = 0.33f;
            dynamicsWorld.AddRigidBody(meshRigidBody);

            //Cargar escena con herramienta TgcSceneLoader
            var loader = new TgcSceneLoader();
            currentScene = loader.loadSceneFromFile(MediaDir + "CamionTroncos\\CamionDeTroncos-TgcScene.xml");
            camion = currentScene.Meshes[0];
            camion.Effect = TGCShaders.Instance.TgcMeshPhongShader;
            var escale = 0.01f;
            camion_size = camion.BoundingBox.PMax - camion.BoundingBox.PMin;
            camion_size = camion_size * escale;
            camion_tr = TGCMatrix.Scaling(escale, escale, -escale) * TGCMatrix.Translation(new TGCVector3(0, -suspensionRestLength-0.06f, 0));

            // create vehicle. Ojo que BoxShape toma halfSize! 
            connectionHeight = 0f;
            float dH = 0.12f;
            float largo_cabina = camion_size.Z;     //  1.3f;
            float bdx = camion_size.X;
            float bdy = 0.34f;
            CollisionShape chassisShape = new BoxShape(bdx * 0.5f, bdy * 0.5f , largo_cabina *0.5f);
            CompoundShape compound = new CompoundShape();
            // Hay que levantar el chassis del suelo, eso esta en proporcion a la altura de las ruedas
            BulletSharp.Math.Matrix localTrans = BulletSharp.Math.Matrix.Translation(0, dH, (camion_size.Z- largo_cabina)*0.5f);
            compound.AddChildShape(localTrans, chassisShape);
            cs_camion[cant_cs_camion] = chassisShape;
            cs_tr[cant_cs_camion] = TGCMatrix.Scaling(new TGCVector3(bdx, bdy, largo_cabina)) * new TGCMatrix(localTrans);
            ++cant_cs_camion;

            // barras laterales
            float col_dx = 0.1f;
            float col_dy = 0.3f;
            float col_dz = 2.7f;
            dH = 0.3f;
            float pos_izq = -camion_size.X * 0.5f + col_dx * 0.5f;
            float pos_der = camion_size.X * 0.5f - col_dx * 0.5f;
            localTrans = BulletSharp.Math.Matrix.Translation(pos_izq, dH + col_dy * 0.5f, -0.5f);
            CollisionShape colShape = new BoxShape(col_dx * 0.5f, col_dy * 0.5f, col_dz * 0.5f);
            compound.AddChildShape(localTrans, colShape);
            cs_camion[cant_cs_camion] = colShape;
            cs_tr[cant_cs_camion] = TGCMatrix.Scaling(new TGCVector3(col_dx, col_dy, col_dz)) * new TGCMatrix(localTrans);
            ++cant_cs_camion;
            localTrans = BulletSharp.Math.Matrix.Translation(pos_der, dH + col_dy * 0.5f, -0.5f);
            colShape = new BoxShape(col_dx * 0.5f, col_dy * 0.5f, col_dz * 0.5f);
            compound.AddChildShape(localTrans, colShape);
            cs_camion[cant_cs_camion] = colShape;
            cs_tr[cant_cs_camion] = TGCMatrix.Scaling(new TGCVector3(col_dx, col_dy, col_dz)) * new TGCMatrix(localTrans);
            ++cant_cs_camion;
            col_dz = 0.1f;

            // trasera
            localTrans = BulletSharp.Math.Matrix.Translation(0, dH + col_dy * 0.5f, -camion_size.Z*0.5f+0.1f);
            CollisionShape tapaShape = new BoxShape(camion_size.X * 0.5f, col_dy * 0.5f, col_dz * 0.5f);
            compound.AddChildShape(localTrans, tapaShape);
            cs_camion[cant_cs_camion] = tapaShape;
            cs_tr[cant_cs_camion] = TGCMatrix.Scaling(new TGCVector3(camion_size.X, col_dy, col_dz)) * new TGCMatrix(localTrans);
            ++cant_cs_camion;

            // cabina y tambien actua de tapa delantera
            col_dy = 0.3f;
            col_dz = 0.7f;
            dH = 0.3f;
            localTrans = BulletSharp.Math.Matrix.Translation(0, dH + col_dy * 0.5f, 1.2f);
            tapaShape = new BoxShape(camion_size.X * 0.5f, col_dy * 0.5f, col_dz * 0.5f);
            compound.AddChildShape(localTrans, tapaShape);
            cs_camion[cant_cs_camion] = tapaShape;
            cs_tr[cant_cs_camion] = TGCMatrix.Scaling(new TGCVector3(camion_size.X, col_dy, col_dz)) * new TGCMatrix(localTrans);
            ++cant_cs_camion;

            // creo el rb
            carChassis = LocalCreateRigidBody(800, BulletSharp.Math.Matrix.Identity, compound);

            VehicleTuning tuning = new VehicleTuning();
            DefaultVehicleRaycaster vehicleRayCaster = new DefaultVehicleRaycaster(dynamicsWorld);
            vehicle = new RaycastVehicle(tuning, carChassis, vehicleRayCaster);

            carChassis.ActivationState = ActivationState.DisableDeactivation;
            dynamicsWorld.AddAction(vehicle);


            // choose coordinate system
            vehicle.SetCoordinateSystem(rightIndex, upIndex, forwardIndex);
            BulletSharp.Math.Vector3 connectionPointCS0;
            float[] pos_z = { 2.04f, 0.5f, -0.12f, -0.53f, -1.14f, -1.56f};
            float[] desf_xi = { 16, 11, 5, 5, 5, 5 };
            float[] desf_xd = { 16, 13, 10, 10, 10, 10 };
            for (int i = 0; i < 6; ++i)
            {
                bool isFrontWheel = i<2;
                connectionPointCS0 = new BulletSharp.Math.Vector3(camion_size.X * 0.5f - (0.3f * wheelWidth)- desf_xi[i]/100.0f, connectionHeight, pos_z[i]);
                vehicle.AddWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);

                connectionPointCS0 = new BulletSharp.Math.Vector3(-camion_size.X * 0.5f + (0.3f * wheelWidth) + desf_xd[i] / 100.0f, connectionHeight, pos_z[i]);
                vehicle.AddWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, tuning, isFrontWheel);
            }


            for (int i = 0; i < vehicle.NumWheels; i++)
            {
                WheelInfo wheel = vehicle.GetWheelInfo(i);
                wheel.SuspensionStiffness = suspensionStiffness;
                wheel.WheelsDampingRelaxation = suspensionDamping;
                wheel.WheelsDampingCompression = suspensionCompression;
                wheel.FrictionSlip = wheelFriction;
                wheel.RollInfluence = rollInfluence;
                wi_rotation[i] = 0;
                wi_delta_rotation[i] = 0;
                wi_position[i] = new TGCVector3(0, 0, 0);
            }

            bb.setPositionSize(new TGCVector3(0, 0, 0), new TGCVector3(1,1,1));
            bb.updateValues();


            tronco = loader.loadSceneFromFile(MediaDir + "SketchUp\\tronco2-TgcScene.xml").Meshes[0];
            tronco.Effect = TGCShaders.Instance.TgcMeshPhongShader;
            tronco.Technique = "DIFFUSE_MAP";
            tronco.Effect.SetValue("lightPosition", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(1000, 1000, 1000)));
            tronco.Effect.SetValue("ambientColor", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(0.3f, 0.3f, 0.3f)));
            tronco.Effect.SetValue("diffuseColor", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(1f, 1f, 1f)));

            rueda = loader.loadSceneFromFile(MediaDir + "SketchUp\\rueda-TgcScene.xml").Meshes[0];
            rueda.Effect = TGCShaders.Instance.TgcMeshPhongShader;


            // posicion inicial del camion
            var x = pos_camion.X;
            var y = pos_camion.Y;
            var z = pos_camion.Z;
            vehicle.RigidBody.WorldTransform = BulletSharp.Math.Matrix.Translation(x, y, z);


            float r = 0.08f;
            float d = 2 * r;
            TGCVector3 tronco_size = new TGCVector3(r, 1.1f, r);
            tronco_tr = TGCMatrix.RotationX(FastMath.PI_HALF) * TGCMatrix.Scaling(2.3f*r/21.0f , 2.3f * 1.1f / 100.0f , 2.3f * r /19.0f);
            cant_troncos = 0;
            float tronco_mass = 10.0f;
            for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
            {
                rgTronco[cant_troncos] = CreateCylinder(tronco_size, 
                        new TGCVector3(x-d-r+j*d+ (i % 2 == 0 ? 1 : 0) * r, y +bdy*0.5f+0.02f+i*d, z-0.3f), tronco_mass);
                rgTronco[cant_troncos].Friction = 110.1f;
                rgTronco[cant_troncos].AnisotropicFriction = new BulletSharp.Math.Vector3(0.01f, 0.5f, 1.3f);
                dynamicsWorld.AddRigidBody(rgTronco[cant_troncos]);
                rgTronco[cant_troncos].ActivationState = ActivationState.DisableDeactivation;
                ++cant_troncos;
            }
            cant_troncos_ok = cant_troncos;
            cant_obstaculos = 0;


            x = pos_final.X;
            y = pos_final.Y;
            z = pos_final.Z;

            for (int j = 0; j< 3; ++j)
            {
                rgObstaculos[cant_obstaculos] = BulletRigidBodyFactory.Instance.CreateBox(new TGCVector3(4-j*0.5f, 0.1f, 4-j * 0.5f), 0,
                        new TGCVector3(x, y+0.1f*j, z), 0, 0, 0, 0, false);
                rgObstaculos[cant_obstaculos].ActivationState = ActivationState.DisableDeactivation;
                ob_tr[cant_obstaculos] = TGCMatrix.Scaling(8-j, 0.2f, 8-j);
                rgObstaculos[cant_obstaculos].UserIndex = 1;
                dynamicsWorld.ContactTest(rgObstaculos[cant_obstaculos], new BulletContactCallback(dynamicsWorld));
                dynamicsWorld.AddRigidBody(rgObstaculos[cant_obstaculos++]);
            }


            /*
            bdy = wheelRadius + 0.3f;
            rgObstaculos[cant_obstaculos] = BulletRigidBodyFactory.Instance.CreateBox(new TGCVector3(bdx*0.5f, bdy*0.5f, 0.5f), 0,
                    new TGCVector3(0, bdy*0.5f,5), 0, 0, 0, 0, false);
            rgObstaculos[cant_obstaculos].ActivationState = ActivationState.DisableDeactivation;
            ob_tr[cant_obstaculos] = TGCMatrix.Scaling(bdx, bdy, 1);
            dynamicsWorld.AddRigidBody(rgObstaculos[cant_obstaculos++]);
            rgObstaculos[cant_obstaculos] = BulletRigidBodyFactory.Instance.CreateBox(new TGCVector3(0.2f, 0.1f, 0.2f), 100,
                    new TGCVector3(0, 3, 0), 0, 0, 0, 0, false);
            rgObstaculos[cant_obstaculos].ActivationState = ActivationState.DisableDeactivation;
            rgObstaculos[cant_obstaculos].Friction = 2.0f;
            ob_tr[cant_obstaculos] = TGCMatrix.Scaling(0.4f, 0.2f, 0.4f);
            dynamicsWorld.AddRigidBody(rgObstaculos[cant_obstaculos++]);
            */

            bb.Color = Color.Yellow;
            bb.updateValues();

            TGCBox box_aux = TGCBox.fromSize(new TGCVector3(1,1,1), TgcTexture.createTexture(D3DDevice.Instance.Device, MediaDir + "metal.jpg"));
            box_aux.UVTiling = new TGCVector2(3, 3);
            box_aux.updateValues();
            boxMesh = box_aux.ToMesh("llegada");
            boxMesh.Effect = TGCShaders.Instance.TgcMeshPhongShader;

        }

        class BulletContactCallback : ContactResultCallback
        {
            private DynamicsWorld world;
            public BulletContactCallback(DynamicsWorld p_world)
            {
                world = p_world;
            }

            public override float AddSingleResult(ManifoldPoint cp, CollisionObjectWrapper colObj0Wrap, int partId0, int index0, CollisionObjectWrapper colObj1Wrap, int partId1, int index1)
            {
                if (colObj0Wrap.CollisionObject.UserIndex == 1)
                {
                    if (colObj1Wrap.CollisionObject.UserIndex != 0)
                    {
                        var bp = 1;
                    }
                }
                else
                {
                    if (colObj0Wrap.CollisionObject.UserIndex != 0)
                    {
                        var bp = 1;
                    }

                }
                return 0;
            }
        };


        public void Update(TgcD3dInput input)
        {
            float FrameDelta = 1 / 60f;
            dynamicsWorld.StepSimulation(FrameDelta, 100);

             if (input.keyDown(Key.Right))
             {
                 gVehicleSteering -= FrameDelta * steeringIncrement;
                 if (gVehicleSteering < -steeringClamp)
                     gVehicleSteering = -steeringClamp;
             }
             else if ((gVehicleSteering + float.Epsilon) < 0)
             {
                 gVehicleSteering += FrameDelta * steeringIncrement;
             }

             if (input.keyDown(Key.Left))
             {
                 gVehicleSteering += FrameDelta * steeringIncrement;
                 if (gVehicleSteering > steeringClamp)
                     gVehicleSteering = steeringClamp;
             }
             else if ((gVehicleSteering - float.Epsilon) > 0)
             {
                 gVehicleSteering -= FrameDelta * steeringIncrement;
             }

             if (input.keyDown(Key.Up))
             {
                 gEngineForce = maxEngineForce;
             }

             if (input.keyDown(Key.Down))
             {
                 gEngineForce = -maxEngineForce;
             }

             if (input.keyDown(Key.Space))
             {
                 gBreakingForce = maxBreakingForce;
                 gEngineForce = 0;
             }

             if (input.keyUp(Key.Space))
             {
                 gBreakingForce = 0;
             }


            gEngineForce *= 0.5f;
            // traccion delantera
            vehicle.ApplyEngineForce(gEngineForce, 0);
            vehicle.ApplyEngineForce(gEngineForce, 1);
            // traccion 4 x 4
            vehicle.ApplyEngineForce(gEngineForce, 2);
            vehicle.ApplyEngineForce(gEngineForce, 3);

            // el freno solo se aplica las ruedas de adelante. Si no se hace totalmente INESTABLE
            vehicle.SetBrake(gBreakingForce, 0);
            vehicle.SetBrake(gBreakingForce, 1);

            vehicle.SetSteeringValue(gVehicleSteering, 0);
            vehicle.SetSteeringValue(gVehicleSteering, 1);


            TGCVector3 Z_ws = new TGCVector3(0, 0, 1);
            Z_ws.TransformNormal(new TGCMatrix(vehicle.ChassisWorldTransform));

            for (int i = 0; i < vehicle.NumWheels; i++)
            {
                TGCVector3 pos = new TGCMatrix(vehicle.GetWheelTransformWS(i)).Origin;
                if (flag_wi)
                {
                    WheelInfo wi = vehicle.GetWheelInfo(i);
                    TGCVector3 ds = pos - wi_position[i];
                    if (wi.RaycastInfo.IsInContact)
                    {
                        float k = TGCVector3.Dot(Z_ws, ds) > 0 ? 1 : -1;
                        wi_delta_rotation[i] = ds.Length() / wheelRadius * k;
                    }
                    wi_delta_rotation[i] *= 0.99f;
                    wi_rotation[i] += wi_delta_rotation[i];
                    wi_is_in_contact[i] = wi.RaycastInfo.IsInContact;
                    wi_contact_pt[i] = new TGCVector3(wi.RaycastInfo.ContactPointWS);
                }
                wi_position[i] = pos;

            }

            cant_troncos_ok = 0;
            TGCVector3 p0 = new TGCVector3(vehicle.ChassisWorldTransform.Origin);
            for (int i = 0; i < cant_troncos; ++i)
            {
                TGCVector3 p = new TGCVector3(rgTronco[i].InterpolationWorldTransform.Origin) - p0;
                if (p.LengthSq() < 3)
                    cant_troncos_ok++;
            }


            flag_wi = true;

    }

    public void Render()
        {

            camion.Effect.SetValue("specularColor", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(0.5f, 0.5f, 0.5f)));
            camion.Effect.SetValue("specularExp", 8f);
            camion.Transform = camion_tr * new TGCMatrix(vehicle.ChassisWorldTransform);
            camion.AlphaBlendEnable = true;
            camion.Render();

            // ruedas
            rueda.Effect.SetValue("specularColor", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(0.5f, 0.5f, 0.5f)));
            TGCMatrix vehicle_tr = new TGCMatrix(vehicle.ChassisWorldTransform);
            vehicle_tr.M41 = vehicle_tr.M42 = vehicle_tr.M43 = 0;
            for (var i = 0; i < vehicle.NumWheels; ++i)
            {
                vehicle.UpdateWheelTransform(i, true);

                TGCVector3 pos = new TGCMatrix(vehicle.GetWheelTransformWS(i)).Origin;
                vehicle_tr.M41 = pos.X;
                vehicle_tr.M42 = pos.Y;
                vehicle_tr.M43 = pos.Z;

                WheelInfo wi = vehicle.GetWheelInfo(i);
                float st = wi.Steering;
                float rot = wi_rotation[i]; ///  wi.Rotation;
                rueda.Transform = TGCMatrix.RotationY(FastMath.PI_HALF) *
                    TGCMatrix.Scaling(2 * wheelWidth / 200, 2 * wheelRadius / 200, 2 * wheelRadius / 200) 
                    * TGCMatrix.RotationX(rot)
                    * TGCMatrix.RotationY(-st) * vehicle_tr;
                rueda.Render();
            }

            bool debug = false;

            Color[] clr = new Color[7];
            clr[0] = Color.Blue;
            clr[1] = Color.Yellow;
            clr[2] = Color.Red;
            clr[3] = Color.BlueViolet;
            clr[4] = Color.DarkGreen;
            clr[5] = Color.GhostWhite;
            clr[6] = Color.LightYellow;

            if (debug)
            {


                for (int i = 0; i < cant_cs_camion; ++i)
                {
                    bb.Color = clr[ (i+4) % 7];
                    bb.updateValues();
                    bb.Transform = cs_tr[i] * new TGCMatrix(vehicle.ChassisWorldTransform);
                    bb.Render();
                }
            }

            tronco.Effect.SetValue("specularColor", TGCVector3.TGCVector3ToFloat4Array(new TGCVector3(0.2f, 0.2f, 0.2f)));
            tronco.Effect.SetValue("specularExp", 4f);
            for (int i = 0; i < cant_troncos; ++i)
            {
                tronco.Transform = tronco_tr * new TGCMatrix(rgTronco[i].InterpolationWorldTransform);
                tronco.Render();
            }

            bb.AlphaBlendEnable = true;
            float ep = 0.5f;
            BulletSharp.Math.Vector3 min, max;
            flag_llegada = false;
            for (int i=0;i<cant_obstaculos;++i)
            {
                rgObstaculos[i].GetAabb(out min, out max);
                int cant = 0;
                for (var j = 0; j < vehicle.NumWheels; ++j)
                    if (wi_is_in_contact[j])
                    {
                        float x = wi_contact_pt[j].X;
                        float z = wi_contact_pt[j].Z;
                        if (x > min.X - ep && x < max.X - ep && z > min.Z - ep && z < max.Z - ep)
                            ++cant;
                    }

                /*if (cant > 0)
                {
                    int alfa = (int)FastMath.Clamp(128 + 128 * (float)cant / (float)vehicle.NumWheels , 0,255);
                    bb.Color = Color.FromArgb(alfa, 80, 80, 255);
                }
                */

                if(cant== vehicle.NumWheels)
                {
                    flag_llegada = true;
                }
                boxMesh.Transform =  ob_tr[i] * new TGCMatrix(rgObstaculos[i].WorldTransform);
                boxMesh.Render();

            }



        }

        public void Dispose()
        {
            currentScene.DisposeAll();
            //Se hace dispose del modelo fisico.
            dynamicsWorld.Dispose();
            dispatcher.Dispose();
            collisionConfiguration.Dispose();
            constraintSolver.Dispose();
            overlappingPairCache.Dispose();
        }
    }
}