using ObjLoader.Loader.Loaders;
using SharpEngineMathUtility;
using SharpPhysicsEngine;
using SharpPhysicsEngine.LCPSolver;
using SharpPhysicsEngine.Wrapper;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utility;

namespace TestPhysics
{
    public class FrictionTest
    {
        #region Fields

        public List<string> ShapeFilename { get; set; }
        public List<string> TextureFilename { get; set; }
        public List<float> ShapeScale { get; set; }

        #endregion

        #region Constructor

        public FrictionTest()
        {
            ShapeFilename = new List<string>();
            ShapeScale = new List<float>();
            TextureFilename = new List<string>();
        }

        #endregion

        #region Public Methods

        public SharpEngine GetPhysicsEnvironment()
        {
            var physicsEnvironment = new SharpEngine();

            List<ICollisionShape> objects = GetSimulationObjects();

            foreach (var obj in objects)
                physicsEnvironment.AddShape(obj);

            //physicsEnvironment.RemoveShape(0);

            physicsEnvironment.SetSolverType(SolverType.ProjectedGaussSeidel);
            physicsEnvironment.SolverParameters.SetSolverMaxIteration(100);
            physicsEnvironment.SolverParameters.SetSOR(1.0);
            physicsEnvironment.SolverParameters.SetErrorTolerance(1E-8);

            return physicsEnvironment;
        }

        public int[][] LoadTexture()
        {
            int[][] textureID = new int[TextureFilename.Count][];

            for (int i = 0; i < TextureFilename.Count; i++)
            {
                textureID[i] = new int[1];

                for (int j = 0; j < 1; j++)
                    textureID[i][j] = OpenGLUtilities.LoadTexture(TextureFilename[i]);
            }
            return textureID;
        }

        public int[][] GetOpenGLEnvironment()
        {
            ObjImporter.meshStruct[][] loadObjects = new ObjImporter.meshStruct[ShapeFilename.Count][];

            for (int i = 0; i < ShapeFilename.Count; i++)
            {
                loadObjects[i] = new ObjImporter.meshStruct[1];

                for (int j = 0; j < 1; j++)
                {
                    loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i], new Vector3d(0.0, 0.0, 0.0), 0.0);
                    if(i==1)
                    {
                        loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i], new Vector3d(1.0, 0.0, 0.0), 0.75);
                    }
                    if (i > 1)
                        loadObjects[i][j] = LoadObjMesh(ShapeFilename[i], ShapeScale[i], new Vector3d(1.0, 0.0, 0.0), 0.75);
                }
            }

            return OpenGLUtilities.LoadGLObjects(
                loadObjects,
                ShapeFilename.Count,
                true,
                false,
                true);
        }


        #endregion

        #region Private Methods

        private List<ICollisionShape> GetSimulationObjects()
        {
            List<ICollisionShape> objects = new List<ICollisionShape>();

            #region Terrain Base

            ShapeFilename.Add("cube1.obj");
            ShapeScale.Add(60);
            TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom0 = GetObjectGeometry(ShapeFilename[0], ShapeScale[0], 0.0);
            var objects0 = new ConvexShape(geom0.VertexPoint, geom0.TriagleIdx, new Vector3d(0.0, -2.0, 0.0), 0.0, true);
            objects0.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 1.0));
            objects0.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects0.SetRestitutionCoeff(0.1);
            objects0.SetDynamicFrictionCoeff(1.0);
            objects0.SetStaticFrictionCoeff(1.0);
            objects0.ExcludeFromCollisionDetection(false);
            objects0.SetErrorReductionParam(0.3);

            objects.Add(objects0);

            #endregion

            #region Dynamic Objects

            Vector3d shift = new Vector3d(3.0, 0.0, 0.0);
            Vector3d position = new Vector3d(0.0, 7.0, 0.0);

            double[] mass = new double[] { 50, 20, 8, 3, 1 };

            
            ShapeFilename.Add("cube1.obj");
            ShapeScale.Add(10.0f);
            TextureFilename.Add("texture/woodbox.bmp");

            GeometryProperties geom1 = GetObjectGeometry("cube1.obj", 10.0f, 0.75);
            //var objects1 = new ConcaveShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0, true);
            var objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 0.0, true);
            //var objects1 = new ConvexShape(geom1.VertexPoint, position, 1.0);
            objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 1.0));
            objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
            objects1.SetRestitutionCoeff(0.01);
            objects1.SetDynamicFrictionCoeff(0.3);
            objects1.SetStaticFrictionCoeff(1.0);
            objects1.ExcludeFromCollisionDetection(false);
            objects1.SetErrorReductionParam(0.3);
            
            objects.Add(objects1);

            double[] friction = new double[] { 0.8, 0.6, 0.4, 0.2 };

            position = new Vector3d(-3.0, 13.50, -5.0);

            for (int i = 0; i < 4; i++)
            {
                ShapeFilename.Add("cube.obj");
                ShapeScale.Add(1);
                TextureFilename.Add("texture/woodbox.bmp");


                geom1 = GetObjectGeometry("cube.obj", 1, 0.75);
                objects1 = new ConvexShape(geom1.VertexPoint, geom1.TriagleIdx, position, 1.0);
                objects1.SetRotationStatus(new Quaternion(new Vector3d(0.0, 0.0, 0.0), 1.0));
                //objects1.SetGeometry(geom1.VertexPoint, geom1.TriagleIdx);
                objects1.SetLinearVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetAngularVelocity(new Vector3d(0.0, 0.0, 0.0));
                objects1.SetRestitutionCoeff(0.1);
                objects1.SetDynamicFrictionCoeff(friction[i]);
                objects1.SetStaticFrictionCoeff(0.9);
                objects1.ExcludeFromCollisionDetection(false);
                objects1.SetErrorReductionParam(0.5);
                position = position + shift;

                objects.Add(objects1);
            }

            
            

            #endregion

            return objects;
        }

        public class GeometryProperties
        {
            public Vector3d[] VertexPoint { get; private set; }
            public int[][] TriagleIdx { get; private set; }

            public GeometryProperties(
                Vector3d[] vertexPoint,
                int[][] triangleIndexes)
            {
                VertexPoint = vertexPoint;
                TriagleIdx = triangleIndexes;
            }
        }

        public static GeometryProperties GetObjectGeometry(
            string fileName,
            float scale,
            double rotate)
        {
            GenericUtility.ObjProperties properties = GenericUtility.GetImportedObjectProperties(fileName, scale);

            RotateObj(ref properties, new Vector3d(1.0, 0.0, 0.0), rotate);

            return new GeometryProperties(
                properties.vertexPoint,
                properties.triangleIndex);
        }

        private LoadResult LoadObjSolid(
            string fileName,
            float scale)
        {
            var objLoaderFactory = new ObjLoaderFactory();
            var objLoader = objLoaderFactory.Create();
            var fileStream = new FileStream(fileName, FileMode.OpenOrCreate);
            LoadResult solid = objLoader.Load(fileStream);
            fileStream.Close();

            OpenGLUtilities.UnitizeObject(ref solid);
            OpenGLUtilities.ScaleObject(ref solid, scale);

            return solid;
        }

        private ObjImporter.meshStruct LoadObjMesh(
            string fileName,
            double scale,
            Vector3d versor,
            double angle)
        {
            ObjImporter importer = new ObjImporter();
            ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

            Vector3d[] vertexStartPoint = new Vector3d[mesh.vertices.Length];

            for (int i = 0; i < mesh.vertices.Length; i++)
                vertexStartPoint[i] = mesh.vertices[i];

            OpenGLUtilities.UnitizeObject(ref vertexStartPoint);
            OpenGLUtilities.ScaleObject(ref vertexStartPoint, scale);

            RotateObj(ref vertexStartPoint, versor, angle);

            for (int i = 0; i < mesh.vertices.Length; i++)
                mesh.vertices[i] = vertexStartPoint[i];



            return mesh;
        }

        private SoftShape BuildSoftBody(
            string fileName,
            double scale,
            Vector3d position)
        {
            GenericUtility.ObjProperties prop = GenericUtility.GetImportedObjectProperties(fileName, scale);

            RotateObj(ref prop, new Vector3d(0.0, 0.0, 1.0), -Math.PI / 4.5);

            return new SoftShape(
                prop.triangleIndex,
                prop.vertexPoint,
                position,
                1.0,
                0.2,
                2.0,
                60.0);
        }


        private static void RotateObj(ref GenericUtility.ObjProperties obj, Vector3d versor, double angle)
        {
            for (int i = 0; i < obj.vertexPoint.Length; i++)
            {
                obj.vertexPoint[i] = Vector3d.RotatePoint(obj.vertexPoint[i], versor, angle);
            }
        }

        private static void RotateObj(ref Vector3d[] vertexStartPoint, Vector3d versor, double angle)
        {
            for (int i = 0; i < vertexStartPoint.Length; i++)
            {
                vertexStartPoint[i] = Vector3d.RotatePoint(vertexStartPoint[i], versor, angle);
            }
        }

        #endregion
    }
}
