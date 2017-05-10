using PhysicsEngineMathUtility;
using System.Globalization;

namespace Utility
{
    public static class GenericUtility
    {
        public struct ObjProperties
        {
            public Vector3[] vertexPoint;
            public int[][] triangleIndex;
        }

        public static double InvariantCultureDoubleConverter(string value)
        {
            string text = value.Replace(',', '.');
            double tmp;
            double.TryParse(text, NumberStyles.Any, CultureInfo.InvariantCulture, out tmp);
            return tmp;
        }

        public static ObjProperties GetImportedObjectProperties(
            string fileName,
            double scale)
        {
            ObjImporter importer = new ObjImporter();
            ObjImporter.meshStruct mesh = importer.ImportFile(fileName);

            Vector3[] vertexStartPoint = new Vector3[mesh.vertices.Length];

            for (int i = 0; i < mesh.vertices.Length; i++)
                vertexStartPoint[i] = mesh.vertices[i];

            OpenGLUtilities.UnitizeObject(ref vertexStartPoint);
            OpenGLUtilities.ScaleObject(ref vertexStartPoint, scale);

            int nTriangle = mesh.faceData.Length / 3;
            int[][] triangleIndex = new int[nTriangle][];

            for (int i = 0; i < nTriangle; i++)
            {
                triangleIndex[i] = new int[3];
                triangleIndex[i][0] = (int)mesh.faceData[i * 3].x - 1;
                triangleIndex[i][1] = (int)mesh.faceData[(i * 3) + 1].x - 1;
                triangleIndex[i][2] = (int)mesh.faceData[(i * 3) + 2].x - 1;
            }

            return new ObjProperties { vertexPoint = vertexStartPoint, triangleIndex = triangleIndex };
        }


    }
}
