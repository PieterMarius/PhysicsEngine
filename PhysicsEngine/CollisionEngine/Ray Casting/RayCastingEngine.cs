using SharpEngineMathUtility;
using SharpPhysicsEngine.ShapeDefinition;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class RayCastingEngine
    {
        #region Constructor

        #endregion

        #region Public Methods

        public Vector3d? Execute(
            IShape shape,
            Vector3d point, 
            Vector3d direction)
        {
            var vertices = Helper.GetVertexPosition(shape);

            
            
            //double radius = 1E-4;
            //double t = 0.0;
            ////Vector3d collisionPoint = GetAABBDist(shapeA, shapeB);

            //double distance = collisionPoint.Length();

            //while (distance > radius)
            //{
            //    double nLinear = direction.Dot(collisionPoint.Normalize());
                
            //    t += distance / nLinear;

            //    if (t < 0.0)
            //    {
            //        // never hit during this timestep
            //        return null;
            //    }



            //    //integratePosition.IntegrateObjectPosition(shapeA, t);
            //    //integratePosition.IntegrateObjectPosition(shapeB, t);

            //    //collisionPoint = GetAABBDist(shapeA, shapeB);

            //    //distance = collisionPoint.Length();
            //}

            return new Vector3d();
        }

        #endregion
    }
}
