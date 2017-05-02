using System.Collections.Generic;
using ShapeDefinition;
using PhysicsEngineMathUtility;

namespace CollisionEngine.SoftBody
{
    public class SoftBodyCollisionDetection
    {
        #region Constructor

        public SoftBodyCollisionDetection()
        { }

        #endregion

        #region Public Methods

        public List<CollisionPointBaseStructure> RigidVsSoftBodyCollisionDetection(
            IGeometry bodyA,
            ISoftShape bodyB,
            double minDistance)
        {

            List<CollisionPointBaseStructure> collisionPoint = new List<CollisionPointBaseStructure>();

            return collisionPoint;
        }

        public List<CollisionPointBaseStructure> SoftVsSoftBodyCollisionDetection(
            ISoftShape bodyA,
            ISoftShape bodyB,
            double minDistance)
        {
            List<CollisionPointBaseStructure> collisionPoint = new List<CollisionPointBaseStructure>();

            for (int i = 0; i < bodyA.ShapePoint.Length; i++)
            {
                for (int j = 0; j < bodyB.ShapePoint.Length; j++)
                {
                    double diameter = bodyA.ShapePoint[i].Diameter + bodyB.ShapePoint[j].Diameter + minDistance;
                    double distance = (bodyA.ShapePoint[i].Position - bodyB.ShapePoint[j].Position).Length();

                    if (distance < diameter)
                    {
                        bool intersection = (distance - minDistance < 0);

                        Vector3 normal = (bodyA.ShapePoint[i].Position - bodyB.ShapePoint[j].Position).Normalize();

                        CollisionPoint cp = new CollisionPoint(bodyA.ShapePoint[i].Position, bodyB.ShapePoint[j].Position, normal);
                        collisionPoint.Add(new CollisionPointBaseStructure(
                            distance,
                            intersection,
                            cp,
                            new CollisionPoint[] { cp }));
                    }
                }
            }

            return collisionPoint;
        }

        public List<CollisionPointBaseStructure> SelfSoftBodyCollisionDetection(
            ISoftShape softBody,
            double minDistance)
        {
            List<CollisionPointBaseStructure> collisionPoint = new List<CollisionPointBaseStructure>();

            for (int i = 0; i < softBody.ShapePoint.Length; i++)
            {
                for (int j = i; j < softBody.ShapePoint.Length; j++)
                {
                    double diameter = softBody.ShapePoint[i].Diameter + softBody.ShapePoint[j].Diameter + minDistance;
                    double distance = (softBody.ShapePoint[i].Position - softBody.ShapePoint[j].Position).Length();

                    if (distance < diameter)
                    {
                        bool intersection = (distance - minDistance < 0);

                        Vector3 normal = (softBody.ShapePoint[i].Position - softBody.ShapePoint[j].Position).Normalize();

                        CollisionPoint cp = new CollisionPoint(softBody.ShapePoint[i].Position, softBody.ShapePoint[j].Position, normal);

                        collisionPoint.Add(new CollisionPointBaseStructure(
                            distance,
                            intersection,
                            cp,
                            new CollisionPoint[] { cp }));
                    }
                }
            }

            return collisionPoint;
        }

        #endregion

        #region Private Methods



        #endregion
    }
}
