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

            for (int i = 0; i < bodyA.ShapePoints.Length; i++)
            {
                for (int j = 0; j < bodyB.ShapePoints.Length; j++)
                {
                    double diameter = bodyA.ShapePoints[i].Diameter + bodyB.ShapePoints[j].Diameter + minDistance;
                    double distance = (bodyA.ShapePoints[i].Position - bodyB.ShapePoints[j].Position).Length();

                    if (distance < diameter)
                    {
                        bool intersection = (distance - minDistance < 0);

                        Vector3 normal = (bodyA.ShapePoints[i].Position - bodyB.ShapePoints[j].Position).Normalize();

                        CollisionPoint cp = new CollisionPoint(bodyA.ShapePoints[i].Position, bodyB.ShapePoints[j].Position, normal);
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

        public List<CollisionPointBaseStructure> SelfSoftBodyCollisionDetect(
            ISoftShape softBody,
            double minDistance)
        {
            List<CollisionPointBaseStructure> collisionPoint = new List<CollisionPointBaseStructure>();

            for (int i = 0; i < softBody.ShapePoints.Length; i++)
            {
                for (int j = i; j < softBody.ShapePoints.Length; j++)
                {
                    double diameter = softBody.ShapePoints[i].Diameter + softBody.ShapePoints[j].Diameter + minDistance;
                    double distance = (softBody.ShapePoints[i].Position - softBody.ShapePoints[j].Position).Length();

                    if (distance < diameter)
                    {
                        bool intersection = (distance - minDistance < 0);

                        Vector3 normal = (softBody.ShapePoints[i].Position - softBody.ShapePoints[j].Position).Normalize();

                        CollisionPoint cp = new CollisionPoint(softBody.ShapePoints[i].Position, softBody.ShapePoints[j].Position, normal);

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
