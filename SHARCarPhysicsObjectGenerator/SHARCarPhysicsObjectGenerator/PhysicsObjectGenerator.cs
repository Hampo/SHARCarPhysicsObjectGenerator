using NetP3DLib.Numerics;
using NetP3DLib.P3D.Chunks;
using System.Diagnostics;
using System.Numerics;

namespace SHARCarPhysicsObjectGenerator;

public static class PhysicsObjectGenerator
{
    private static readonly HashSet<string> JointNames =
    [
        "DoorDRot", "DoorPRot", "TrunkRot", "HoodRot", "w0", "w1", "w2", "w3"
    ];
    private static readonly HashSet<string> JointNamesBV =
    [
        "w0", "w1", "w2", "w3"
    ];

    public static PhysicsObjectChunk GeneratePhysicsObject(SkeletonChunk skeleton, CollisionObjectChunk collisionObject, bool BV = false)
    {
        var jointNames = BV ? JointNamesBV : JointNames;

        var collisionVolume = collisionObject.GetFirstChunkOfType<CollisionVolumeChunk>();
        var physicsObject = new PhysicsObjectChunk(collisionObject.Name, 1, "", skeleton.NumJoints, 0, 1);

        var joints = skeleton.GetChunksOfType<SkeletonJointChunk>();

        for (uint jointIndex = 0; jointIndex < joints.Length; jointIndex++)
        {
            var joint = joints[jointIndex];
            if (jointIndex == 0 || jointNames.Contains(joint.Name))
            {
                var physicsJoint = new PhysicsJointChunk(jointIndex, 0, 0, 0, 0, 0);

                // YAY HARDCODING
                switch (joint.Name)
                {
                    case "DoorDRot":
                    case "DoorPRot":
                    case "TrunkRot":
                        physicsJoint.Stiffness = 0.8f;
                        physicsJoint.MinAngle = 0.0f;
                        physicsJoint.MaxAngle = 1.0f;
                        physicsJoint.DOF = 1.0f;
                        break;
                    case "HoodRot":
                        physicsJoint.Stiffness = 0.5f;
                        physicsJoint.MinAngle = 0.0f;
                        physicsJoint.MaxAngle = 0.5f;
                        physicsJoint.DOF = 1.0f;
                        break;
                }

                var relevantVolumes = collisionVolume.GetChunksOfType<CollisionVolumeChunk>()
                    .Where(v => v.ObjectReferenceIndex == jointIndex)
                    .ToList();

                if (relevantVolumes.Count != 0)
                {
                    physicsJoint.Volume += relevantVolumes.Sum(CalculateVolumeMass);

                    var centerOfMass = CalculateCenterOfMass(relevantVolumes);
                    var physicsVector = new PhysicsVectorChunk(centerOfMass);
                    physicsJoint.Children.Add(physicsVector);

                    var inertiaMatrix = CalculateInertiaMatrix(relevantVolumes);
                    var physicsInertiaMatrixChunk = new PhysicsInertiaMatrixChunk(inertiaMatrix);
                    physicsJoint.Children.Add(physicsInertiaMatrixChunk);
                }
                else
                {
                    var physicsVector = new PhysicsVectorChunk(Vector3.Zero);
                    physicsJoint.Children.Add(physicsVector);

                    var physicsInertiaMatrixChunk = new PhysicsInertiaMatrixChunk(SymmetricMatrix3x3.Zero);
                    physicsJoint.Children.Add(physicsInertiaMatrixChunk);
                }

                physicsObject.Children.Add(physicsJoint);
                physicsObject.Volume += physicsJoint.Volume;
            }
        }

        return physicsObject;
    }

    private static Vector3 CalculateCenterOfMass(List<CollisionVolumeChunk> volumes)
    {
        var totalMass = 0f;
        var weightedSum = Vector3.Zero;

        foreach (var volume in volumes)
        {
            var vector = volume.Children[0].GetFirstChunkOfType<CollisionVectorChunk>();
            if (vector == null) continue;

            var mass = CalculateVolumeMass(volume);
            totalMass += mass;
            weightedSum += vector.Vector * mass;
        }

        return totalMass > 0 ? weightedSum / totalMass : Vector3.Zero;
    }

    private static float CalculateVolumeMass(CollisionVolumeChunk volume)
    {
        const float density = 1.0f;

        if (volume.Children.OfType<CollisionSphereChunk>().FirstOrDefault() is CollisionSphereChunk sphere)
        {
            return (4f / 3f) * MathF.PI * MathF.Pow(sphere.Radius, 3) * density;
        }

        if (volume.Children.OfType<CollisionOrientedBoundingBoxChunk>().FirstOrDefault() is CollisionOrientedBoundingBoxChunk obb)
        {
            return (8f * obb.HalfExtents.X * obb.HalfExtents.Y * obb.HalfExtents.Z) * density;
        }

        if (volume.Children.OfType<CollisionCylinderChunk>().FirstOrDefault() is CollisionCylinderChunk cylinder)
        {
            return (MathF.PI * MathF.Pow(cylinder.Radius, 2) * (cylinder.HalfLength * 2)) * density;
        }

        return 0f;
    }

    private static SymmetricMatrix3x3 CalculateInertiaMatrix(List<CollisionVolumeChunk> volumes)
    {
        var inertiaMatrix = new SymmetricMatrix3x3();

        foreach (var volume in volumes)
        {
            var vector = volume.Children[0].GetFirstChunkOfType<CollisionVectorChunk>();
            if (vector == null) continue;

            var mass = CalculateVolumeMass(volume);

            if (volume.GetFirstChunkOfType<CollisionOrientedBoundingBoxChunk>() is CollisionOrientedBoundingBoxChunk obb)
            {
                float ex = obb.HalfExtents.X * 2;
                float ey = obb.HalfExtents.Y * 2;
                float ez = obb.HalfExtents.Z * 2;
                var massFactor = (1f / 12f) * mass;

                float[,] inertia = new float[3, 3];
                inertia[0, 0] = massFactor * (float)(Math.Pow(ey, 2) + Math.Pow(ez, 2));
                inertia[1, 1] = massFactor * (float)(Math.Pow(ex, 2) + Math.Pow(ez, 2));
                inertia[2, 2] = massFactor * (float)(Math.Pow(ex, 2) + Math.Pow(ey, 2));

                var vectors = obb.GetChunksOfType<CollisionVectorChunk>();
                if (vectors.Length != 4)
                {
                    Debugger.Break();
                }
                var matrixX = vectors[1].Vector;
                var matrixY = vectors[2].Vector;
                var matrixZ = vectors[3].Vector;
                float[,] rot =
                {
                    { matrixX.X, matrixX.Y, matrixX.Z },
                    { matrixY.X, matrixY.Y, matrixY.Z },
                    { matrixZ.X, matrixZ.Y, matrixZ.Z },
                };

                var rotT = Transpose(rot);
                float[,] inertiaRotated = Multiply(Multiply(rotT, inertia), rot);
                inertiaMatrix += new SymmetricMatrix3x3(
                    inertiaRotated[0, 0], inertiaRotated[0, 1], inertiaRotated[0, 2],
                    inertiaRotated[1, 1], inertiaRotated[1, 2],
                    inertiaRotated[2, 2]
                );
            }
            else if (volume.GetFirstChunkOfType<CollisionSphereChunk>() is CollisionSphereChunk sphere)
            {
                var radius = sphere.Radius;
                var massFactor = (2f / 5f) * mass;

                float inertiaValue = massFactor * (float)Math.Pow(radius, 2);

                inertiaMatrix += new SymmetricMatrix3x3(inertiaValue, 0, 0, inertiaValue, 0, inertiaValue);
            }
            else if (volume.GetFirstChunkOfType<CollisionCylinderChunk>() is CollisionCylinderChunk cylinder)
            {
                float radius = cylinder.Radius;
                float halfLength = cylinder.HalfLength;
                float massFactor = mass;

                float inertiaX = (1f / 12f) * massFactor * (3 * radius * radius + halfLength * halfLength);
                float inertiaY = (1f / 2f) * massFactor * radius * radius;
                float inertiaZ = inertiaX;

                var vectors = cylinder.GetChunksOfType<CollisionVectorChunk>();
                if (vectors.Length == 2)
                {
                    var direction = vectors[1];

                    var matrixX = direction.Vector;
                    var matrixY = direction.Vector;
                    var matrixZ = direction.Vector;

                    float[,] rot =
                    {
                        { matrixX.X, matrixX.Y, matrixX.Z },
                        { matrixY.X, matrixY.Y, matrixY.Z },
                        { matrixZ.X, matrixZ.Y, matrixZ.Z },
                    };

                    var rotT = Transpose(rot);
                    float[,] inertiaMatrixLocal = new float[3, 3]
                    {
                        { inertiaX, 0, 0 },
                        { 0, inertiaY, 0 },
                        { 0, 0, inertiaZ }
                    };

                    float[,] inertiaRotated = Multiply(Multiply(rotT, inertiaMatrixLocal), rot);
                    inertiaMatrix += new SymmetricMatrix3x3(
                        inertiaRotated[0, 0], inertiaRotated[0, 1], inertiaRotated[0, 2],
                        inertiaRotated[1, 1], inertiaRotated[1, 2],
                        inertiaRotated[2, 2]
                    );
                }
                else
                {
                    inertiaMatrix += new SymmetricMatrix3x3(inertiaX, 0, 0, inertiaY, 0, inertiaZ);
                }
            }
            else
            {
                Debugger.Break();
            }
        }

        return inertiaMatrix;
    }

    private static float[,] Transpose(float[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);
        float[,] result = new float[cols, rows];

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result[j, i] = matrix[i, j];
            }
        }

        return result;
    }

    private static float[,] Multiply(float[,] a, float[,] b)
    {
        int aRows = a.GetLength(0);
        int aCols = a.GetLength(1);
        int bCols = b.GetLength(1);

        float[,] result = new float[aRows, bCols];

        for (int i = 0; i < aRows; i++)
        {
            for (int j = 0; j < bCols; j++)
            {
                result[i, j] = 0;
                for (int k = 0; k < aCols; k++)
                {
                    result[i, j] += a[i, k] * b[k, j];
                }
            }
        }

        return result;
    }
}
