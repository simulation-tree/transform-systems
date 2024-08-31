using Simulation;
using System.Numerics;
using Transforms.Components;
using Transforms.Events;
using Unmanaged.Collections;

namespace Transforms.Systems
{
    public class TransformSystem : SystemBase
    {
        private readonly Query<IsTransform> transformQuery;
        private readonly Query<IsTransform, Anchor> anchorsQuery;
        private readonly Query<IsTransform, LocalToWorld> ltwQuery;
        private readonly Query<IsTransform, WorldRotation> worldRotationQuery;
        private readonly UnmanagedArray<uint> parentEntities;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedArray<Matrix4x4> anchoredLtwValues;
        private readonly UnmanagedArray<Quaternion> worldRotations;
        private readonly UnmanagedList<UnmanagedList<uint>> sortedEntities;

        public TransformSystem(World world) : base(world)
        {
            Subscribe<TransformUpdate>(Update);
            transformQuery = new(world);
            anchorsQuery = new(world);
            ltwQuery = new(world);
            worldRotationQuery = new(world);
            parentEntities = new();
            ltwValues = new();
            anchoredLtwValues = new();
            worldRotations = new();
            sortedEntities = new();
        }

        public override void Dispose()
        {
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                entities.Dispose();
            }

            sortedEntities.Dispose();
            worldRotations.Dispose();
            ltwValues.Dispose();
            anchoredLtwValues.Dispose();
            parentEntities.Dispose();
            worldRotationQuery.Dispose();
            ltwQuery.Dispose();
            anchorsQuery.Dispose();
            transformQuery.Dispose();
            base.Dispose();
        }

        private void Update(TransformUpdate e)
        {
            //clear state
            parentEntities.Resize(world.MaxEntityValue + 1);
            parentEntities.Clear();
            ltwValues.Resize(world.MaxEntityValue + 1);
            ltwValues.Clear();
            worldRotations.Resize(world.MaxEntityValue + 1);
            worldRotations.Clear();
            anchoredLtwValues.Resize(world.MaxEntityValue + 1);
            anchoredLtwValues.Clear();

            //reset entities list
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            //calculate ltp of all entities first
            transformQuery.Update();
            foreach (var x in transformQuery)
            {
                uint entity = x.entity;
                uint parent = world.GetParent(x.entity);
                parentEntities[entity] = parent;
                ltwValues[entity] = CalculateLocalToParent(x.entity, out Quaternion localRotation);
                worldRotations[entity] = localRotation;

                //calculate how deep the entity is
                uint depth = 0;
                uint current = parent;
                while (current != default)
                {
                    depth++;
                    current = world.GetParent(current);
                }

                while (sortedEntities.Count <= depth)
                {
                    sortedEntities.Add(new());
                }

                //put the entity into a list located at the index, where the index is the depth
                ref UnmanagedList<uint> entities = ref sortedEntities[depth];
                entities.Add(x.entity);

                //make sure it has world component
                if (!world.ContainsComponent<LocalToWorld>(x.entity))
                {
                    world.AddComponent<LocalToWorld>(x.entity, default);
                }

                if (!world.ContainsComponent<WorldRotation>(x.entity))
                {
                    world.AddComponent<WorldRotation>(x.entity, default);
                }
            }

            //calculate ltw in descending order (roots towards leafs)
            anchorsQuery.Update();
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                foreach (uint entity in entities)
                {
                    uint parent = parentEntities[entity];
                    ref Matrix4x4 ltw = ref ltwValues[entity];
                    ref Quaternion worldRotation = ref worldRotations[entity];
                    if (parent != default)
                    {
                        LocalToWorld parentLtw = new(ltwValues[parent]);
                        Quaternion parentWorldRotation = worldRotations[parent];
                        if (anchorsQuery.TryIndexOf(entity, out uint anchorIndex))
                        {
                            Anchor anchor = anchorsQuery[anchorIndex].Component2;
                            (Vector3 parentPosition, Quaternion _, Vector3 parentSize) = parentLtw.Decomposed;
                            float minX = anchor.minX.Evaluate(parentSize.X);
                            float minY = anchor.minY.Evaluate(parentSize.Y);
                            float minZ = anchor.minZ.Evaluate(parentSize.Z);
                            float maxX = anchor.maxX.Evaluate(parentSize.X);
                            float maxY = anchor.maxY.Evaluate(parentSize.Y);
                            float maxZ = anchor.maxZ.Evaluate(parentSize.Z);
                            Vector3 anchorScale = new(maxX - minX, maxY - minY, maxZ - minZ);
                            Vector3 anchorPosition = parentPosition + new Vector3(minX, minY, minZ);
                            if (anchorScale.X == default)
                            {
                                anchorScale.X = 1f;
                            }

                            if (anchorScale.Y == default)
                            {
                                anchorScale.Y = 1f;
                            }

                            if (anchorScale.Z == default)
                            {
                                anchorScale.Z = 1f;
                            }

                            Matrix4x4 anchorLtw = Matrix4x4.CreateScale(anchorScale) * Matrix4x4.CreateTranslation(anchorPosition);
                            ltw *= anchorLtw;
                        }
                        else
                        {
                            ltw *= parentLtw.value;
                        }

                        worldRotation = parentWorldRotation * worldRotation;
                    }
                }

                entities.Clear();
            }

            //apply ltw values
            ltwQuery.Update();
            foreach (var r in ltwQuery)
            {
                ref LocalToWorld ltw = ref r.Component2;
                ltw.value = ltwValues[r.entity];
            }

            //apply world rotations
            worldRotationQuery.Update();
            foreach (var r in worldRotationQuery)
            {
                ref WorldRotation worldRotation = ref r.Component2;
                worldRotation.value = worldRotations[r.entity];
            }
        }

        private Matrix4x4 CalculateLocalToParent(uint entity, out Quaternion localRotation)
        {
            ref Position position = ref world.TryGetComponentRef<Position>(entity, out bool hasPosition);
            ref EulerAngles eulerAngles = ref world.TryGetComponentRef<EulerAngles>(entity, out bool hasEulerAngles);
            ref Rotation rotation = ref world.TryGetComponentRef<Rotation>(entity, out bool hasRotation);
            ref Scale scale = ref world.TryGetComponentRef<Scale>(entity, out bool hasScale);
            Matrix4x4 ltp = Matrix4x4.Identity;
            localRotation = Quaternion.Identity;
            if (hasScale)
            {
                ltp *= Matrix4x4.CreateScale(scale.value);
            }

            if (hasEulerAngles)
            {
                localRotation = eulerAngles.AsQuaternion() * localRotation;
            }

            if (hasRotation)
            {
                localRotation = rotation.value * localRotation;
            }

            ltp *= Matrix4x4.CreateFromQuaternion(localRotation);

            if (hasPosition)
            {
                ltp *= Matrix4x4.CreateTranslation(position.value);
            }

            return ltp;
        }
    }
}
