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
        private readonly UnmanagedArray<eint> parentEntities;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedArray<Matrix4x4> anchoredLtwValues;
        private readonly UnmanagedArray<Quaternion> worldRotations;
        private readonly UnmanagedList<UnmanagedList<eint>> sortedEntities;

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
            foreach (UnmanagedList<eint> entities in sortedEntities)
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
            foreach (UnmanagedList<eint> entities in sortedEntities)
            {
                entities.Clear();
            }

            //calculate ltp of all entities first
            transformQuery.Update();
            foreach (var x in transformQuery)
            {
                uint index = (uint)x.entity;
                eint parent = world.GetParent(x.entity);
                parentEntities[index] = parent;
                ltwValues[index] = CalculateLocalToParent(x.entity, out Quaternion localRotation);
                worldRotations[index] = localRotation;

                //calculate how deep the entity is
                uint depth = 0;
                eint current = parent;
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
                ref UnmanagedList<eint> entities = ref sortedEntities[depth];
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
            foreach (UnmanagedList<eint> entities in sortedEntities)
            {
                foreach (eint entity in entities)
                {
                    uint index = (uint)entity;
                    eint parent = parentEntities[index];
                    ref Matrix4x4 ltw = ref ltwValues[index];
                    ref Quaternion worldRotation = ref worldRotations[index];
                    if (parent != default)
                    {
                        ref Matrix4x4 parentLtw = ref ltwValues[(uint)parent];
                        ref Quaternion parentWorldRotation = ref worldRotations[(uint)parent];
                        if (anchorsQuery.TryIndexOf(entity, out uint anchorIndex))
                        {
                            Anchor anchor = anchorsQuery[anchorIndex].Component2;
                            Vector3 parentSize = new LocalToWorld(parentLtw).Scale;
                            Vector3 parentPosition = new LocalToWorld(parentLtw).Position;
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

                            ltw *= Matrix4x4.CreateScale(anchorScale) * Matrix4x4.CreateTranslation(anchorPosition);
                        }
                        else
                        {
                            ltw *= parentLtw;
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
                ltw.value = ltwValues[(uint)r.entity];
            }

            //apply world rotations
            worldRotationQuery.Update();
            foreach (var r in worldRotationQuery)
            {
                ref WorldRotation worldRotation = ref r.Component2;
                worldRotation.value = worldRotations[(uint)r.entity];
            }
        }

        private Matrix4x4 CalculateLocalToParent(eint entity, out Quaternion localRotation)
        {
            ref Position position = ref world.GetComponentRef<Position>(entity, out bool hasPosition);
            ref EulerAngles eulerAngles = ref world.GetComponentRef<EulerAngles>(entity, out bool hasEulerAngles);
            ref Rotation rotation = ref world.GetComponentRef<Rotation>(entity, out bool hasRotation);
            ref Scale scale = ref world.GetComponentRef<Scale>(entity, out bool hasScale);
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
