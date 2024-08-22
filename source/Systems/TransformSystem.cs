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
        private readonly UnmanagedArray<eint> parentEntities;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedArray<Matrix4x4> anchoredLtwValues;
        private readonly UnmanagedList<UnmanagedList<eint>> sortedEntities;

        public TransformSystem(World world) : base(world)
        {
            Subscribe<TransformUpdate>(Update);
            transformQuery = new(world);
            anchorsQuery = new(world);
            ltwQuery = new(world);
            parentEntities = new();
            ltwValues = new();
            anchoredLtwValues = new();
            sortedEntities = new();
        }

        public override void Dispose()
        {
            foreach (UnmanagedList<eint> entities in sortedEntities)
            {
                entities.Dispose();
            }

            sortedEntities.Dispose();
            ltwValues.Dispose();
            anchoredLtwValues.Dispose();
            parentEntities.Dispose();
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
                ltwValues[index] = CalculateLocalToParent(x.entity);

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
                ref UnmanagedList<eint> entities = ref sortedEntities.GetRef(depth);
                entities.Add(x.entity);

                //make sure it has an ltw component
                if (!world.ContainsComponent<LocalToWorld>(x.entity))
                {
                    world.AddComponent<LocalToWorld>(x.entity, default);
                }
            }

            //calculate ltw from ltp in descending order (roots to leafs)
            anchorsQuery.Update();
            foreach (UnmanagedList<eint> entities in sortedEntities)
            {
                foreach (eint entity in entities)
                {
                    uint index = (uint)entity;
                    eint parent = parentEntities[index];
                    ref Matrix4x4 ltw = ref ltwValues.GetRef(index);
                    if (parent != default)
                    {
                        ref Matrix4x4 parentLtw = ref ltwValues.GetRef((uint)parent);
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
                    }
                }

                entities.Clear();
            }

            //apply ltw values
            ltwQuery.Update();
            foreach (Query<IsTransform, LocalToWorld>.Result result in ltwQuery)
            {
                ref LocalToWorld ltw = ref result.Component2;
                ltw.value = ltwValues[(uint)result.entity];
            }
        }

        private Matrix4x4 CalculateLocalToParent(eint entity)
        {
            ref Position position = ref world.GetComponentRef<Position>(entity, out bool hasPosition);
            ref EulerAngles eulerAngles = ref world.GetComponentRef<EulerAngles>(entity, out bool hasEulerAngles);
            ref Rotation rotation = ref world.GetComponentRef<Rotation>(entity, out bool hasRotation);
            ref Scale scale = ref world.GetComponentRef<Scale>(entity, out bool hasScale);
            Matrix4x4 ltp = Matrix4x4.Identity;
            if (hasScale)
            {
                ltp *= Matrix4x4.CreateScale(scale.value);
            }

            if (hasEulerAngles)
            {
                ltp *= Matrix4x4.CreateFromQuaternion(eulerAngles.AsQuaternion());
            }

            if (hasRotation)
            {
                ltp *= Matrix4x4.CreateFromQuaternion(rotation.value);
            }

            if (hasPosition)
            {
                ltp *= Matrix4x4.CreateTranslation(position.value);
            }

            return ltp;
        }
    }
}
