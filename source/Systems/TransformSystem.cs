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
        private readonly Query<IsTransform, LocalToWorld> ltwQuery;
        private readonly UnmanagedArray<eint> parentEntities;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedList<UnmanagedList<uint>> sortedEntities;

        public TransformSystem(World world) : base(world)
        {
            Subscribe<TransformUpdate>(Update);
            transformQuery = new(world);
            ltwQuery = new(world);
            parentEntities = UnmanagedArray<eint>.Create();
            ltwValues = UnmanagedArray<Matrix4x4>.Create();
            sortedEntities = UnmanagedList<UnmanagedList<uint>>.Create();
        }

        public override void Dispose()
        {
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                entities.Dispose();
            }

            sortedEntities.Dispose();
            ltwValues.Dispose();
            parentEntities.Dispose();
            ltwQuery.Dispose();
            transformQuery.Dispose();
            base.Dispose();
        }

        private void Update(TransformUpdate e)
        {
            //clear state
            transformQuery.Update();
            parentEntities.Resize(world.MaxEntityValue + 1);
            parentEntities.Clear();
            ltwValues.Resize(world.MaxEntityValue + 1);
            ltwValues.Clear();

            //reset entities list
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            //calculate ltp of all entities first
            foreach (Query<IsTransform>.Result result in transformQuery)
            {
                eint parent = world.GetParent(result.entity);
                parentEntities[result.entity] = parent;
                ltwValues[result.entity] = CalculateLocalToParent(result.entity);

                //calculate how deep the entity is
                uint depth = 0;
                eint current = parent;
                while (current != default)
                {
                    depth++;
                    current = world.GetParent(current);
                }

                if (sortedEntities.Count <= depth)
                {
                    sortedEntities.Add(new());
                }

                //put the entity into a list located at the index, where the index is the depth
                ref UnmanagedList<uint> entities = ref sortedEntities.GetRef(depth);
                entities.Add(result.entity);

                //make sure it has an ltw component
                if (!world.ContainsComponent<LocalToWorld>(result.entity))
                {
                    world.AddComponent<LocalToWorld>(result.entity, default);
                }
            }

            //calculate ltw from ltp
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                foreach (uint entity in entities)
                {
                    eint parent = parentEntities[entity];
                    ref Matrix4x4 ltp = ref ltwValues.GetRef(entity);
                    if (parent != default)
                    {
                        ref Matrix4x4 parentLtp = ref ltwValues.GetRef(parent);
                        ltp *= parentLtp;
                    }
                }

                entities.Clear();
            }

            //apply ltw values
            ltwQuery.Update();
            foreach (Query<IsTransform, LocalToWorld>.Result result in ltwQuery)
            {
                ref LocalToWorld ltw = ref result.Component2;
                ltw.value = ltwValues[result.entity];
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
