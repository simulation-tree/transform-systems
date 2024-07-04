using System.Numerics;
using Transforms.Components;
using Transforms.Events;
using Unmanaged.Collections;

namespace Game.Systems
{
    public class TransformSystem : SystemBase
    {
        private readonly Query<IsTransform> transformQuery;
        private readonly Query<IsTransform, LocalToWorld> ltwQuery;
        private readonly UnmanagedArray<EntityID> parentEntities;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedList<UnmanagedList<uint>> sortedEntities;

        public TransformSystem(World world) : base(world)
        {
            Subscribe<TransformUpdate>(Update);
            transformQuery = new(world);
            ltwQuery = new(world);
            parentEntities = new();
            ltwValues = new();
            sortedEntities = new();
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
            transformQuery.Fill();
            parentEntities.Resize(transformQuery.Count + 1);
            parentEntities.Clear();
            ltwValues.Resize(transformQuery.Count + 1);
            ltwValues.Clear();

            //calculate ltp of all entities first
            sortedEntities.Clear();
            foreach (Query<IsTransform>.Result result in transformQuery)
            {
                EntityID parent = world.GetParent(result.entity);
                parentEntities[result.entity.value] = parent;
                ltwValues[result.entity.value] = CalculateLocalToParent(result.entity);

                //calculate how deep the entity is
                uint depth = 0;
                EntityID current = parent;
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
                entities.Add(result.entity.value);

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
                    EntityID parent = parentEntities[entity];
                    ref Matrix4x4 ltp = ref ltwValues.GetRef(entity);
                    if (parent != default)
                    {
                        ref Matrix4x4 parentLtp = ref ltwValues.GetRef(parent.value);
                        ltp *= parentLtp;
                    }
                }

                entities.Clear();
            }

            //apply ltw values
            ltwQuery.Fill();
            foreach (Query<IsTransform, LocalToWorld>.Result result in ltwQuery)
            {
                ref LocalToWorld ltw = ref result.Component2;
                ltw.value = ltwValues[result.entity.value];
            }
        }

        private Matrix4x4 CalculateLocalToParent(EntityID entity)
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
