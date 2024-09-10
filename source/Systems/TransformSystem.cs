using Simulation;
using System.Numerics;
using Transforms.Components;
using Transforms.Events;
using Unmanaged.Collections;

namespace Transforms.Systems
{
    public class TransformSystem : SystemBase
    {
        private readonly ComponentQuery<IsTransform> transformQuery;
        private readonly ComponentQuery<IsTransform, Pivot> pivotQuery;
        private readonly ComponentQuery<IsTransform, Anchor> anchorsQuery;
        private readonly ComponentQuery<IsTransform, LocalToWorld> ltwQuery;
        private readonly ComponentQuery<IsTransform, WorldRotation> worldRotationQuery;
        private readonly UnmanagedArray<uint> parentEntities;
        private readonly UnmanagedArray<Vector3> pivots;
        private readonly UnmanagedArray<Matrix4x4> ltwValues;
        private readonly UnmanagedArray<Matrix4x4> anchoredLtwValues;
        private readonly UnmanagedArray<Quaternion> worldRotations;
        private readonly UnmanagedList<UnmanagedList<uint>> sortedEntities;

        public TransformSystem(World world) : base(world)
        {
            Subscribe<TransformUpdate>(Update);
            transformQuery = new();
            pivotQuery = new();
            anchorsQuery = new();
            ltwQuery = new();
            worldRotationQuery = new();
            parentEntities = new();
            pivots = new();
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
            pivots.Dispose();
            worldRotationQuery.Dispose();
            ltwQuery.Dispose();
            anchorsQuery.Dispose();
            pivotQuery.Dispose();
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
            pivots.Resize(world.MaxEntityValue + 1);
            pivots.Clear();

            //reset entities list
            foreach (UnmanagedList<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            //find pivot values for later
            pivotQuery.Update(world);
            foreach (var x in pivotQuery)
            {
                pivots[x.entity] = x.Component2.value;
            }

            //calculate ltp of all entities first
            anchorsQuery.Update(world);
            transformQuery.Update(world);
            foreach (var x in transformQuery)
            {
                uint entity = x.entity;
                uint parent = world.GetParent(x.entity);
                parentEntities[entity] = parent;
                ltwValues[entity] = CalculateLocalToParent(x.entity, !anchorsQuery.Contains(x.entity), out Quaternion localRotation);
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
                            float minX = anchor.minX.Number;
                            float minY = anchor.minY.Number;
                            float minZ = anchor.minZ.Number;
                            float maxX = anchor.maxX.Number;
                            float maxY = anchor.maxY.Number;
                            float maxZ = anchor.maxZ.Number;
                            if (!anchor.minX.IsRelative)
                            {
                                minX *= parentSize.X;
                            }

                            if (!anchor.minY.IsRelative)
                            {
                                minY *= parentSize.Y;
                            }

                            if (!anchor.minZ.IsRelative)
                            {
                                minZ *= parentSize.Z;
                            }

                            if (!anchor.maxX.IsRelative)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if (!anchor.maxY.IsRelative)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if (!anchor.maxZ.IsRelative)
                            {
                                maxZ *= parentSize.Z;
                            }
                            else
                            {
                                maxZ = parentSize.Z - maxZ;
                            }

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

                            //affect ltw by pivot
                            Vector3 anchorPivot = pivots[entity];
                            Vector3 pivotOffset = anchorPivot * new LocalToWorld(ltw).Scale;
                            ltw *= Matrix4x4.CreateTranslation(-pivotOffset);
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
            ltwQuery.Update(world);
            foreach (var r in ltwQuery)
            {
                ref LocalToWorld ltw = ref r.Component2;
                ltw.value = ltwValues[r.entity];
            }

            //apply world rotations
            worldRotationQuery.Update(world);
            foreach (var r in worldRotationQuery)
            {
                ref WorldRotation worldRotation = ref r.Component2;
                worldRotation.value = worldRotations[r.entity];
            }
        }

        private Matrix4x4 CalculateLocalToParent(uint entity, bool applyPivot, out Quaternion localRotation)
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
                Vector3 pivot = applyPivot ? pivots[entity] : Vector3.Zero;
                if (hasScale)
                {
                    ltp *= Matrix4x4.CreateTranslation(position.value - pivot * scale.value);
                }
                else
                {
                    ltp *= Matrix4x4.CreateTranslation(position.value - pivot);
                }
            }

            return ltp;
        }
    }
}
