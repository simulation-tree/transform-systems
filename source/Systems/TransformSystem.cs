using Collections;
using Simulation;
using Simulation.Functions;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Transforms.Components;

namespace Transforms.Systems
{
    public readonly struct TransformSystem : ISystem
    {
        private readonly ComponentQuery<IsTransform> transformQuery;
        private readonly ComponentQuery<IsTransform, Pivot> pivotQuery;
        private readonly ComponentQuery<IsTransform, Anchor> anchorsQuery;
        private readonly ComponentQuery<IsTransform, LocalToWorld> ltwQuery;
        private readonly ComponentQuery<IsTransform, WorldRotation> worldRotationQuery;
        private readonly Array<uint> parentEntities;
        private readonly Array<Vector3> pivots;
        private readonly Array<Matrix4x4> ltwValues;
        private readonly Array<Matrix4x4> anchoredLtwValues;
        private readonly Array<Quaternion> worldRotations;
        private readonly List<List<uint>> sortedEntities;

        readonly unsafe InitializeFunction ISystem.Initialize => new(&Initialize);
        readonly unsafe IterateFunction ISystem.Iterate => new(&Update);
        readonly unsafe FinalizeFunction ISystem.Finalize => new(&Finalize);

        [UnmanagedCallersOnly]
        private static void Initialize(SystemContainer container, World world)
        {
        }

        [UnmanagedCallersOnly]
        private static void Update(SystemContainer container, World world, TimeSpan delta)
        {
            ref TransformSystem system = ref container.Read<TransformSystem>();
            system.Update(world);
        }

        [UnmanagedCallersOnly]
        private static void Finalize(SystemContainer container, World world)
        {
            if (container.World == world)
            {
                ref TransformSystem system = ref container.Read<TransformSystem>();
                system.CleanUp();
            }
        }

        public TransformSystem()
        {
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

        private void CleanUp()
        {
            foreach (List<uint> entities in sortedEntities)
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
        }

        private void Update(World world)
        {
            //clear state
            parentEntities.Length = world.MaxEntityValue + 1;
            parentEntities.Clear();
            ltwValues.Length = world.MaxEntityValue + 1;
            ltwValues.Clear();
            worldRotations.Length = world.MaxEntityValue + 1;
            worldRotations.Clear();
            anchoredLtwValues.Length = world.MaxEntityValue + 1;
            anchoredLtwValues.Clear();
            pivots.Length = world.MaxEntityValue + 1;
            pivots.Clear();

            //reset entities list
            foreach (List<uint> entities in sortedEntities)
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
                uint parent = world.GetParent(entity);
                parentEntities[entity] = parent;
                ltwValues[entity] = CalculateLocalToParent(world, entity, !anchorsQuery.Contains(entity), out Quaternion localRotation);
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
                ref List<uint> entities = ref sortedEntities[depth];
                entities.Add(entity);

                //make sure it has world component
                if (!world.ContainsComponent<LocalToWorld>(entity))
                {
                    world.AddComponent<LocalToWorld>(entity, default);
                }

                if (!world.ContainsComponent<WorldRotation>(entity))
                {
                    world.AddComponent<WorldRotation>(entity, default);
                }
            }

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
            anchorsQuery.Update(world);
            foreach (List<uint> entities in sortedEntities)
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
                            if (!anchor.minX.FromEdge)
                            {
                                minX *= parentSize.X;
                            }

                            if (!anchor.minY.FromEdge)
                            {
                                minY *= parentSize.Y;
                            }

                            if (!anchor.minZ.FromEdge)
                            {
                                minZ *= parentSize.Z;
                            }

                            if (!anchor.maxX.FromEdge)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if (!anchor.maxY.FromEdge)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if (!anchor.maxZ.FromEdge)
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

        private Matrix4x4 CalculateLocalToParent(World world, uint entity, bool applyPivot, out Quaternion localRotation)
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
