using Collections;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Worlds;

namespace Transforms.Systems
{
    public readonly partial struct TransformSystem : ISystem
    {
        private readonly Array<uint> parentEntities;
        private readonly Array<Vector3> pivots;
        private readonly Array<Anchor> anchors;
        private readonly Array<bool> hasAnchors;
        private readonly Array<Matrix4x4> ltwValues;
        private readonly Array<Matrix4x4> anchoredLtwValues;
        private readonly Array<Quaternion> worldRotations;
        private readonly List<List<uint>> sortedEntities;

        public TransformSystem()
        {
            parentEntities = new();
            pivots = new();
            anchors = new();
            hasAnchors = new();
            ltwValues = new();
            anchoredLtwValues = new();
            worldRotations = new();
            sortedEntities = new();
        }

        void ISystem.Start(in SystemContainer systemContainer, in World world)
        {
            if (systemContainer.World == world)
            {
                systemContainer.Write(new TransformSystem());
            }
        }

        void ISystem.Update(in SystemContainer systemContainer, in World world, in TimeSpan delta)
        {
            Update(world);
        }

        void ISystem.Finish(in SystemContainer systemContainer, in World world)
        {
            if (systemContainer.World == world)
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
                anchors.Dispose();
                hasAnchors.Dispose();
                pivots.Dispose();
            }
        }

        private readonly void Update(World world)
        {
            //ensure capacity is met
            uint capacity = Allocations.GetNextPowerOf2(world.MaxEntityValue + 1);
            if (ltwValues.Length < capacity)
            {
                parentEntities.Length = capacity;
                ltwValues.Length = capacity;
                worldRotations.Length = capacity;
                anchoredLtwValues.Length = capacity;
                pivots.Length = capacity;
                anchors.Length = capacity;
                hasAnchors.Length = capacity;
            }

            //clear state
            parentEntities.Clear();
            ltwValues.Clear();
            worldRotations.Clear();
            anchoredLtwValues.Clear();
            pivots.Clear();
            hasAnchors.Clear();
            anchors.Clear();

            //reset entities list
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            FindPivots(world);
            FindAnchors(world);
            AddMissingComponents(world);
            FindTransforms(world);

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
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
                        if (hasAnchors[entity])
                        {
                            Anchor anchor = anchors[entity];
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

            //apply values
            ApplyValues(world);
        }

        private readonly void FindTransforms(World world)
        {
            ComponentQuery<IsTransform> transformQuery = new(world);
            foreach (var r in transformQuery)
            {
                uint entity = r.entity;
                uint parent = world.GetParent(entity);
                parentEntities[entity] = parent;
                ltwValues[entity] = CalculateLocalToParent(world, entity, !hasAnchors[entity], out Quaternion localRotation);
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
                sortedEntities[depth].Add(entity);
            }
        }

        private readonly void AddMissingComponents(World world)
        {
            using Operation operation = new();
            ComponentQuery<IsTransform> transformWithoutLtwQuery = new(world, ComponentType.GetBitSet<LocalToWorld>());
            foreach (var r in transformWithoutLtwQuery)
            {
                operation.SelectEntity(r.entity);
            }

            if (operation.Count > 0)
            {
                operation.AddComponent<LocalToWorld>();
                world.Perform(operation);
                operation.ClearInstructions();
            }

            ComponentQuery<IsTransform> transformWithoutWorldRotationQuery = new(world, ComponentType.GetBitSet<WorldRotation>());
            foreach (var r in transformWithoutWorldRotationQuery)
            {
                operation.SelectEntity(r.entity);
            }

            if (operation.Count > 0)
            {
                operation.AddComponent<WorldRotation>();
                world.Perform(operation);
            }
        }

        private readonly void FindAnchors(World world)
        {
            ComponentQuery<IsTransform, Anchor> anchorQuery = new(world);
            foreach (var r in anchorQuery)
            {
                ref Anchor anchor = ref r.component2;
                anchors[r.entity] = anchor;
                hasAnchors[r.entity] = true;
            }
        }

        private readonly void FindPivots(World world)
        {
            ComponentQuery<IsTransform, Pivot> pivotQuery = new(world);
            foreach (var r in pivotQuery)
            {
                ref Pivot pivot = ref r.component2;
                pivots[r.entity] = pivot.value;
            }
        }

        private readonly void ApplyValues(World world)
        {
            ComponentQuery<LocalToWorld> ltwQuery = new(world);
            foreach (var r in ltwQuery)
            {
                ref LocalToWorld ltw = ref r.component1;
                ltw.value = ltwValues[r.entity];
            }

            ComponentQuery<WorldRotation> worldRotationQuery = new(world);
            foreach (var r in worldRotationQuery)
            {
                ref WorldRotation worldRotation = ref r.component1;
                worldRotation.value = worldRotations[r.entity];
            }
        }

        private readonly Matrix4x4 CalculateLocalToParent(World world, uint entity, bool applyPivot, out Quaternion localRotation)
        {
            //todo: efficiency: optimize this by fetching all instances of these components ahead of time
            ref Position position = ref world.TryGetComponent<Position>(entity, out bool hasPosition);
            ref EulerAngles eulerAngles = ref world.TryGetComponent<EulerAngles>(entity, out bool hasEulerAngles);
            ref Rotation rotation = ref world.TryGetComponent<Rotation>(entity, out bool hasRotation);
            ref Scale scale = ref world.TryGetComponent<Scale>(entity, out bool hasScale);
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
