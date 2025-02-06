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
        private readonly Array<LocalToWorld> ltwValues;
        private readonly Array<LocalToWorld> anchoredLtwValues;
        private readonly Array<Quaternion> worldRotations;
        private readonly List<List<uint>> sortedEntities;
        private readonly Operation operation;
        private readonly Array<Vector3> positions;
        private readonly Array<Vector3> scales;
        private readonly Array<Quaternion> rotations;
        private readonly Array<Quaternion> eulerAngles;

        private TransformSystem(World world)
        {
            parentEntities = new();
            pivots = new();
            anchors = new();
            hasAnchors = new();
            ltwValues = new();
            anchoredLtwValues = new();
            worldRotations = new();
            sortedEntities = new();
            operation = new();
            positions = new();
            scales = new();
            rotations = new();
            eulerAngles = new();
        }

        void ISystem.Start(in SystemContainer systemContainer, in World world)
        {
            if (systemContainer.World == world)
            {
                systemContainer.Write(new TransformSystem(world));
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

                eulerAngles.Dispose();
                rotations.Dispose();
                scales.Dispose();
                positions.Dispose();
                operation.Dispose();
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
            ComponentType positionComponent = world.Schema.GetComponent<Position>();
            ComponentType scaleComponent = world.Schema.GetComponent<Scale>();
            ComponentType rotationComponent = world.Schema.GetComponent<Rotation>();
            ComponentType eulerAnglesComponent = world.Schema.GetComponent<EulerAngles>();
            ComponentType anchorComponent = world.Schema.GetComponent<Anchor>();
            ComponentType pivotComponent = world.Schema.GetComponent<Pivot>();
            ComponentType ltwComponent = world.Schema.GetComponent<LocalToWorld>();
            ComponentType worldRotationComponent = world.Schema.GetComponent<WorldRotation>();
            TagType transformTag = world.Schema.GetTag<IsTransform>();

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
                positions.Length = capacity;
                scales.Length = capacity;
                rotations.Length = capacity;
                eulerAngles.Length = capacity;
            }

            //clear state
            parentEntities.Clear();
            ltwValues.Fill(LocalToWorld.Default);
            worldRotations.Fill(Rotation.Default.value);
            anchoredLtwValues.Clear();
            pivots.Clear();
            hasAnchors.Clear();
            anchors.Clear();
            positions.Clear();
            scales.Fill(Scale.Default.value);
            rotations.Fill(Rotation.Default.value);
            eulerAngles.Fill(Rotation.Default.value);

            //collect values
            foreach (Chunk chunk in world.Chunks)
            {
                Definition definition = chunk.Definition;
                bool containsPosition = definition.ComponentTypes.Contains(positionComponent);
                bool containsScale = definition.ComponentTypes.Contains(scaleComponent);
                bool containsRotation = definition.ComponentTypes.Contains(rotationComponent);
                bool containsEulerAngles = definition.ComponentTypes.Contains(eulerAnglesComponent);
                if (containsPosition || containsScale || containsRotation || containsEulerAngles)
                {
                    USpan<uint> entities = chunk.Entities;
                    USpan<Position> positionComponents = containsPosition ? chunk.GetComponents<Position>(positionComponent) : default;
                    USpan<Scale> scaleComponents = containsScale ? chunk.GetComponents<Scale>(scaleComponent) : default;
                    USpan<Rotation> rotationComponents = containsRotation ? chunk.GetComponents<Rotation>(rotationComponent) : default;
                    USpan<EulerAngles> eulerAngleComponents = containsEulerAngles ? chunk.GetComponents<EulerAngles>(eulerAnglesComponent) : default;
                    for (uint i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        if (containsPosition)
                        {
                            positions[entity] = positionComponents[i].value;
                        }

                        if (containsScale)
                        {
                            scales[entity] = scaleComponents[i].value;
                        }

                        if (containsRotation)
                        {
                            rotations[entity] = rotationComponents[i].value;
                        }

                        if (containsEulerAngles)
                        {
                            eulerAngles[entity] = eulerAngleComponents[i].AsQuaternion();
                        }
                    }
                }
            }

            //reset entities list
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            FindPivots(world, pivotComponent, transformTag);
            FindAnchors(world, anchorComponent, transformTag);
            AddMissingComponents(world, transformTag);
            FindTransforms(world, transformTag);

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
            foreach (List<uint> entities in sortedEntities)
            {
                foreach (uint entity in entities)
                {
                    uint parent = parentEntities[entity];
                    ref LocalToWorld ltw = ref ltwValues[entity];
                    ref Quaternion worldRotation = ref worldRotations[entity];
                    if (parent != default)
                    {
                        LocalToWorld parentLtw = ltwValues[parent];
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
                            if (!anchor.minX.Absolute)
                            {
                                minX *= parentSize.X;
                            }

                            if (!anchor.minY.Absolute)
                            {
                                minY *= parentSize.Y;
                            }

                            if (!anchor.minZ.Absolute)
                            {
                                minZ *= parentSize.Z;
                            }

                            if (!anchor.maxX.Absolute)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if (!anchor.maxY.Absolute)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if (!anchor.maxZ.Absolute)
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
                            ltw.value *= anchorLtw;

                            //affect ltw by pivot
                            Vector3 anchorPivot = pivots[entity];
                            Vector3 pivotOffset = anchorPivot * ltw.Scale;
                            ltw.value *= Matrix4x4.CreateTranslation(-pivotOffset);
                        }
                        else
                        {
                            ltw.value *= parentLtw.value;
                        }

                        worldRotation = parentWorldRotation * worldRotation;
                    }
                }

                entities.Clear();
            }

            //apply values
            ApplyValues(world, ltwComponent, worldRotationComponent, transformTag);
        }

        private readonly void FindTransforms(World world, TagType transformTag)
        {
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(transformTag)) //todo: replace this with a query
                {
                    foreach (uint entity in chunk.Entities)
                    {
                        Entity e = new(world, entity);
                        uint parent = world.GetParent(entity);
                        parentEntities[entity] = parent;
                        LocalToWorld ltp = CalculateLocalToParent(world, entity, !hasAnchors[entity], out Quaternion localRotation);
                        ltwValues[entity] = ltp;
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
            }
        }

        private readonly void AddMissingComponents(World world, TagType transformTag)
        {
            //go through all entities without a ltw component
            Schema schema = world.Schema;
            ComponentType ltwComponent = schema.GetComponent<LocalToWorld>();
            ComponentType worldRotationComponent = schema.GetComponent<WorldRotation>();
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(transformTag) && !key.Contains(ltwComponent))
                {
                    foreach (uint entity in chunk.Entities)
                    {
                        operation.SelectEntity(entity);
                    }
                }
            }

            if (operation.Count > 0)
            {
                operation.AddComponent<LocalToWorld>(schema);
                operation.ClearSelection();
            }

            //go through all without a world rotation component
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(transformTag) && !key.Contains(worldRotationComponent))
                {
                    foreach (uint entity in chunk.Entities)
                    {
                        operation.SelectEntity(entity);
                    }
                }
            }

            if (operation.HasSelection)
            {
                operation.AddComponent<WorldRotation>(schema);
            }

            if (operation.Count > 0)
            {
                world.Perform(operation);
                operation.Clear();
            }
        }

        private readonly void FindAnchors(World world, ComponentType anchorComponent, TagType transformTag)
        {
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(anchorComponent) && key.Contains(transformTag))
                {
                    USpan<uint> entities = chunk.Entities;
                    USpan<Anchor> components = chunk.GetComponents<Anchor>(anchorComponent);
                    for (uint i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        anchors[entity] = components[i];
                        hasAnchors[entity] = true;
                    }
                }
            }
        }

        private readonly void FindPivots(World world, ComponentType pivotComponent, TagType transformTag)
        {
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(pivotComponent) && key.Contains(transformTag))
                {
                    USpan<uint> entities = chunk.Entities;
                    USpan<Pivot> components = chunk.GetComponents<Pivot>(pivotComponent);
                    for (uint i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        pivots[entity] = components[i].value;
                    }
                }
            }
        }

        private readonly void ApplyValues(World world, ComponentType ltwComponent, ComponentType worldRotationComponent, TagType transformTag)
        {
            foreach (Chunk chunk in world.Chunks)
            {
                Definition key = chunk.Definition;
                if (key.Contains(transformTag) && key.Contains(ltwComponent) && key.Contains(worldRotationComponent))
                {
                    USpan<uint> entities = chunk.Entities;
                    USpan<LocalToWorld> ltwComponents = chunk.GetComponents<LocalToWorld>(ltwComponent);
                    USpan<WorldRotation> worldRotationComponents = chunk.GetComponents<WorldRotation>(worldRotationComponent);
                    for (uint i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        ltwComponents[i] = ltwValues[entity];
                        worldRotationComponents[i] = new(worldRotations[entity]);
                    }
                }
            }
        }

        private readonly LocalToWorld CalculateLocalToParent(World world, uint entity, bool applyPivot, out Quaternion localRotation)
        {
            ref Vector3 position = ref positions[entity];
            ref Quaternion eulerAngle = ref eulerAngles[entity];
            ref Quaternion rotation = ref rotations[entity];
            ref Vector3 scale = ref scales[entity];
            Matrix4x4 ltp = Matrix4x4.Identity;
            localRotation = Quaternion.Identity;
            ltp *= Matrix4x4.CreateScale(scale);
            localRotation = eulerAngle * localRotation;
            localRotation = rotation * localRotation;
            ltp *= Matrix4x4.CreateFromQuaternion(localRotation);
            Vector3 pivot = applyPivot ? pivots[entity] : Vector3.Zero;
            ltp *= Matrix4x4.CreateTranslation(position - pivot * scale);
            return new(ltp);
        }
    }
}
