using Collections.Generic;
using Simulation;
using System;
using System.Numerics;
using Transforms.Components;
using Unmanaged;
using Worlds;

namespace Transforms.Systems
{
    public partial class TransformSystem : ISystem, IDisposable
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
        private readonly int positionComponent;
        private readonly int scaleComponent;
        private readonly int rotationComponent;
        private readonly int anchorComponent;
        private readonly int pivotComponent;
        private readonly int ltwComponent;
        private readonly int worldRotationComponent;
        private readonly int transformTag;

        public TransformSystem(Simulator simulator)
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

            Schema schema = simulator.world.Schema;
            positionComponent = schema.GetComponentType<Position>();
            scaleComponent = schema.GetComponentType<Scale>();
            rotationComponent = schema.GetComponentType<Rotation>();
            anchorComponent = schema.GetComponentType<Anchor>();
            pivotComponent = schema.GetComponentType<Pivot>();
            ltwComponent = schema.GetComponentType<LocalToWorld>();
            worldRotationComponent = schema.GetComponentType<WorldRotation>();
            transformTag = schema.GetTagType<IsTransform>();
        }

        public void Dispose()
        {
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Dispose();
            }

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

        void ISystem.Update(Simulator simulator, double deltaTime)
        {
            //ensure capacity is met
            World world = simulator.world;
            int capacity = (world.MaxEntityValue + 1).GetNextPowerOf2();
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

            //reset entities list
            foreach (List<uint> entities in sortedEntities)
            {
                entities.Clear();
            }

            FindComponents(world, positionComponent, rotationComponent, scaleComponent, pivotComponent, anchorComponent, transformTag);
            AddMissingComponents(world, transformTag, ltwComponent, worldRotationComponent);
            CalculateTransforms();

            //apply values
            ApplyValues(world, ltwComponent, worldRotationComponent, transformTag);
        }

        private void CalculateTransforms()
        {
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();
            Span<List<uint>> sortedEntities = this.sortedEntities.AsSpan();

            //calculate ltw in descending order (roots towards leafs)
            //where each entity list is descending in depth
            foreach (List<uint> entities in sortedEntities)
            {
                Span<uint> entitiesSpan = entities.AsSpan();
                foreach (uint entity in entitiesSpan)
                {
                    uint parent = parentEntities[(int)entity];
                    ref LocalToWorld ltw = ref ltwValues[(int)entity];
                    ref Quaternion worldRotation = ref worldRotations[(int)entity];
                    if (parent != default)
                    {
                        LocalToWorld parentLtw = ltwValues[(int)parent];
                        Quaternion parentWorldRotation = worldRotations[(int)parent];
                        if (hasAnchors[(int)entity])
                        {
                            Anchor anchor = anchors[(int)entity];
                            (Vector3 parentPosition, Quaternion _, Vector3 parentSize) = parentLtw.Decomposed;
                            float minX = anchor.minX;
                            float minY = anchor.minY;
                            float minZ = anchor.minZ;
                            float maxX = anchor.maxX;
                            float maxY = anchor.maxY;
                            float maxZ = anchor.maxZ;
                            Anchor.Relativeness flags = anchor.flags;
                            if ((flags & Anchor.Relativeness.MinX) == 0)
                            {
                                minX *= parentSize.X;
                            }

                            if ((flags & Anchor.Relativeness.MinY) == 0)
                            {
                                minY *= parentSize.Y;
                            }

                            if ((flags & Anchor.Relativeness.MinZ) == 0)
                            {
                                minZ *= parentSize.Z;
                            }

                            if ((flags & Anchor.Relativeness.MaxX) == 0)
                            {
                                maxX *= parentSize.X;
                            }
                            else
                            {
                                maxX = parentSize.X - maxX;
                            }

                            if ((flags & Anchor.Relativeness.MaxY) == 0)
                            {
                                maxY *= parentSize.Y;
                            }
                            else
                            {
                                maxY = parentSize.Y - maxY;
                            }

                            if ((flags & Anchor.Relativeness.MaxZ) == 0)
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
                            Vector3 anchorPivot = pivots[(int)entity];
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
        }

        private void FindComponents(World world, int positionComponent, int rotationComponent, int scaleComponent, int pivotComponent, int anchorComponent, int transformTag)
        {
            Span<Vector3> positions = this.positions.AsSpan();
            Span<Vector3> scales = this.scales.AsSpan();
            Span<Quaternion> rotations = this.rotations.AsSpan();
            Span<Vector3> pivots = this.pivots.AsSpan();
            Span<Anchor> anchors = this.anchors.AsSpan();
            Span<bool> hasAnchors = this.hasAnchors.AsSpan();
            Span<uint> parentEntities = this.parentEntities.AsSpan();
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            foreach (Chunk chunk in chunks)
            {
                BitMask componentTypes = chunk.Definition.componentTypes;
                ReadOnlySpan<uint> entities = chunk.Entities;

                if (componentTypes.Contains(positionComponent))
                {
                    ComponentEnumerator<Position> positionComponents = chunk.GetComponents<Position>(positionComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        positions[(int)entity] = positionComponents[i].value;
                    }
                }

                if (componentTypes.Contains(scaleComponent))
                {
                    ComponentEnumerator<Scale> scaleComponents = chunk.GetComponents<Scale>(scaleComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        scales[(int)entity] = scaleComponents[i].value;
                    }
                }

                if (componentTypes.Contains(rotationComponent))
                {
                    ComponentEnumerator<Rotation> rotationComponents = chunk.GetComponents<Rotation>(rotationComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        rotations[(int)entity] = rotationComponents[i].value;
                    }
                }

                if (componentTypes.Contains(pivotComponent))
                {
                    ComponentEnumerator<Pivot> pivotComponents = chunk.GetComponents<Pivot>(pivotComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        pivots[(int)entity] = pivotComponents[i].value;
                    }
                }

                if (componentTypes.Contains(anchorComponent))
                {
                    ComponentEnumerator<Anchor> anchorComponents = chunk.GetComponents<Anchor>(anchorComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        anchors[(int)entity] = anchorComponents[i];
                        hasAnchors[(int)entity] = true;
                    }
                }
            }

            foreach (Chunk chunk in chunks)
            {
                if (chunk.Definition.tagTypes.Contains(transformTag))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    foreach (uint entity in entities)
                    {
                        uint parent = world.GetParent(entity);
                        parentEntities[(int)entity] = parent;

                        //get local to parent matrix first
                        ref Vector3 position = ref positions[(int)entity];
                        ref Quaternion rotation = ref rotations[(int)entity];
                        ref Vector3 scale = ref scales[(int)entity];
                        Matrix4x4 ltp = Matrix4x4.Identity;
                        ltp *= Matrix4x4.CreateScale(scale);
                        ltp *= Matrix4x4.CreateFromQuaternion(rotation);
                        Vector3 pivot = !hasAnchors[(int)entity] ? pivots[(int)entity] : Vector3.Zero;
                        ltp *= Matrix4x4.CreateTranslation(position - pivot * scale);

                        ltwValues[(int)entity] = new(ltp);
                        worldRotations[(int)entity] = rotation;

                        //calculate how deep the entity is
                        int depth = 0;
                        uint current = parent;
                        while (current != default)
                        {
                            depth++;
                            current = world.GetParent(current);
                        }

                        if (sortedEntities.Count <= depth)
                        {
                            int toAdd = depth - sortedEntities.Count;
                            for (int i = 0; i <= toAdd; i++)
                            {
                                sortedEntities.Add(new());
                            }
                        }

                        //put the entity into a list located at the index, where the index is the depth
                        sortedEntities[depth].Add(entity);
                    }
                }
            }
        }

        private void AddMissingComponents(World world, int transformTag, int ltwComponent, int worldRotationComponent)
        {
            //go through all entities without a ltw component, and add it
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            foreach (Chunk chunk in chunks)
            {
                Definition definition = chunk.Definition;
                if (definition.ContainsTag(transformTag) && !definition.ContainsComponent(ltwComponent))
                {
                    operation.SelectEntities(chunk.Entities);
                }
            }

            if (operation.Count > 0)
            {
                operation.AddComponentType<LocalToWorld>();
                operation.ClearSelection();
            }

            //go through all without a world rotation component, and add that too
            bool selectedAny = false;
            chunks = world.Chunks;
            foreach (Chunk chunk in chunks)
            {
                Definition definition = chunk.Definition;
                if (definition.ContainsTag(transformTag) && !definition.ContainsComponent(worldRotationComponent))
                {
                    operation.SelectEntities(chunk.Entities);
                    selectedAny = true;
                }
            }

            if (selectedAny)
            {
                operation.AddComponentType<WorldRotation>();
            }

            if (operation.Count > 0)
            {
                operation.Perform(world);
                operation.Reset();
            }
        }

        private void ApplyValues(World world, int ltwComponent, int worldRotationComponent, int transformTag)
        {
            Span<LocalToWorld> ltwValues = this.ltwValues.AsSpan();
            Span<Quaternion> worldRotations = this.worldRotations.AsSpan();
            BitMask componentMask = new(ltwComponent, worldRotationComponent);
            ReadOnlySpan<Chunk> chunks = world.Chunks;
            foreach (Chunk chunk in chunks)
            {
                Definition definition = chunk.Definition;
                if (definition.tagTypes.Contains(transformTag) && definition.componentTypes.ContainsAll(componentMask))
                {
                    ReadOnlySpan<uint> entities = chunk.Entities;
                    ComponentEnumerator<LocalToWorld> ltwComponents = chunk.GetComponents<LocalToWorld>(ltwComponent);
                    ComponentEnumerator<WorldRotation> worldRotationComponents = chunk.GetComponents<WorldRotation>(worldRotationComponent);
                    for (int i = 0; i < entities.Length; i++)
                    {
                        uint entity = entities[i];
                        ltwComponents[i] = ltwValues[(int)entity];
                        worldRotationComponents[i] = new(worldRotations[(int)entity]);
                    }
                }
            }
        }
    }
}
